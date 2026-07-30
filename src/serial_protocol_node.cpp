/**
 * serial_protocol_node.cpp
 *
 * ANO_LX_FC_PRO OBC protocol bridge.
 *
 * Frame format:
 *   [0xA5][ID][data...][CRC8][0x5B]
 *   CRC8: poly 0x31, init 0x00, over header + ID + payload.
 *
 * ROS -> FC:
 *   0x00  Unlock/Arm, no payload
 *   0x02  Land, no payload
 *   0x03  Task complete, no payload
 *   0x04  Payload release/drop, no payload
 *   0x05  Gripper close/tighten, no payload
 *   0x06  Velocity, 8 bytes: int16 LE vx/vy/vz cm/s + yaw deg/s
 *
 * FC -> ROS:
 *   0x03  Attitude, 7 bytes: roll/pitch/yaw 0.01 deg + state
 *   0x04  Quaternion, 9 bytes: q0-q3 0.0001 + state
 *   0x05  Altitude, 9 bytes: fused/add cm + state
 *   0x07  Velocity, 6 bytes: vx/vy/vz cm/s
 *   0x08  Task select, 1 byte: task id
 *   0x09  Car position, 1 byte: position id
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cstring>
#include <cmath>

#include "serial_protocol/serial_port.hpp"

// ������ CRC8 (poly 0x31, init 0x00) ������������������������������������������������������������������������������������
static uint8_t crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x31;
            else            crc <<= 1;
        }
    }
    return crc;
}

// ������ ֡�������� ������������������������������������������������������������������������������������������������������������������������
/**
 * ����һ֡��д�� buf�����÷���֤ buf �㹻�󣩡�
 * ����֡�ܳ��ȣ�1(head) + 1(id) + data_len + 1(crc) + 1(tail) = data_len + 4
 */
static size_t build_frame(uint8_t* buf, uint8_t id,
                           const uint8_t* data, size_t data_len)
{
    buf[0] = 0xA5;
    buf[1] = id;
    if (data_len > 0) {
        memcpy(buf + 2, data, data_len);
    }
    // CRC8 ���� head + id + data
    buf[2 + data_len] = crc8(buf, 2 + data_len);
    buf[3 + data_len] = 0x5B;
    return 4 + data_len;
}

// ������ ���ڵ� ��������������������������������������������������������������������������������������������������������������������������������
class SerialProtocolNode : public rclcpp::Node
{
public:
    // ���͸��ɿص����� ID���� FC User_Task.c OBC_Recv_Callback ���룩
    static constexpr uint8_t CMD_ARM      = 0x00;   // Unlock
    static constexpr uint8_t CMD_LAND     = 0x02;   // Land
    static constexpr uint8_t CMD_TASK_COMPLETE = 0x03;
    static constexpr uint8_t CMD_PAYLOAD_RELEASE = 0x04;
    static constexpr uint8_t CMD_GRIPPER_CLOSE = 0x05;
    static constexpr uint8_t CMD_VELOCITY = 0x06;
    static constexpr uint8_t CMD_HEARTBEAT = 0xFF;  // ��������ռλ�������͵� FC

    // �ɿط��͸����ǵ�ң�� ID���� FC UserTask_OneKeyCmd ���룩
    static constexpr uint8_t FC_ID_ATTITUDE   = 0x03;
    static constexpr uint8_t FC_ID_QUATERNION = 0x04;
    static constexpr uint8_t FC_ID_ALTITUDE   = 0x05;
    static constexpr uint8_t FC_ID_VELOCITY   = 0x07;
    static constexpr uint8_t FC_ID_TASK_SELECT = 0x08;
    static constexpr uint8_t FC_ID_CAR_POSITION = 0x09;

    enum Priority : int {
        PRIORITY_HEARTBEAT = 0,
        PRIORITY_VELOCITY  = 1,
        PRIORITY_LAND      = 3,
        PRIORITY_ARM       = 4,
    };

    struct ActiveCommand {
        uint8_t  cmd      = CMD_HEARTBEAT;
        int16_t  vel[4]   = {0, 0, 0, 0};   // vx, vy, vz cm/s; yaw dps
        Priority priority = PRIORITY_HEARTBEAT;
        bool     is_one_shot = false;
    };

    SerialProtocolNode() : Node("serial_protocol_node")
    {
        this->declare_parameter<std::string>("port", "/dev/ttyS1");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<double>("send_rate_hz", 50.0);
        this->declare_parameter<double>("vel_timeout", 0.2);

        std::string port = this->get_parameter("port").as_string();
        int baud = this->get_parameter("baud").as_int();

        try {
            serial_ = std::make_unique<SerialPort>(port, baud);
            RCLCPP_INFO(get_logger(), "Opened serial port %s @ %d baud", port.c_str(), baud);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Serial init error: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // ���� ���� ��������������������������������������������������������������������������������������������������������������������
        vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SerialProtocolNode::vel_callback, this, std::placeholders::_1));
        task_running_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/task/running", 10,
            std::bind(&SerialProtocolNode::task_running_callback, this, std::placeholders::_1));

        // ���� ���� ��������������������������������������������������������������������������������������������������������������������
        arm_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/arm",
            std::bind(&SerialProtocolNode::handle_arm, this,
                      std::placeholders::_1, std::placeholders::_2));
        disarm_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/disarm",
            std::bind(&SerialProtocolNode::handle_disarm, this,
                      std::placeholders::_1, std::placeholders::_2));
        land_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/land",
            std::bind(&SerialProtocolNode::handle_land, this,
                      std::placeholders::_1, std::placeholders::_2));
        task_complete_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/task_complete",
            std::bind(&SerialProtocolNode::handle_task_complete, this,
                      std::placeholders::_1, std::placeholders::_2));
        release_payload_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/release_payload",
            std::bind(&SerialProtocolNode::handle_release_payload, this,
                      std::placeholders::_1, std::placeholders::_2));
        gripper_close_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/gripper_close",
            std::bind(&SerialProtocolNode::handle_gripper_close, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ���� �������ɿ�ң�� ������������������������������������������������������������������������������������������������
        // /fc/attitude:  [roll_deg, pitch_deg, yaw_deg, state]
        att_pub_  = create_publisher<std_msgs::msg::Float32MultiArray>("/fc/attitude",   10);
        // /fc/quaternion: [q0, q1, q2, q3, state]
        quat_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/fc/quaternion", 10);
        // /fc/altitude:  [fused_cm, add_cm, state]
        alt_pub_  = create_publisher<std_msgs::msg::Float32MultiArray>("/fc/altitude",   10);
        // /fc/velocity:  Twist��linear.x/y/z = vx/vy/vz m/s��angular = 0��
        fc_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/fc/velocity",        10);
        fc_vel_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/fc/velocity_stamped", 10);
        task_request_pub_ = create_publisher<std_msgs::msg::UInt8>("/fc/task_request", 10);
        car_position_pub_ = create_publisher<std_msgs::msg::UInt8>("/fc/car_position", 10);

        // ���� ���Ͷ�ʱ�� ��������������������������������������������������������������������������������������������������������
        double period = 1.0 / get_parameter("send_rate_hz").as_double();
        send_timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&SerialProtocolNode::timer_callback, this));

        last_vel_time_ = now();
        active_cmd_    = ActiveCommand{};

        // ���� �����߳� ������������������������������������������������������������������������������������������������������������
        rx_running_ = true;
        rx_thread_  = std::thread(&SerialProtocolNode::rx_loop, this);
    }

    ~SerialProtocolNode()
    {
        rx_running_ = false;
        if (rx_thread_.joinable()) rx_thread_.join();

        if (serial_) {
            RCLCPP_INFO(get_logger(), "Sending stop velocity before shutdown...");
            send_velocity(0, 0, 0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            send_velocity(0, 0, 0, 0);
        }
    }

private:
    // ���� �ص� ����������������������������������������������������������������������������������������������������������������������������

    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // ��λ���㣺m/s �� cm/s��rad/s �� deg/s
        desired_vel_[0] = static_cast<int16_t>(msg->linear.x  * 100.0);
        desired_vel_[1] = static_cast<int16_t>(msg->linear.y  * 100.0);
        desired_vel_[2] = static_cast<int16_t>(msg->linear.z  * 100.0);
        desired_vel_[3] = static_cast<int16_t>(msg->angular.z * 180.0 / M_PI);
        last_vel_time_  = now();
        request_command(CMD_VELOCITY, desired_vel_, PRIORITY_VELOCITY, false);
    }

    void task_running_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        const bool running = msg->data;
        if (last_task_running_ && !running) {
            if (send_simple_frame(CMD_TASK_COMPLETE)) {
                RCLCPP_INFO(get_logger(), "Task complete sent to FC (0x03)");
            } else {
                RCLCPP_ERROR(get_logger(), "Task complete serial write failed");
            }
        }
        last_task_running_ = running;
    }

    void handle_arm(const std_srvs::srv::Trigger::Request::SharedPtr,
                    std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        int16_t zero[4] = {0};
        { std::lock_guard<std::mutex> lock(mutex_); request_command(CMD_ARM, zero, PRIORITY_ARM, true); }
        res->success = true;
        res->message = "ARM (Unlock) requested";
    }

    void handle_disarm(const std_srvs::srv::Trigger::Request::SharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        res->success = false;
        res->message = "DISARM is not supported by the new FC OBC protocol; use land/safety landing instead";
    }

    void handle_land(const std_srvs::srv::Trigger::Request::SharedPtr,
                     std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        int16_t zero[4] = {0};
        { std::lock_guard<std::mutex> lock(mutex_); request_command(CMD_LAND, zero, PRIORITY_LAND, true); }
        res->success = true;
        res->message = "LAND requested";
    }

    void handle_task_complete(const std_srvs::srv::Trigger::Request::SharedPtr,
                              std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        res->success = send_simple_frame(CMD_TASK_COMPLETE);
        res->message = res->success ? "Task complete requested (0x03)" : "Task complete serial write failed";
    }

    void handle_release_payload(const std_srvs::srv::Trigger::Request::SharedPtr,
                                std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        res->success = send_simple_frame(CMD_PAYLOAD_RELEASE);
        res->message = res->success ? "Payload release requested (0x04)" : "Payload release serial write failed";
    }

    void handle_gripper_close(const std_srvs::srv::Trigger::Request::SharedPtr,
                              std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        res->success = send_simple_frame(CMD_GRIPPER_CLOSE);
        res->message = res->success ? "Gripper close requested (0x05)" : "Gripper close serial write failed";
    }

    // ���� ������ȣ�����������ã�������������������������������������������������������������������������������������

    void request_command(uint8_t cmd, const int16_t* vel,
                         Priority prio, bool one_shot)
    {
        if (prio >= active_cmd_.priority) {
            active_cmd_.cmd = cmd;
            std::copy(vel, vel + 4, active_cmd_.vel);
            active_cmd_.priority  = prio;
            active_cmd_.is_one_shot = one_shot;
        }
    }

    void timer_callback()
    {
        ActiveCommand cmd_to_send;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            double timeout = get_parameter("vel_timeout").as_double();
            bool vel_active = ((now() - last_vel_time_).seconds() < timeout);

            if (vel_active) {
                request_command(CMD_VELOCITY, desired_vel_, PRIORITY_VELOCITY, false);
            }

            // �ٶȳ�ʱ����һ�������ٻ�����
            if (active_cmd_.cmd == CMD_VELOCITY && !vel_active) {
                bool in_zero_trans = active_cmd_.is_one_shot;
                if (!in_zero_trans) {
                    active_cmd_.vel[0] = active_cmd_.vel[1] =
                    active_cmd_.vel[2] = active_cmd_.vel[3] = 0;
                    active_cmd_.is_one_shot = true;
                } else {
                    active_cmd_.cmd      = CMD_HEARTBEAT;
                    active_cmd_.priority = PRIORITY_HEARTBEAT;
                    active_cmd_.is_one_shot = false;
                }
            }

            cmd_to_send = active_cmd_;
        }

        // ����ռλ���������ɿ�
        if (cmd_to_send.cmd != CMD_HEARTBEAT) {
            do_send(cmd_to_send);
        }

        if (cmd_to_send.is_one_shot) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (active_cmd_.cmd == cmd_to_send.cmd &&
                active_cmd_.priority == cmd_to_send.priority) {
                recompute_base_command();
            }
        }
    }

    void recompute_base_command()
    {
        double timeout   = get_parameter("vel_timeout").as_double();
        bool vel_active  = ((now() - last_vel_time_).seconds() < timeout);
        if (vel_active) {
            active_cmd_.cmd = CMD_VELOCITY;
            std::copy(desired_vel_, desired_vel_ + 4, active_cmd_.vel);
            active_cmd_.priority    = PRIORITY_VELOCITY;
            active_cmd_.is_one_shot = false;
        } else {
            active_cmd_.cmd      = CMD_HEARTBEAT;
            active_cmd_.vel[0]   = active_cmd_.vel[1] =
            active_cmd_.vel[2]   = active_cmd_.vel[3] = 0;
            active_cmd_.priority = PRIORITY_HEARTBEAT;
            active_cmd_.is_one_shot = false;
        }
    }

    // ���� ���� ����������������������������������������������������������������������������������������������������������������������������

    /**
     * ���ɿ� OBC Э�鷢������֡��
     * ���������ARM/DISARM/LAND�����ܳ� 4 �ֽ�
     * �ٶ����0x06�������� 8 �ֽ� = 4��int16_t LE���ܳ� 12 �ֽ�
     */
    void do_send(const ActiveCommand& cmd)
    {
        if (!serial_) return;

        uint8_t buf[16];
        size_t  frame_len = 0;

        switch (cmd.cmd) {
        case CMD_ARM:
        case CMD_LAND:
            frame_len = build_frame(buf, cmd.cmd, nullptr, 0);
            break;

        case CMD_VELOCITY: {
            uint8_t data[8];
            // int16_t little-endian: vel_x, vel_y, vel_z cm/s; yaw_dps
            for (int i = 0; i < 4; ++i) {
                data[i * 2]     = static_cast<uint8_t>(cmd.vel[i] & 0xFF);
                data[i * 2 + 1] = static_cast<uint8_t>((cmd.vel[i] >> 8) & 0xFF);
            }
            frame_len = build_frame(buf, CMD_VELOCITY, data, 8);
            break;
        }
        default:
            return;
        }

        std::ostringstream oss;
        oss << "TX[" << frame_len << "]: ";
        for (size_t i = 0; i < frame_len; ++i)
            oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                << static_cast<int>(buf[i]) << ' ';
        RCLCPP_DEBUG(get_logger(), "%s", oss.str().c_str());

        std::lock_guard<std::mutex> write_lock(serial_write_mutex_);
        if (!serial_->write(buf, frame_len)) {
            RCLCPP_ERROR(get_logger(), "Serial write failed");
        }
    }

    bool send_simple_frame(uint8_t id)
    {
        if (!serial_) return false;
        uint8_t buf[8];
        size_t frame_len = build_frame(buf, id, nullptr, 0);
        std::lock_guard<std::mutex> write_lock(serial_write_mutex_);
        return serial_->write(buf, frame_len);
    }

    /** ��ݷ�װ��ֱ�ӷ��ٶ�֡������ʱ�ã�?*/
    void send_velocity(int16_t vx, int16_t vy, int16_t vz, int16_t yaw_dps)
    {
        ActiveCommand cmd;
        cmd.cmd    = CMD_VELOCITY;
        cmd.vel[0] = vx; cmd.vel[1] = vy;
        cmd.vel[2] = vz; cmd.vel[3] = yaw_dps;
        do_send(cmd);
    }

    // ���� �����߳� ��������������������������������������������������������������������������������������������������������������������

    /**
     * �ɿ� OBC ֡����״̬������Ӧ User_Task.c OBC_Recv_Callback �߼����񣩡�
     * ֡��ʽ��[0xA5][ID][data...][CRC8][0x5B]
     * ң��֡����
     *   0x03 attitude:   7 data bytes �� �� 11 B
     *   0x04 quaternion: 9 data bytes �� �� 13 B
     *   0x05 altitude:   9 data bytes �� �� 13 B
     *   0x07 velocity:   6 data bytes �� �� 10 B
     */
    void rx_loop()
    {
        // ���ջ��壨�㹻�����֡��
        static constexpr size_t BUF_MAX = 64;
        uint8_t raw[BUF_MAX];

        enum class RxState { WAIT_HEAD, WAIT_ID, RECV_DATA, WAIT_CRC, WAIT_TAIL };

        RxState state    = RxState::WAIT_HEAD;
        uint8_t frame[BUF_MAX];
        size_t  frame_len = 0;   // ������ frame[] ���ֽ�����head+id+data��
        size_t  data_expect = 0; // ��ǰ֡ data ���ֽ���

        while (rx_running_) {
            int n = serial_->read(raw, sizeof(raw));
            if (n <= 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            for (int i = 0; i < n; ++i) {
                uint8_t b = raw[i];
                switch (state) {
                case RxState::WAIT_HEAD:
                    if (b == 0xA5) {
                        frame[0] = b;
                        frame_len = 1;
                        state = RxState::WAIT_ID;
                    }
                    break;

                case RxState::WAIT_ID:
                    frame[1]  = b;
                    frame_len = 2;
                    // ���� ID ȷ�����ݶγ���
                    switch (b) {
                    case FC_ID_ATTITUDE:   data_expect = 7; break;
                    case FC_ID_QUATERNION: data_expect = 9; break;
                    case FC_ID_ALTITUDE:   data_expect = 9; break;
                    case FC_ID_VELOCITY:   data_expect = 6; break;
                    case FC_ID_TASK_SELECT: data_expect = 1; break;
                    case FC_ID_CAR_POSITION: data_expect = 1; break;
                    default:
                        // δ֪ ID��������֡
                        state = RxState::WAIT_HEAD;
                        continue;
                    }
                    state = (data_expect > 0) ? RxState::RECV_DATA : RxState::WAIT_CRC;
                    break;

                case RxState::RECV_DATA:
                    if (frame_len < BUF_MAX) frame[frame_len++] = b;
                    // ���� head(1)+id(1)+data(data_expect)
                    if (frame_len == 2 + data_expect) {
                        state = RxState::WAIT_CRC;
                    }
                    break;

                case RxState::WAIT_CRC: {
                    uint8_t expected = crc8(frame, frame_len);
                    if (b == expected) {
                        state = RxState::WAIT_TAIL;
                    } else {
                        RCLCPP_WARN(get_logger(),
                            "CRC8 mismatch for ID 0x%02X: got 0x%02X expected 0x%02X",
                            frame[1], b, expected);
                        state = RxState::WAIT_HEAD;
                    }
                    break;
                }

                case RxState::WAIT_TAIL:
                    if (b == 0x5B) {
                        dispatch_rx_frame(frame[1], frame + 2, data_expect);
                    } else {
                        RCLCPP_WARN(get_logger(), "Missing tail 0x5B for ID 0x%02X", frame[1]);
                    }
                    state = RxState::WAIT_HEAD;
                    break;
                }
            }
        }
    }

    /**
     * �����������ɿ�ң��֡��
     * ������ֵ��ΪС����
     */
    void dispatch_rx_frame(uint8_t id, const uint8_t* data, size_t len)
    {
        switch (id) {
        case FC_ID_ATTITUDE: {
            // 7�ֽڣ�rol/pit/yaw int16 ��0.01�㣻state uint8
            if (len < 7) break;
            int16_t rol_raw, pit_raw, yaw_raw;
            memcpy(&rol_raw, data + 0, 2);
            memcpy(&pit_raw, data + 2, 2);
            memcpy(&yaw_raw, data + 4, 2);
            uint8_t state = data[6];

            auto msg = std_msgs::msg::Float32MultiArray();
            msg.data = {
                rol_raw * 0.01f,
                pit_raw * 0.01f,
                yaw_raw * 0.01f,
                static_cast<float>(state)
            };
            att_pub_->publish(msg);
            RCLCPP_DEBUG(get_logger(), "ATT roll=%.2f pit=%.2f yaw=%.2f state=%u",
                         msg.data[0], msg.data[1], msg.data[2], state);
            break;
        }

        case FC_ID_QUATERNION: {
            // 9�ֽڣ�q0-q3 int16 ��0.0001��state uint8
            if (len < 9) break;
            int16_t q[4];
            for (int i = 0; i < 4; ++i) memcpy(&q[i], data + i * 2, 2);
            uint8_t state = data[8];

            auto msg = std_msgs::msg::Float32MultiArray();
            msg.data = {
                q[0] * 1e-4f, q[1] * 1e-4f,
                q[2] * 1e-4f, q[3] * 1e-4f,
                static_cast<float>(state)
            };
            quat_pub_->publish(msg);
            break;
        }

        case FC_ID_ALTITUDE: {
            // 9�ֽڣ�fused int32 cm��add int32 cm��state uint8
            if (len < 9) break;
            int32_t fused, add;
            memcpy(&fused, data + 0, 4);
            memcpy(&add,   data + 4, 4);
            uint8_t state = data[8];

            auto msg = std_msgs::msg::Float32MultiArray();
            msg.data = {
                static_cast<float>(fused),
                static_cast<float>(add),
                static_cast<float>(state)
            };
            alt_pub_->publish(msg);
            RCLCPP_DEBUG(get_logger(), "ALT fused=%d add=%d state=%u", fused, add, state);
            break;
        }

        case FC_ID_VELOCITY: {
            // 6�ֽڣ�vx/vy/vz int16 cm/s
            if (len < 6) break;
            int16_t vx, vy, vz;
            memcpy(&vx, data + 0, 2);
            memcpy(&vy, data + 2, 2);
            memcpy(&vz, data + 4, 2);

            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = vx * 0.01;
            msg.linear.y = vy * 0.01;
            msg.linear.z = vz * 0.01;
            fc_vel_pub_->publish(msg);

            auto stamped_msg = geometry_msgs::msg::TwistWithCovarianceStamped();
            stamped_msg.header.stamp = now();
            stamped_msg.header.frame_id = "body";
            stamped_msg.twist.twist = msg;
            stamped_msg.twist.covariance[0] = 0.05;
            stamped_msg.twist.covariance[7] = 0.05;
            stamped_msg.twist.covariance[14] = 0.08;
            stamped_msg.twist.covariance[21] = 99999.0;
            stamped_msg.twist.covariance[28] = 99999.0;
            stamped_msg.twist.covariance[35] = 99999.0;
            fc_vel_stamped_pub_->publish(stamped_msg);
            break;
        }

        case FC_ID_TASK_SELECT: {
            if (len < 1) break;
            auto msg = std_msgs::msg::UInt8();
            msg.data = data[0];
            task_request_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "Task select received from FC: task=%u", msg.data);
            break;
        }

        case FC_ID_CAR_POSITION: {
            if (len < 1) break;
            auto msg = std_msgs::msg::UInt8();
            msg.data = data[0];
            car_position_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "Car position received from FC: position=%u", msg.data);
            break;
        }

        default:
            break;
        }
    }

    // ���� ��Ա���� ��������������������������������������������������������������������������������������������������������������������

    std::unique_ptr<SerialPort> serial_;

    // ����
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr task_running_sub_;

    // ����
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr task_complete_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_payload_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_close_srv_;

    // �������ɿ�ң�⣩
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr att_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr quat_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr alt_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        fc_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr fc_vel_stamped_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr task_request_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr car_position_pub_;

    rclcpp::TimerBase::SharedPtr send_timer_;

    std::mutex mutex_;
    std::mutex serial_write_mutex_;
    int16_t    desired_vel_[4]  = {0, 0, 0, 0};
    rclcpp::Time last_vel_time_;
    bool last_task_running_ = false;

    ActiveCommand active_cmd_;

    std::thread       rx_thread_;
    std::atomic<bool> rx_running_{false};
};

// ������ main ������������������������������������������������������������������������������������������������������������������������������������
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialProtocolNode>());
    rclcpp::shutdown();
    return 0;
}
