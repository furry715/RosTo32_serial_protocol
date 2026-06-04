#ifndef SERIAL_PORT_HPP_
#define SERIAL_PORT_HPP_

#include <string>
#include <stdexcept>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

class SerialPort {
public:
    SerialPort(const std::string& device, int baud_rate) {
        fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open serial port: " + device);
        }

        speed_t speed;
        switch (baud_rate) {
            case 115200: speed = B115200; break;
            case 57600:  speed = B57600; break;
            case 921600: speed = B921600; break;
            default:     speed = B115200;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(fd_, &tty) != 0) {
            throw std::runtime_error("tcgetattr failed");
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        // 8N1, ??????????????
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~(PARENB | CSTOPB);
        // ????? ICRNL ???? 0x0D(\r) ???? 0x0A(\n)
        tty.c_iflag &= ~(INPCK | ISTRIP | IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
        tty.c_oflag &= ~(OPOST | ONLCR | OCRNL);
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;   // 100ms ³¬Ê±

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            throw std::runtime_error("tcsetattr failed");
        }
    }

    ~SerialPort() {
        if (fd_ >= 0) close(fd_);
    }

    bool write(const uint8_t* data, size_t len) {
        return ::write(fd_, data, len) == static_cast<ssize_t>(len);
    }

    int read(uint8_t* buf, size_t max_len) {
        return ::read(fd_, buf, max_len);
    }

private:
    int fd_ = -1;
};

#endif