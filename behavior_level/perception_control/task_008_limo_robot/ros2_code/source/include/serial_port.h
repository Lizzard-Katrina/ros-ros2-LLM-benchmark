#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <string>
#include <cstdint>
#include <termios.h>
#include <unistd.h>

namespace AgileX {

class SerialPort {
public:
    SerialPort(const std::string& path, uint32_t baudrate)
        : path_(path), baudrate_(baudrate), fd_(-1) {}

    int openPort();
    int closePort();

    std::string getDevPath() const { return path_; }

    ssize_t readByte(uint8_t* data) {
        return read(fd_, data, 1);
    }

    ssize_t writeData(const uint8_t* data, size_t len) {
        return write(fd_, data, len);
    }

private:
    std::string path_;
    uint32_t baudrate_;
    int fd_;
};

}  // namespace AgileX

#endif  // SERIAL_PORT_H_