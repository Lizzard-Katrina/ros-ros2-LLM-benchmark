#ifndef __ZMQPP_STUB_HPP__
#define __ZMQPP_STUB_HPP__

// Stub header for zmqpp to allow compilation without the actual library
#include <string>
#include <memory>
#include <cstdint>
#include <cstring>
#include <vector>

namespace zmqpp {

enum class socket_type {
  pub,
  sub
};

class message {
public:
  message() : cursor_(0) {}

  template<typename T>
  message& operator<<(const T& /*value*/) {
    return *this;
  }

  void add_raw(const void* /*data*/, size_t /*len*/) {
  }

  template<typename T>
  message& operator>>(T& /*value*/) {
    return *this;
  }

  size_t read_cursor() const { return cursor_; }

  const void* raw_data(size_t /*part*/) const {
    return nullptr;
  }

private:
  size_t cursor_;
};

class context {
public:
  context() {}
};

class socket {
public:
  socket(context& /*ctx*/, socket_type /*type*/) {}

  void bind(const std::string& /*url*/) {}
  void connect(const std::string& /*url*/) {}
  void subscribe(const std::string& /*topic*/) {}
  void close() {}

  bool send(message& /*msg*/, bool /*dont_block*/ = false) { return true; }
  bool receive(message& /*msg*/, bool /*dont_block*/ = false) { return false; }
};

}  // namespace zmqpp

#endif