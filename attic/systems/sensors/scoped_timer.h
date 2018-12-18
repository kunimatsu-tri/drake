#pragma once

#include <chrono>
#include <iostream>

namespace drake {
namespace systems {
namespace sensors {

class ScopedTimer {
 public:
  explicit ScopedTimer(const std::string& msg, const std::string& indent = "")
      : msg_(msg), indent_(indent) {
    start_ = std::chrono::system_clock::now();
  }

  ~ScopedTimer() {
    auto end = std::chrono::system_clock::now();
    double elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            end - start_).count();
    std::cout << indent_ << msg_ << " : "
              << elapsed << " [ms]" << std::endl;
  }

 private:
  std::string msg_;
  std::string indent_;
  std::chrono::system_clock::time_point start_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
