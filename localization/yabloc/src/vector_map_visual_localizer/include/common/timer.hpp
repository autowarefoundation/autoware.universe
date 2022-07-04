#pragma once
#include <chrono>
#include <ostream>

class Timer
{
public:
  Timer() { reset(); }

  void reset() { start = std::chrono::system_clock::now(); }

  long milliSeconds() const
  {
    auto dur = std::chrono::system_clock::now() - start;
    return std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
  }

  long microSeconds() const
  {
    auto dur = std::chrono::system_clock::now() - start;
    return std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
  }

  friend std::ostream & operator<<(std::ostream & os, Timer & t)
  {
    os << t.microSeconds() / 1000.f << "[ms] ";
    return os;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start;
};
