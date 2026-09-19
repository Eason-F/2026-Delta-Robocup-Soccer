#pragma once
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <type_traits>
using std::abs;
constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;
inline float radians(float x) { return x * DEG_TO_RAD; }
inline float degrees(float x) { return x / DEG_TO_RAD; }
template<class A, class B> auto min(A a, B b) -> typename std::common_type<A,B>::type { return a < b ? a : b; }
template<class A, class B> auto max(A a, B b) -> typename std::common_type<A,B>::type { return a > b ? a : b; }
template<class T> T constrain(T x, T lo, T hi) { return min(max(x, lo), hi); }
extern uint32_t testMillis;
inline uint32_t millis() { return testMillis; }
class elapsedMillis {
    uint32_t start = millis();
public:
    operator uint32_t() const { return millis() - start; }
    elapsedMillis &operator=(uint32_t elapsed) { start = millis() - elapsed; return *this; }
};

constexpr int LOW = 0, HIGH = 1, INPUT_PULLUP = 2, INPUT_PULLDOWN = 3;
inline int testPins[64] = {};
inline void pinMode(int, int) {}
inline int digitalRead(int pin) { return testPins[pin]; }
struct TestSerial {
    template<class T> void print(T) {}
    void println() {}
};
inline TestSerial Serial;

inline int digitalReadFast(int pin) { return digitalRead(pin); }
