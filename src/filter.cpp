#include "robstride_rdk_ros2/filter.hpp"

#include <cmath>

Butterworth2ndOrderLPF::Butterworth2ndOrderLPF(float cutoff_hz)
  : cutoff_hz_(cutoff_hz),
    q_(0.70710678f),
    b0_(0.0f),
    b1_(0.0f),
    b2_(0.0f),
    a1_(0.0f),
    a2_(0.0f),
    z1_(0.0f),
    z2_(0.0f),
    initialized_(false)
{
}

void Butterworth2ndOrderLPF::updateCoefficients(float sample_rate_hz)
{
  const float pi = 3.14159265358979323846f;
  float omega = 2.0f * pi * cutoff_hz_ / sample_rate_hz;

  // Avoid omega==pi (Nyquist) which can yield marginally stable coefficients.
  const float omega_max = 0.99f * pi;
  if (omega > omega_max) {
    omega = omega_max;
  }

  const float cos_omega = std::cos(omega);
  const float sin_omega = std::sin(omega);
  const float alpha = sin_omega / (2.0f * q_);

  const float a0 = 1.0f + alpha;
  const float b0 = (1.0f - cos_omega) * 0.5f;
  const float b1 = 1.0f - cos_omega;
  const float b2 = (1.0f - cos_omega) * 0.5f;
  const float a1 = -2.0f * cos_omega;
  const float a2 = 1.0f - alpha;

  b0_ = b0 / a0;
  b1_ = b1 / a0;
  b2_ = b2 / a0;
  a1_ = a1 / a0;
  a2_ = a2 / a0;
}

float Butterworth2ndOrderLPF::filter(float input, float dt_sec)
{
  if (dt_sec <= 0.0f) {
    return input;
  }

  const float sample_rate_hz = 1.0f / dt_sec;
  updateCoefficients(sample_rate_hz);

  if (!initialized_) {
    reset(input);
    initialized_ = true;
    return input;
  }

  const float output = b0_ * input + z1_;
  z1_ = b1_ * input - a1_ * output + z2_;
  z2_ = b2_ * input - a2_ * output;
  return output;
}

void Butterworth2ndOrderLPF::reset(float value)
{
  z1_ = value;
  z2_ = value;
}

DataLogger::DataLogger()
{
    std::cout << "DataLogger created" << std::endl;
}

DataLogger::~DataLogger()
{
    std::cout << "DataLogger destroyed" << std::endl;
}
