#pragma once

#include <iostream>
#include <fstream>
#include <iomanip>
#include <mutex>

class Butterworth2ndOrderLPF {
public:
  explicit Butterworth2ndOrderLPF(float cutoff_hz = 23.0f);

  float filter(float input, float dt_sec);
  void reset(float value = 0.0f);

private:
  void updateCoefficients(float sample_rate_hz);

  float cutoff_hz_;
  float q_;

  float b0_;
  float b1_;
  float b2_;
  float a1_;
  float a2_;

  float z1_;
  float z2_;
  bool initialized_;
};

class DataLogger {
public:
  DataLogger();
  ~DataLogger();

private:
  std::mutex file_mutex_;
};
