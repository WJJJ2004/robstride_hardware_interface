#pragma once

#include <iostream>
#include <fstream>
#include <iomanip>
#include <mutex>


class DataLogger {
public:
  DataLogger();
  ~DataLogger();

private:
  std::mutex file_mutex_;
};
