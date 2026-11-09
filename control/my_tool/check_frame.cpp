#include <librealsense2/rs.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <ctime>
#include <string>

// Streams to turn on: IR left/right, depth, color (same modes as the README launch commands).
rs2::config make_config_high(const char* serial) {
  rs2::config cfg;
  cfg.enable_device(serial);
  cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);
  cfg.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8, 30);
  cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);
  return cfg;
}
rs2::config make_config_low(const char* serial) {
  rs2::config cfg;
  cfg.enable_device(serial);
  cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);
  cfg.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8, 30);
  cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
  return cfg;
}

int main() {
  // Turn on the cameras: one pipeline per camera, picked by serial number.
  rs2::pipeline top, lower;
  top.start(make_config_low("261322071425"));
  lower.start(make_config_low("261322072019"));

  while (true) {
    double start = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
    rs2::frameset a = top.wait_for_frames();
    rs2::frameset b = lower.wait_for_frames();
    double end = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
    const double delay = end - start;
    std::printf("%6.1f ms\n",delay);
    std::fflush(stdout);  // show each line right away, also through a pipe (tee) or into a file
  }
}
