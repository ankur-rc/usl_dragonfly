#include <PI_Dragonfly.h>
#include <condition_variable>
#include <fstream>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

using namespace std;
inline uint64_t GetCurrentTimeMilliSec() {
  struct timespec t = {};
  clock_gettime(CLOCK_REALTIME, &t);
  return t.tv_nsec / 1000000 + t.tv_sec * 1000;
}

inline uint64_t GetCurrentTimeMicroSec() {
  struct timespec t = {};
  clock_gettime(CLOCK_REALTIME, &t);
  return t.tv_nsec / 1000 + t.tv_sec * 1000000;
}

class ImageWriter {
public:
  explicit ImageWriter(const char *dir);
  ~ImageWriter();

  void Write(PI::ImageData &imageData);
  bool writeable() { return running_; }
  void Stop();

private:
  void flush();
  void Run();
  list<shared_ptr<PI::ImageData>> images_produce;
  list<shared_ptr<PI::ImageData>> images_consume;
  mutex mtx;
  vector<int> cal_fps;
  std::thread image_dump;
  string dir_name;
  bool running_ = false;
  condition_variable cv;

  const vector<string> cam_name{"back_left", "front_left", "back_right",
                                "front_right"};
};

class IMUWriter {
public:
  void Write(PI::IMU_data &data);
  explicit IMUWriter(string dir_name);
  ~IMUWriter();
  void Run();
  void Stop();
  bool writeable() { return running_; }

private:
  void flush();
  std::thread imu_dump;
  list<shared_ptr<PI::IMU_data>> imu_produce;
  bool running_ = true;
  list<shared_ptr<PI::IMU_data>> imu_consume;
  double imu_delay = 4;
  string full_name;
  mutex mtx;
};
