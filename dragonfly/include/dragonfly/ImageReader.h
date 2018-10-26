#pragma once
#include <memory>
#include <string>
#include <thread>
#include <vector>
namespace PI {
const int image_yuv_size = 1280 * 720 * 3 / 2;

struct ImageData {
  std::vector<uint8_t> data;
  uint64_t tms;
  std::string name;
  ImageData(uint8_t *d, int s, uint64_t t, const char *n) : data(d, d + s), tms(t), name(n) {}
  ImageData(const ImageData &other) : data(other.data.begin(), other.data.end()), tms(other.tms), name(other.name) {}
  ImageData() : data(image_yuv_size, 0){};
};

class ImageReader {
 public:
  ImageReader();
  ImageReader(int num);
  ~ImageReader();
  bool ReadAll(ImageData &front_left, ImageData &back_left, ImageData &back_right, ImageData &front_right);
  bool ReadBack(ImageData &back_left, ImageData &back_right);
  bool ReadFront(ImageData &front_left, ImageData &front_right);
  bool Get_l_f(ImageData &image);
  bool Get_r_f(ImageData &image);
  bool Get_l_b(ImageData &image);
  bool Get_r_b(ImageData &image);
  void TrigThread();

 private:
  int handle = 0;
  bool running_ = false;
  std::thread trigthread;
  void Run();
};
}  // namespace PI
