#include "util.h"
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>

class exception_file : public exception {
  virtual const char *what() const throw() { return "file open failed"; }
};
using namespace std;
ImageWriter::ImageWriter(const char *dir) {
  running_ = true;
  dir_name = string(dir);
  image_dump = thread(&ImageWriter::Run, this);
}
ImageWriter::~ImageWriter() {
  if (running_) {
    Stop();
  }
}
void ImageWriter::Stop() {
  running_ = false;
  cv.notify_one();
  image_dump.join();
  try {
    flush();
  } catch (exception &e) {
    std::cout << e.what() << std::endl;
  }
}
void ImageWriter::Write(PI::ImageData &imageData) {
  auto tmp = std::make_shared<PI::ImageData>(imageData);
  lock_guard<std::mutex> lck(mtx);
  if (images_produce.size() > 1000) {
    std::cout << "waiting to write queue too large try to delay 10ms"
              << images_produce.size() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  images_produce.push_back(tmp);
  cv.notify_one();
}
void ImageWriter::flush() {
  while (!images_consume.empty()) {
    if (images_consume.size() > 1000) {
      std::cout << "consumer size too large " << images_consume.size()
                << std::endl;
    }
    auto tmp_image = images_consume.front();
    images_consume.pop_front();
    string filename;
    // to make 123 ---> 0000000000123
    string tms = to_string(tmp_image->tms);
    auto length = 13 - tms.size();
    if (length > 0) {
      tms.insert(tms.begin(), length, '0');
    }
    filename.append(dir_name)
        .append("/")
        .append(tmp_image->name)
        .append("_")
        .append(tms)
        .append(".yuv");

    // use c style file operation to release file source immediatly

    fstream fstream_image(filename, ios::out);
    if (fstream_image.is_open()) {
      fstream_image.write((char *)tmp_image->data.data(),
                          tmp_image->data.size());
      //      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      throw exception_file();
    }

    //    auto fp = fopen(filename.c_str(), "wb");
    //    if (fp) {
    //      fwrite((char *)tmp_image->data.data(), 1, tmp_image->data.size(),
    //      fp); fclose(fp);
    //    } else {
    //      throw exception_file();
    //    }
  }
}
void ImageWriter::Run() {
  while (running_) {
    {
      unique_lock<std::mutex> lck(mtx);
      if (cv.wait_for(lck, std::chrono::milliseconds(1000)) ==
          std::cv_status::timeout)
        continue;
      if (!images_produce.empty()) {
        images_consume.insert(images_consume.end(), images_produce.begin(),
                              images_produce.end());
        images_produce.clear();
      }
    }
    try {
      flush();
    } catch (exception &e) {
      std::cout << e.what() << std::endl;
      running_ = false;
      return;
    }
  }
}
IMUWriter::IMUWriter(string dir_name) : full_name(dir_name.append("/imu.csv")) {
  running_ = true;
  imu_dump = std::thread(&IMUWriter::Run, this);
}
void IMUWriter::Write(PI::IMU_data &data) {
  auto tmp = std::make_shared<PI::IMU_data>(data);
  lock_guard<std::mutex> lck(mtx);
  imu_produce.push_back(tmp);
}
void IMUWriter::flush() {
  while (!imu_consume.empty()) {
    auto data = imu_consume.front();
    imu_consume.pop_front();
    char imu_buff[200] = {};
    snprintf(imu_buff, sizeof(imu_buff), "%ld %ld %f %f %f %ld %f %f %f \n",
             data->tms, data->tms, data->acc_x, data->acc_y, data->acc_z,
             data->tms, data->gyro_x, data->gyro_y, data->gyro_z);

    // write to file
    static fstream fstream_imu(full_name, ios::out);
    if (fstream_imu) {
      fstream_imu.write(imu_buff, strlen(imu_buff));
      fstream_imu.flush();
    } else {
      throw exception_file();
    }
  }
}
IMUWriter::~IMUWriter() {
  if (running_) {
    Stop();
  }
}
void IMUWriter::Stop() {
  running_ = false;
  if (imu_dump.joinable()) {
    imu_dump.join();
  }
  try {
    flush();
  } catch (exception &e) {
    std::cout << e.what() << std::endl;
  }
}
void IMUWriter::Run() {
  while (running_) {
    {
      lock_guard<std::mutex> lck(mtx);
      if (!imu_produce.empty()) {
        imu_consume.insert(imu_consume.begin(), imu_produce.begin(),
                           imu_produce.end());
        imu_produce.clear();
      }
    }
    try {
      flush();
    } catch (exception &e) {
      std::cout << e.what() << std::endl;
      running_ = false;
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}