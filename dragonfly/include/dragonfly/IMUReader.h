#pragma once
#include <inttypes.h>
namespace PI {

struct IMU_data {
  double acc_x;
  double acc_y;
  double acc_z;
  double gyro_x;
  double gyro_y;
  double gyro_z;
  uint64_t tms;
  IMU_data(double ax, double ay, double az, double gx, double gy, double gz, uint64_t t)
      : acc_x(ax), acc_y(ay), acc_z(az), gyro_x(gx), gyro_y(gy), gyro_z(gz), tms(t) {}
  IMU_data() = default;
  IMU_data(const IMU_data& other)
      : acc_x(other.acc_x),
        acc_y(other.acc_y),
        acc_z(other.acc_z),
        tms(other.tms),
        gyro_x(other.gyro_x),
        gyro_y(other.gyro_y),
        gyro_z(other.gyro_z) {}
};
class IMUReader {
 public:
  IMUReader();
  ~IMUReader();
  bool ReadIMU(IMU_data&);
  bool isOpen();

 private:
  int imu_handle;
};
}  // namespace PI
