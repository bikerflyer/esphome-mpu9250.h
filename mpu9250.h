#include "esphome.h"
#include <Wire.h>
#include "MPU9250.h"

// Create MPU9250 instance
MPU9250 mpu(Wire, 0x68);

class MPU9250Sensor : public PollingComponent {
 public:
  Sensor *roll_sensor = new Sensor();
  Sensor *pitch_sensor = new Sensor();
  Sensor *yaw_sensor = new Sensor();
  Sensor *accel_x = new Sensor();
  Sensor *accel_y = new Sensor();
  Sensor *accel_z = new Sensor();
  Sensor *gyro_x = new Sensor();
  Sensor *gyro_y = new Sensor();
  Sensor *gyro_z = new Sensor();
  Sensor *mag_x = new Sensor();
  Sensor *mag_y = new Sensor();
  Sensor *mag_z = new Sensor();

  MPU9250Sensor() : PollingComponent(100) {}

  void setup() override {
    Wire.begin();
    if (mpu.begin() < 0) {
      ESP_LOGE("mpu9250", "MPU9250 init failed!");
      return;
    }
    mpu.setAccelRange(MPU9250::ACCEL_RANGE_4G);
    mpu.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    mpu.setSrd(19); // ~50 Hz
  }

  void update() override {
    mpu.readSensor();

    // Raw accel/gyro/mag
    accel_x->publish_state(mpu.getAccelX_mss());
    accel_y->publish_state(mpu.getAccelY_mss());
    accel_z->publish_state(mpu.getAccelZ_mss());
    gyro_x->publish_state(mpu.getGyroX_rads());
    gyro_y->publish_state(mpu.getGyroY_rads());
    gyro_z->publish_state(mpu.getGyroZ_rads());
    mag_x->publish_state(mpu.getMagX_uT());
    mag_y->publish_state(mpu.getMagY_uT());
    mag_z->publish_state(mpu.getMagZ_uT());

    // Mahony fusion
    mpu.computeOrientation();
    roll_sensor->publish_state(mpu.getRoll());
    pitch_sensor->publish_state(mpu.getPitch());
    yaw_sensor->publish_state(mpu.getYaw());
  }
};
