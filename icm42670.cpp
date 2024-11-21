#include "icm42670.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome
{
  namespace icm42670
  {

    static const char *const TAG = "icm42670";

    const uint8_t ICM42670_REGISTER_WHO_AM_I = 0x75;
    const uint8_t ICM42670_REGISTER_POWER_MANAGEMENT_1 = 0x1F;
    const uint8_t ICM42670_REGISTER_GYRO_CONFIG = 0x4F;
    const uint8_t ICM42670_REGISTER_ACCEL_CONFIG = 0x21;

    const uint8_t ICM42670_REGISTER_ACCEL_XOUT_H = 0x0b;
    const uint8_t ICM42670_REGISTER_GYRO_XOUT_H = 0x11;

    const uint8_t ICM42670_SCALE_2000_DPS = 0b11;
    const float ICM42670_SCALE_DPS_PER_DIGIT_2000 = 0.061f;

    const float ICM42670_RANGE_PER_DIGIT_2G = 16384.0;

    float filtered_ax = 0.0, filtered_ay = 0.0, filtered_az = 0.0;
    float alpha = 0.5;

    void ICM42670Component::setup()
    {
      ESP_LOGV(TAG, "Setting up ICM42670...");
      uint8_t who_am_i;
      this->read_byte(ICM42670_REGISTER_WHO_AM_I, &who_am_i);

      if (!this->read_byte(ICM42670_REGISTER_WHO_AM_I, &who_am_i) ||
          (who_am_i != 0x67))
      {
        this->mark_failed();
        return;
      }

      ESP_LOGV(TAG, "  Setting up Power Management...");
      // Setup power management
      if (!this->write_byte(ICM42670_REGISTER_POWER_MANAGEMENT_1, 0x9F))
      {
        this->mark_failed();
        return;
      }

      ESP_LOGV(TAG, "  Setting up Gyro Config...");
      if (!this->write_byte(ICM42670_REGISTER_GYRO_CONFIG, 0x09))
      {
        this->mark_failed();
        return;
      }

      ESP_LOGV(TAG, "  Setting up Accel Config...");
      if (!this->write_byte(ICM42670_REGISTER_ACCEL_CONFIG, 0x69))
      {
        this->mark_failed();
        return;
      }
    }
    void ICM42670Component::dump_config()
    {
      ESP_LOGCONFIG(TAG, "ICM42670:");
      LOG_I2C_DEVICE(this);
      if (this->is_failed())
      {
        ESP_LOGE(TAG, "Communication with ICM42670 failed!");
      }
      LOG_UPDATE_INTERVAL(this);
      LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
      LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
      LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
      LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
      LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
      LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
      LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
    }

    void ICM42670Component::update()
    {

      ESP_LOGV(TAG, "    Updating ICM42670...");
      uint8_t raw_accel_data[6];
      uint8_t raw_accel_xH, raw_accel_xL, raw_accel_y, raw_accel_z;

      if (!this->read_bytes(ICM42670_REGISTER_ACCEL_XOUT_H, raw_accel_data, 6))
      {
        this->status_set_warning();
        return;
      }

      uint16_t raw_gyro_data[6];
      if (!this->read_bytes_16(ICM42670_REGISTER_GYRO_XOUT_H, raw_gyro_data, 6))
      {
        this->status_set_warning();
        return;
      }

      // Gyro values

      float gyro_x = (int16_t)((raw_gyro_data[0] << 8) | raw_gyro_data[1]);
      float gyro_y = (int16_t)((raw_gyro_data[2] << 8) | raw_gyro_data[3]);
      float gyro_z = (int16_t)((raw_gyro_data[4] << 8) | raw_gyro_data[5]);

      if (this->gyro_x_sensor_ != nullptr)
        this->gyro_x_sensor_->publish_state(gyro_x);

      if (this->gyro_y_sensor_ != nullptr)
        this->gyro_y_sensor_->publish_state(gyro_y);

      if (this->gyro_z_sensor_ != nullptr)
        this->gyro_z_sensor_->publish_state(gyro_z);

      // Accelerometer values

      float accel_x = (int16_t)((raw_accel_data[0] << 8) | raw_accel_data[1]) / ICM42670_RANGE_PER_DIGIT_2G;
      float accel_y = (int16_t)((raw_accel_data[2] << 8) | raw_accel_data[3]) / ICM42670_RANGE_PER_DIGIT_2G;
      float accel_z = (int16_t)((raw_accel_data[4] << 8) | raw_accel_data[5]) / ICM42670_RANGE_PER_DIGIT_2G;

      // Apply low-pass filter
      filtered_ax = alpha * accel_x + (1 - alpha) * filtered_ax;
      filtered_ay = alpha * accel_y + (1 - alpha) * filtered_ay;
      filtered_az = alpha * accel_z + (1 - alpha) * filtered_az;

      if (this->accel_x_sensor_ != nullptr)
        this->accel_x_sensor_->publish_state(accel_x);

      if (this->accel_y_sensor_ != nullptr)
        this->accel_y_sensor_->publish_state(accel_y);

      if (this->accel_z_sensor_ != nullptr)
        this->accel_z_sensor_->publish_state(accel_z);

      float roll = atan2(-filtered_ax, sqrt(filtered_ay * filtered_ay + filtered_az * filtered_az)) * 180.0 / M_PI;
      float pitch = atan2(filtered_ay, sqrt(filtered_ax * filtered_ax + filtered_az * filtered_az)) * 180.0 / M_PI;
      float yaw = atan2(filtered_ay, sqrt(filtered_ax * filtered_ax + filtered_ay * filtered_ay)) * 180.0 / M_PI;

      if (this->pitch_sensor_ != nullptr)
        this->pitch_sensor_->publish_state(pitch);

      if (this->roll_sensor_ != nullptr)
        this->roll_sensor_->publish_state(roll);

      if (this->yaw_sensor_ != nullptr)
        this->yaw_sensor_->publish_state(roll);
    }
    float ICM42670Component::get_setup_priority() const { return setup_priority::DATA; }

  } // namespace ICM42670
} // namespace esphome
