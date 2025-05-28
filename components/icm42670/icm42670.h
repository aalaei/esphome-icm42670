#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome
{
  namespace icm42670
  {

    class ICM42670Component : public PollingComponent, public i2c::I2CDevice
    {
    public:
      // Setup function, called once at startup
      void setup() override;
      // Dump config function, for logging component configuration
      void dump_config() override;
      // Update function, called periodically based on update_interval
      void update() override;
      // Returns the setup priority for this component
      float get_setup_priority() const override;

      // Setter methods for associating sensor pointers
      void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; }
      void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; }
      void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; }
      void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
      void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; }
      void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor_ = gyro_y_sensor; }
      void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; }
      void set_pitch_sensor(sensor::Sensor *pitch_sensor) { pitch_sensor_ = pitch_sensor; }
      void set_roll_sensor(sensor::Sensor *roll_sensor) { roll_sensor_ = roll_sensor; }
      // Yaw sensor is declared but will not be published to directly from accelerometer data
      // as it requires a magnetometer or complex sensor fusion for reliable heading.
      void set_yaw_sensor(sensor::Sensor *yaw_sensor) { yaw_sensor_ = yaw_sensor; } 
      void set_orientation(uint8_t orientation) { orientation_ = orientation; }
      void set_inverted(bool inverted) { inverted_ = inverted; }

    protected:
      // Sensor pointers for publishing data
      sensor::Sensor *accel_x_sensor_{nullptr};
      sensor::Sensor *accel_y_sensor_{nullptr};
      sensor::Sensor *accel_z_sensor_{nullptr};
      sensor::Sensor *temperature_sensor_{nullptr};
      sensor::Sensor *gyro_x_sensor_{nullptr};
      sensor::Sensor *gyro_y_sensor_{nullptr};
      sensor::Sensor *gyro_z_sensor_{nullptr};
      sensor::Sensor *pitch_sensor_{nullptr};
      sensor::Sensor *roll_sensor_{nullptr};
      sensor::Sensor *yaw_sensor_{nullptr}; // Will not be used for publishing from accel data
      uint8_t orientation_;
      bool inverted_;

      float estimated_pitch_{0.0f};
      float estimated_roll_{0.0f};
      uint32_t last_update_ms_{0}; // To store last update time in milliseconds
    };

  } // namespace icm42670
} // namespace esphome
