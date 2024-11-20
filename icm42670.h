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
      void setup() override;
      void dump_config() override;

      void update() override;

      float get_setup_priority() const override;

      void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; }
      void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; }
      void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; }
      void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
      void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; }
      void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor_ = gyro_y_sensor; }
      void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; }
      void set_pitch_sensor(sensor::Sensor *pitch_sensor) { pitch_sensor_ = pitch_sensor; }
      void set_roll_sensor(sensor::Sensor *roll_sensor) { roll_sensor_ = roll_sensor; }
      void set_yaw_sensor(sensor::Sensor *yaw_sensor) { yaw_sensor_ = yaw_sensor; }

    protected:
      sensor::Sensor *accel_x_sensor_{nullptr};
      sensor::Sensor *accel_y_sensor_{nullptr};
      sensor::Sensor *accel_z_sensor_{nullptr};
      sensor::Sensor *temperature_sensor_{nullptr};
      sensor::Sensor *gyro_x_sensor_{nullptr};
      sensor::Sensor *gyro_y_sensor_{nullptr};
      sensor::Sensor *gyro_z_sensor_{nullptr};
      sensor::Sensor *pitch_sensor_{nullptr};
      sensor::Sensor *roll_sensor_{nullptr};
      sensor::Sensor *yaw_sensor_{nullptr};
    };
    ;

  } // namespace icm42670
} // namespace esphome
