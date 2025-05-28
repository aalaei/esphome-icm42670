// icm42670.cpp
#include "icm42670.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h" // For millis() and get_time()
#include <cmath> // For M_PI and atan2

namespace esphome
{
  namespace icm42670
  {

    static const char *const TAG = "icm42670";

    // ICM-42607-P Register Addresses
    const uint8_t ICM42670_REGISTER_WHO_AM_I = 0x75;         // WHO_AM_I register, should return 0x67
    const uint8_t ICM42670_REGISTER_PWR_MGMT0 = 0x1F;         // Power Management 0 register
    const uint8_t ICM42670_REGISTER_GYRO_CONFIG0 = 0x4F;     // Gyroscope Configuration 0 register
    const uint8_t ICM42670_REGISTER_ACCEL_CONFIG0 = 0x21;    // Accelerometer Configuration 0 register
    const uint8_t ICM42670_REGISTER_TEMP_DATA_H = 0x09;      // Temperature Data High Byte
    const uint8_t ICM42670_REGISTER_ACCEL_DATA_X_H = 0x0B;   // Accelerometer X-axis Data High Byte
    const uint8_t ICM42670_REGISTER_GYRO_DATA_X_H = 0x11;    // Gyroscope X-axis Data High Byte
    const uint8_t ICM42670_REGISTER_DEVICE_CONFIG = 0x11;   // Device Configuration Register for software reset

    // Full-Scale Range and Sensitivity Constants
    // For Gyroscope: 2000 dps (degrees per second)
    // LSB/dps for 2000 dps range is 16.4 LSB/dps (from datasheet, 32768 / 2000)
    const float ICM42670_DPS_PER_LSB_2000 = 1.0f / 16.4f; 

    // For Accelerometer: +/- 2g range
    // LSB/g for 2g range is 16384 LSB/g (from datasheet, 32768 / 2)
    const float ICM42670_G_PER_LSB_2G = 1.0f / 16384.0f;

    // Temperature sensitivity and offset (from datasheet)
    const float ICM42670_TEMP_SENSITIVITY = 132.48f; // LSB/°C
    const float ICM42670_TEMP_OFFSET = 25.0f;        // °C

    // Low-pass filter alpha for raw accelerometer data (used by original code)
    float filtered_ax = 0.0, filtered_ay = 0.0, filtered_az = 0.0;
    float alpha_accel_lp = 0.5; // Alpha for accelerometer low-pass filter (for smoother raw output)

    // --- Complementary Filter Alpha ---
    // This value determines the weighting between gyro (high-pass) and accel (low-pass)
    // Range 0.0 to 1.0.
    // Higher value (e.g., 0.98-0.995): Trusts gyro more. Faster response, but more drift/noise.
    // Lower value (e.g., 0.90-0.95): Trusts accel more. Smoother, less drift, but slower response.
    // For smoother and slower changes, try decreasing this value (e.g., 0.95f or 0.90f).
    const float COMPLEMENTARY_FILTER_ALPHA = 0.94f; 
    // --- END Complementary Filter Alpha ---

    void ICM42670Component::setup()
    {
      ESP_LOGV(TAG, "Setting up ICM42670...");

      delay(200); 

      ESP_LOGV(TAG, "  Attempting to set Power Management (PWR_MGMT0) to active mode...");
      if (!this->write_byte(ICM42670_REGISTER_PWR_MGMT0, 0x8F))
      {
        ESP_LOGE(TAG, "Failed to set initial PWR_MGMT0 register. This might prevent WHO_AM_I from working.");
      }
      delay(50); 

      ESP_LOGV(TAG, "  Performing software reset...");
      if (!this->write_byte(ICM42670_REGISTER_DEVICE_CONFIG, 0x01)) {
        ESP_LOGE(TAG, "Failed to perform software reset.");
        this->mark_failed();
        return;
      }
      delay(100); 

      uint8_t who_am_i = 0x00; 
      bool success = false;
      const int max_retries = 5; 
      
      for (int i = 0; i < max_retries; ++i) {
        ESP_LOGV(TAG, "Attempting to read WHO_AM_I (retry %d/%d)...", i + 1, max_retries);
        if (this->read_byte(ICM42670_REGISTER_WHO_AM_I, &who_am_i)) {
          // Changed expected WHO_AM_I to 0x60 based on your discovery for production board
          if (who_am_i == 0x60) { 
            success = true;
            break; 
          } else {
            ESP_LOGW(TAG, "  WHO_AM_I mismatch on retry %d: Expected 0x60, got 0x%02X", i + 1, who_am_i);
          }
        } else {
          ESP_LOGW(TAG, "  Failed to read WHO_AM_I register on retry %d.", i + 1);
        }
        delay(20); 
      }

      if (!success)
      {
        ESP_LOGE(TAG, "WHO_AM_I check failed after %d retries. Final WHO_AM_I: 0x%02X. Communication with ICM42670 failed!", max_retries, who_am_i);
        this->mark_failed();
        return;
      }
      ESP_LOGI(TAG, "WHO_AM_I: 0x%02X - ICM42670 detected successfully!", who_am_i);

      ESP_LOGV(TAG, "  Setting up Power Management (PWR_MGMT0)...");
      if (!this->write_byte(ICM42670_REGISTER_PWR_MGMT0, 0x8F))
      {
        ESP_LOGE(TAG, "Failed to set PWR_MGMT0 register after WHO_AM_I. This might affect sensor operation.");
        this->mark_failed(); 
        return;
      }
      ESP_LOGV(TAG, "  PWR_MGMT0 set to 0x8F (Accel/Gyro Low-Noise Mode, Temp Enabled)");

      ESP_LOGV(TAG, "  Setting up Gyro Config (GYRO_CONFIG0)...");
      if (!this->write_byte(ICM42670_REGISTER_GYRO_CONFIG0, 0x01))
      {
        ESP_LOGE(TAG, "Failed to set GYRO_CONFIG0 register.");
        this->mark_failed();
        return;
      }
      ESP_LOGV(TAG, "  GYRO_CONFIG0 set to 0x01 (+/- 2000 dps, 1 kHz ODR)");

      ESP_LOGV(TAG, "  Setting up Accel Config (ACCEL_CONFIG0)...");
      if (!this->write_byte(ICM42670_REGISTER_ACCEL_CONFIG0, 0x19))
      {
        ESP_LOGE(TAG, "Failed to set ACCEL_CONFIG0 register.");
        this->mark_failed();
        return;
      }
      ESP_LOGV(TAG, "  ACCEL_CONFIG0 set to 0x19 (+/- 2g, 1 kHz ODR)");

      // --- NEW: Initialize filter state with initial accelerometer readings ---
      uint8_t initial_raw_data[14]; 
      if (this->read_bytes(ICM42670_REGISTER_TEMP_DATA_H, initial_raw_data, 14)) {
          int16_t initial_raw_accel_x = (int16_t)((initial_raw_data[2] << 8) | initial_raw_data[3]);
          int16_t initial_raw_accel_y = (int16_t)((initial_raw_data[4] << 8) | initial_raw_data[5]);
          int16_t initial_raw_accel_z = (int16_t)((initial_raw_data[6] << 8) | initial_raw_data[7]);
          
          float initial_ax = initial_raw_accel_x * ICM42670_G_PER_LSB_2G;
          float initial_ay = initial_raw_accel_y * ICM42670_G_PER_LSB_2G;
          float initial_az = initial_raw_accel_z * ICM42670_G_PER_LSB_2G;

          // Initial pitch and roll from accelerometer
          this->estimated_pitch_ = atan2(initial_ay, sqrt(initial_ax * initial_ax + initial_az * initial_az)) * 180.0f / M_PI;
          this->estimated_roll_ = atan2(-initial_ax, initial_az) * 180.0f / M_PI;
          ESP_LOGD(TAG, "Initial Filter State - Pitch: %.2f, Roll: %.2f", this->estimated_pitch_, this->estimated_roll_);
      } else {
          ESP_LOGW(TAG, "Failed to read initial accel data for filter initialization. Filter will start from 0.");
      }
      this->last_update_ms_ = millis(); // Initialize last_update_ms_ after initial data read
      // --- END NEW ---
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
      LOG_SENSOR("  ", "Pitch", this->pitch_sensor_);
      LOG_SENSOR("  ", "Roll", this->roll_sensor_);
      ESP_LOGCONFIG(TAG, "  Orientation: %d", this->orientation_);
      ESP_LOGCONFIG(TAG, "  Inverted: %s", this->inverted_ ? "true" : "false");
      ESP_LOGCONFIG(TAG, "  Complementary Filter Alpha: %.2f", COMPLEMENTARY_FILTER_ALPHA);
    }

    void ICM42670Component::update()
    {
      if (this->is_failed())
      {
        return;
      }

      ESP_LOGD(TAG, "Updating ICM42670 sensor data...");

      uint8_t raw_data[14]; 
      if (!this->read_bytes(ICM42670_REGISTER_TEMP_DATA_H, raw_data, 14))
      {
        ESP_LOGW(TAG, "Failed to read sensor data!");
        this->status_set_warning();
        return;
      }

      int16_t raw_temp = (int16_t)((raw_data[0] << 8) | raw_data[1]);
      float temperature_c = (raw_temp / ICM42670_TEMP_SENSITIVITY) + ICM42670_TEMP_OFFSET;
      if (this->temperature_sensor_ != nullptr)
      {
        this->temperature_sensor_->publish_state(temperature_c);
      }

      int16_t raw_accel_x = (int16_t)((raw_data[2] << 8) | raw_data[3]);
      int16_t raw_accel_y = (int16_t)((raw_data[4] << 8) | raw_data[5]);
      int16_t raw_accel_z = (int16_t)((raw_data[6] << 8) | raw_data[7]);

      float accel_x = raw_accel_x * ICM42670_G_PER_LSB_2G;
      float accel_y = raw_accel_y * ICM42670_G_PER_LSB_2G;
      float accel_z = raw_accel_z * ICM42670_G_PER_LSB_2G;

      // Apply low-pass filter for smoother raw accel output (if needed for direct publishing)
      filtered_ax = alpha_accel_lp * accel_x + (1 - alpha_accel_lp) * filtered_ax;
      filtered_ay = alpha_accel_lp * accel_y + (1 - alpha_accel_lp) * filtered_ay;
      filtered_az = alpha_accel_lp * accel_z + (1 - alpha_accel_lp) * filtered_az;

      if (this->accel_x_sensor_ != nullptr)
        this->accel_x_sensor_->publish_state(accel_x); // Publish raw accel data
      if (this->accel_y_sensor_ != nullptr)
        this->accel_y_sensor_->publish_state(accel_y);
      if (this->accel_z_sensor_ != nullptr)
        this->accel_z_sensor_->publish_state(accel_z);

      int16_t raw_gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
      int16_t raw_gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
      int16_t raw_gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]);

      float gyro_x = raw_gyro_x * ICM42670_DPS_PER_LSB_2000;
      float gyro_y = raw_gyro_y * ICM42670_DPS_PER_LSB_2000;
      float gyro_z = raw_gyro_z * ICM42670_DPS_PER_LSB_2000;

      if (this->gyro_x_sensor_ != nullptr)
        this->gyro_x_sensor_->publish_state(gyro_x);
      if (this->gyro_y_sensor_ != nullptr)
        this->gyro_y_sensor_->publish_state(gyro_y);
      if (this->gyro_z_sensor_ != nullptr)
        this->gyro_z_sensor_->publish_state(gyro_z);

      // --- Complementary Filter Calculation ---

      // Calculate time difference (dt) in seconds
      uint32_t current_time_ms = millis();
      float dt = (current_time_ms - this->last_update_ms_) / 1000.0f;
      this->last_update_ms_ = current_time_ms;

      // Calculate accelerometer-derived pitch and roll (in degrees)
      // These are 'noisy' but gravity-aligned reference angles
      // Ensure accel_z is not zero to avoid division by zero in atan2
      float accel_pitch_deg = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0f / M_PI;
      float accel_roll_deg = atan2(-accel_x, accel_z) * 180.0f / M_PI;

      // Integrate gyroscope data to predict new pitch and roll
      float gyro_pitch_change = gyro_y * dt;
      float gyro_roll_change = gyro_x * dt;

      // Apply complementary filter:
      // estimated_angle = alpha * (estimated_angle + gyro_change) + (1 - alpha) * accel_angle
      this->estimated_pitch_ = COMPLEMENTARY_FILTER_ALPHA * (this->estimated_pitch_ + gyro_pitch_change) + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_pitch_deg;
      this->estimated_roll_ = COMPLEMENTARY_FILTER_ALPHA * (this->estimated_roll_ + gyro_roll_change) + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_roll_deg;

      // Apply inversion directly to filtered values
      float final_pitch = this->estimated_pitch_;
      float final_roll = this->estimated_roll_;
      
      if (this->inverted_)
      {
        final_pitch = -final_pitch;
        final_roll = -final_roll;
      }

      if (this->pitch_sensor_ != nullptr)
        this->pitch_sensor_->publish_state(final_pitch);

      if (this->roll_sensor_ != nullptr)
        this->roll_sensor_->publish_state(final_roll);

      // --- END Complementary Filter Calculation ---
    }

    float ICM42670Component::get_setup_priority() const { return setup_priority::BUS; }

  } // namespace icm42670
} // namespace esphome
