# ESPhome Custom Component for ESP32-S3-BOX-3

This repository contains a custom ESPhome component designed for the ESP32-S3-BOX-3, utilizing the ICM42670 sensor.

## Features
- Integration with the ICM42670(ICM42607P) sensor.
- Easy configuration within ESPhome.
- Supports real-time motion and orientation data.

## Installation
1. Clone this repository into your ESPhome project directory:
    ```bash
    git clone https://github.com/aalaei/esphome-icm42670.git
    ```
2. Add the custom component to your ESPhome configuration:
    ```yaml
    external_components:
      - source:
          type: local
          path: components
        components: [ icm42670 ] 
    ```
or directly 
```yaml
    external_components:
      - source:
          type: git
          url: https://github.com/aalaei/esphome-icm42670.git
          ref: main
        components: [ icm42670 ]
```

## Configuration Example
```yaml
sensor:
  - platform: icm42670
    i2c_id: bus_a
    update_interval: 200ms
    id: icm
    address: 0x68
    accel_x:
      name: "ICM42607P Acceleration X"
      unit_of_measurement: "g"
      accuracy_decimals: 3
    accel_y:
      name: "ICM42607P Acceleration Y"
      unit_of_measurement: "g"
      accuracy_decimals: 3
    accel_z:
      name: "ICM42607P Acceleration Z"
      unit_of_measurement: "g"
      accuracy_decimals: 3
    gyro_x:
      name: "ICM42607P Gyroscope X"
      unit_of_measurement: "°/s"
      accuracy_decimals: 2
    gyro_y:
      name: "ICM42607P Gyroscope Y"
      unit_of_measurement: "°/s"
      accuracy_decimals: 2
    gyro_z:
      name: "ICM42607P Gyroscope Z"
      unit_of_measurement: "°/s"
      accuracy_decimals: 2
    temperature:
      name: "ICM42607P Temperature"
      unit_of_measurement: "°C"
      accuracy_decimals: 2
      device_class: "temperature"
    pitch:
      name: "ICM42607P Pitch"
      unit_of_measurement: "°"
      accuracy_decimals: 2
    roll:
      name: "ICM42607P Roll"
      unit_of_measurement: "°"
      accuracy_decimals: 2


```

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contributions
Contributions are welcome! Feel free to open issues or submit pull requests.

## References
- [ESPhome Documentation](https://esphome.io/)
- [ESP32-S3-BOX-3 Datasheet](https://www.espressif.com/en/products/devkits/esp32-s3-box-3)
- [ICM42670 Datasheet](https://www.invensense.com/products/motion-tracking/icm-42670/)