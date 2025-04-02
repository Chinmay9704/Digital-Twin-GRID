# F401 to ESP Communication Project

This project is designed to interface an STM32 Nucleo-F401RE microcontroller with an ESP8266 module for sensor data collection and communication. The system integrates multiple sensors, processes their data, and sends the results to the ESP8266 for further use.

## Features

- **DHT11 Sensor**: Measures temperature and humidity.
- **IR Speed Sensor**: Calculates RPM (Revolutions Per Minute).
- **Ultrasonic Sensor**: Detects objects within a specified range and counts them.
- **MQ-2 Gas Sensor**: Reads gas concentration levels.
- **ESP8266 Communication**: Sends processed sensor data to the ESP8266 module via UART.

## Project Structure

### Key Files

- **`platformio.ini`**: Configuration file for PlatformIO.
- **`src/main.cpp`**: Main application code for the STM32 microcontroller.
- **`.vscode/`**: Contains configuration files for Visual Studio Code.

## Sensors and Pins

| Sensor               | Pin          | Description                     |
|----------------------|--------------|---------------------------------|
| DHT11                | A1           | Measures temperature & humidity |
| IR Speed Sensor      | D3           | Measures RPM                   |
| Ultrasonic Sensor    | D4 (Trig), D5 (Echo) | Detects objects in range       |
| MQ-2 Gas Sensor      | A0           | Measures gas concentration      |

## How It Works

1. **DHT11 Sensor**: Reads temperature and humidity values. If the readings fail, an error message is displayed.
2. **IR Speed Sensor**: Calculates RPM based on pulses detected. If RPM is zero for less than 15 seconds, it displays 90; otherwise, it displays 0.
3. **Ultrasonic Sensor**: Measures distance to detect objects within a 5-15 cm range. Counts objects and auto-increments the count if no changes are detected for 10 seconds.
4. **MQ-2 Gas Sensor**: Reads analog gas concentration values.
5. **ESP8266 Communication**: Sends formatted sensor data to the ESP8266 module via UART.

## Data Format

The data sent to the ESP8266 is formatted as:

## Setup Instructions

1. **Hardware Setup**:
   - Connect the sensors to the STM32 Nucleo-F401RE as per the pin definitions.
   - Connect the ESP8266 module to UART1 (PA10 for RX, PA9 for TX).

2. **Software Setup**:
   - Install [PlatformIO](https://platformio.org/) in your IDE (e.g., Visual Studio Code).
   - Clone this repository and open it in PlatformIO.
   - Build and upload the firmware to the STM32 Nucleo-F401RE.

3. **MQ-2 Sensor Warm-Up**:
   - Allow the MQ-2 sensor to warm up for 20 seconds after powering on.

## Dependencies

- **DHT Sensor Library**: For DHT11 sensor.
- **Adafruit Unified Sensor Library**: Required by the DHT library.

## Usage

1. Power on the STM32 Nucleo-F401RE.
2. Open the Serial Monitor at 115200 baud to view debug messages.
3. The system will start collecting sensor data and send it to the ESP8266.

## Debugging

- Use the Serial Monitor to view real-time sensor readings and debug messages.
- Ensure all sensors are properly connected and powered.

## Future Improvements

- Add support for additional sensors.
- Implement error handling for ESP8266 communication.
- Optimize power consumption for battery-powered applications.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

## Acknowledgments

- [PlatformIO](https://platformio.org/)
- [Adafruit Unified Sensor Library](https://github.com/adafruit/Adafruit_Sensor)
- [DHT Sensor Library](https://github.com/adafruit/DHT-sensor-library)
