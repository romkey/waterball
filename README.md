# The Waterball

[![Build Status](https://travis-ci.com/romkey/waterball.svg?branch=master)](https://travis-ci.com/romkey/waterball)

Waterball is an open-source hardware and software project for monitoring hydroponics. 

It's also intended to help with the research and development of HomeBus, an IoT auto-provisioning framework. It's based on the "furball", which is an homage to Dave Mills' ["Fuzzball"](https://en.wikipedia.org/wiki/Fuzzball_router), one of the first routers on the nascent Internet.

The Furball hardware performs environmental monitoring and is intended to be deployed indoors with fixed power source. It will (eventually) auto-provision with HomeBus to feed sensor readings into an IoT system.

## Hardware

Waterball is based on the ESP32 processor. ESP32 is more capable than its predecessor, the ESP8266 while remaining inexpensive. The ESP32 supports both 802.11b/g/n Wifi and Bluetooth 4.2/BLE. It also has hardware acceleration for encryption. It includes 4 SPI controllers, 2 I2C controllers, 3 UARTs, up to 18 channels of ADC and two 8 bit DACs. 

The hardware should support several common environmental sensors in order of importance:
- temperature
- humidity
- air pressure 
- light (lux and a rough break down of red, green and blue intensities)
- water temperature
- water pH
- water level


### Temperature, Humidity and Pressure

Waterball uses the BME280 - temperature/pressure/humidity. $2.57 in single unit quantities, AliExpress.

### Light

TCS34725 RGB sensor

### Water temperature

Dallas 1 wire waterproof temperature sensor

### Total Cost

If bought through AliExpress, parts cost should run roughly:
- $4.50 - ESP32 LOLIN32
- $2.57 - BME280
- $1 - TCS34725
- $1 - Dallas 1 wire waterproof temperature sensor
- $1 - DIYMORE pH sensor
- $1 - random resistors, capacitors

Total of roughly $20 in parts before the circuit board.

## Software

Waterball uses the Arduino Core for the ESP32. It's built using [PlatformIO](https://platformio.org). With some renaming and rearranging it should also build with the Arduino IDE.


## Iterations

The intial version is a simple breadboard.

Second version is a soldered printed circuit breadboard.

The third version will be a custom printed circuit board with modules soldered into it.

The fourth, most ambituous version, will be a custom printed circuit board with surface mount components and will have components mounted directly on it rather than using 3rd party modules.


# License

Software and documentation is licensed under the [MIT license](https://romkey.mit-license.org/).

Circuits are licensed under [Creative Commons Attribution Share-Alike license](https://creativecommons.org/licenses/by-sa/4.0). 
