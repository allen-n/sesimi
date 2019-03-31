# sesimi
The goal of this project is a defense against fixed code gate attack based on the [OpenSesame Attack](https://github.com/samyk/opensesame).

## Getting Started

### NodeMCU ESP8266 (HiLetgo)
We're using the NodeMCU ESP8266 board from HiLetgo, youtube video to get up and running on this board, along with some info on it, here: [Getting started with NodeMCU (ESP8266 tutorial #1)](https://www.youtube.com/watch?v=p06NNRq5NTU)

Url to add the board to the arduino IDE: http://arduino.esp8266.com/versions/2.5.0/package_esp8266com_index.json

Node MCU Driver code (necessary if building on Windows like I am), the bottom of your board will tell you which driver you need, the 2102 driver is available here: [CP210x USB to UART Bridge VCP Drivers](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers)

### CC1101 Board

We are using the Solu CC1101 Wireless Transceiver Module with spring Antenna for Arduino// Wireless RF Transceiver 315/433/868/915MHZ + spring Antenna Wireless Module, link to purchase [here](https://smile.amazon.com/gp/product/B00XDL9E64/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)

Pinout thanks to [David](https://smile.amazon.com/gp/profile/amzn1.account.AFHOMZSV6FL4BO6DST5TR4UGLGQQ/ref=cm_cr_dp_d_gw_tr?ie=UTF8):
1.GND 2.VCC

3.GDO0 4.CSN

5.SCLK 6.SI

7.SO 8.GDO2

For our particular combination of boards, this mapping looks like the following from CC1101 to NodeMCU ESP8622:
* 1 -> GND
* 2 -> 3V3
* 3 -> GPIO5 (D1)
* 4 -> GPIO15 (D8)
* 5 -> GPIO14 (D5)
* 6 -> GPIO13 (D7)
* 7 -> GPIO12 (D6)
* 8 -> NC
<!-- 
### Prerequisites

In progress FIXME

```
Give examples
```

### Installing

In progress FIXME

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

## Running the tests

In progress FIXME
Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system -->

## Built With

* [arduino-cc1101](https://github.com/veonik/arduino-cc1101) - library for interfacing CC1101 IC with Arduino
* [OpenSesame](https://github.com/samyk/opensesame) - Definition of OpenSesame attack by Samy Kamkar


## Contributing

Send me an email, but at the moment this is just a personal project. Always happy to add contributors!

## Versioning

None at the moment. 

## Authors

* **Allen Nikka** - *Initial work* - [See more here](http://allennikka.com)

<!-- See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project. -->

## License

<!-- This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details -->

## Acknowledgments

* [Samy](https://github.com/samyk) for revitalizing the project after an initial stagnation, reccomending CC1101 chip
* [veoink](https://github.com/veonik) for your arduino - cc1101 library (modified here for use with ESP8622 on NodeMCU)

