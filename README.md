# myAHRS+ / WeMos D1 Mini ESP8266 WiFi Communication

## myAHRS-euler-wifi:

An Arduino project that configures the WiFi of the D1 Mini WiFi board, creates an I2C connection with the myAHRS+ sensor.

I2C specification: (D1 Mini WiFi is MASTER, myAHRS+ is SLAVE). Makes use of myAHRS+ I2C register mapping sample code. 

WeMos D1 Mini ESP8266 retrieves Euler angles from the orientation sensor, via I2C protocol, and sends them through UDP packets to a user-specified port (4210 by default) of the local network at a specified rate (default is every 100ms).

The myAHRS+ is capable of up to 1kHz I2C connectivity.

## Installing the myAHRS-euler-wifi project onto the D1 Mini ESP8266 Wifi Board

The WiFi board can be programmed through the Arduino IDE.
```
Step 1: Install the USB-to-SERIAL conversion driver: https://wiki.wemos.cc/downloads (Windows and Mac OS X)
Step 2: Open your Arduino IDE, go to Preferences, paste this link under 'Additional Boards Manager URLs': http://arduino.esp8266.com/versions/2.5.0-beta2/package_esp8266com_index.json
Step 3: In Arduino, go to Tools -> Board -> Boards Manager... Type in esp8266 and download the board package.
```

## udp-recv.c:

A C program that sniffs a designated port (4210) for UDP packets, parses and displays them as a string. Enables the user to access and observe the Euler angles provided by the myAHRS+ IMU in real time - over a Wifi connection.

Command Line Instructions:    
```
make
./udp-recv
```

To change the port the udp socket is bound to, edit port.h.


(A modified version of Paul Krzyzanowski's [demo-udp-03](https://www.cs.rutgers.edu/~pxk/417/notes/sockets/demo-udp-03.html))
