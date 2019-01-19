# myAHRS+ / WeMos D1 Mini ESP8266 WiFi Communication

## myAHRS-euler-wifi:

An Arduino project that configures the WiFi of the D1 Mini WiFi board, creates an I2C connection with the myAHRS+ sensor (D1 Mini as MASTER, myAHRS+ as SLAVE). Retrieves Euler angles from the orientation sensor, via I2C protocol, and sends them through UDP packets to a user-specified port (4210 by default) of the local network every 100ms.

## udp-recv.c:

A C program that sniffs a designated port (4210) for UDP packets, parses and displays them as a string. Enables the user to access and observe the Euler angles provided by the myAHRS+ IMU in real time - over a Wifi connection.

Command Line Instructions:    
```
make
./udp-recv
```

To change the port the udp socket is bound to, edit port.h.


(A modified version of Paul Krzyzanowski's [demo-udp-03](https://www.cs.rutgers.edu/~pxk/417/notes/sockets/demo-udp-03.html))
