.. PicoW_Copter_Docs documentation master file, created by
   sphinx-quickstart on Tue May  9 16:37:36 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to PicoW_Copter_Docs's documentation!
=============================================
This is the Official Documentation for the PicoW Copter
The PicoW Copter is an open source micro sub 60g quadcopter that is controlled using WiFi via a smartphone or computer.
The PicoW Copter is designed to be a low cost, easy to build, and fun to fly quadcopter.

Hardware
========

The Hardware required can be found in the Hardware folder of PicoW Copter project.
The Harware folders contains a BOM, PCB gerber files, and STL file for the frame.
The BOM provided contains links for the parts from online indian websites but you can easily find them for other countries as well.

Bill of Materials
-----------------

.. list-table:: BOM
   :widths: 25 25 50
   :header-rows: 1

   * - Part Name 
     - Quantity 
     - Link
   * - Raspberry Pi Pico W 
     - 1 
     - https://robu.in/product/raspberry-pi-pico-w/ |
   * - MPU6050 
     - 1 
     - https://robu.in/product/mpu-6050-gyro-sensor-2-accelerometer/ |
   * - 1S 360mAh Lipo Battery 
     - 1 
     - https://robu.in/product/orange-360-mah-1s-30c-60c-lithium-polymer-battery-pack-lipo/ |
   * - 720 Coreless Motor 
     - 4 
     - https://robu.in/product/8520-magnetic-micro-coreless-motor-for-micro-quadcopters-2xcw-2xccw/ |
   * - 55mm Propeller (included with motors) 
     - 4 
     - https://robu.in/product/8520-magnetic-micro-coreless-motor-for-micro-quadcopters-2xcw-2xccw/ |
   * - si2302 N channel Mosfet 
     - 4 
     - https://www.ktron.in/product/si2302-mosfet-sot23/ |
   * - 1N4007 Diode 
     - 4 
     - https://robu.in/product/1n4148-surface-mount-zener-diode-pack-of-30/ |
   * - Male Pin headers 
     - 44 
     - https://www.ktron.in/product/header-pins-40x1-2-54mm-pitch-brass/ |
   * - Female Pin headers 
     - 40 
     - https://www.ktron.in/product/berg-strip-female-2-54mm-brass/ |
   * - XH male connector 
     - 1 
     - https://www.ktron.in/product/2pin-jst-xh-male-connector/ |
   * - Push Button 
     - 1 
     - https://www.ktron.in/product/3x6x2-5mm-tactile-switch-smd/ |
   * - BMP 280 
     - 1 
     - https://robu.in/product/bmp280-barometric-pressure-and-altitude-sensor-i2c-spi-module/ |
   * - 1S Lipo charger 
     - 1 
     - https://robu.in/product/4-port-dc5v-1s-rc-lithium-lipo-battery-compact-balance-charger-rc-quadcopter/ |

Taking Rs 160 for 1 PCB and Rs 160 for 3D printed frame from an online service.
The total cost of the hardware is just above 2000 INR or 30 USD.

.. note::
    Most of the hardware won't be available in small quantities so it is adviced to anyone lloking to build this project to try to build multiple of them either for yourself or a group of friends.

PCB
---

.. image::
    images/PCB.png
    :width: 100%
    :align: center
    :alt: PCB

The PCB Gerber files are given in the *PCB_Gerber_PicoW_Copter* folder.
The PCB is a double layered, 4cm X 6cm, 1oz copper, 1.6mm PCB. 

.. image::
  images/Schematic_PicoW_Copter.png
  :width: 100%
  :align: center
  :alt: Schematic

Testing
=======

Testing script are not only useful to test individual hardware components of the PicoW Copter
but also a great way to learn how individual components work.
This project is complex but extremely affordable allowing room for errors and mistakes even for complete beginners.
Anyone looking to modify the code and hardware is more than welcome to do so. 

Receiver
--------

The PicoW Copter uses the onboard infenion WiFi chip of the raspberry Pi PicoW to communicate with a device (smartphone or computer) through UDP packets.
The UDP packets are sent by an APP or software on the device to the PicoW's IP address and port number provided in the Receiver.ino file.
The static IP address when using the Pi Pico over access point mode (hotspot mode) is 192.168.1.42 by default.  
The main motivation for using UDP is real time control of the PicoW Copter. The time taken to read packets is around 200 microseconds.


.. code-block:: arduino
  :linenos:

  #include <WiFi.h>
  #include <WiFiUdp.h>

  #ifndef APSSID
  #define APSSID "PicoW"    // name of your PicoW Hotspot
  #define APPSW "password" // password of your PicoW Hotspot
  #endif
  #define UDP_PKT_MAX_SIZE 16 //  number of characters in a UDP packet

  unsigned int localPort = 8888;  // local port for UDP communication
  char packetBuffer[UDP_PKT_MAX_SIZE + 1];  // max number of characters received in one message
  int Throttle, Roll, Pitch, Yaw; // values received from each channel
  int prev;

  WiFiUDP Udp; // Object for WIFI UDP class

  void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_AP); // Access Point mode
    WiFi.begin(APSSID, APPSW);  // By default static IP for PicoW will be 192.168.42.1
    while(WiFi.status() != WL_CONNECTED) {
      Serial.print('.');  // waiting for connection
      delay(500); // 0.5 sec delay
    }    
    Serial.print("\nConnected! IP address: ");
    Serial.println(WiFi.localIP());   // The IP Address is 192.168.42.1
    Serial.printf("UDP server on port %d\n", localPort);  // Port is 8888
    Udp.begin(localPort); // start listening on port 8888
  }

  void loop() {
    // if there is data available to read then read a packet
    int packetSize = Udp.parsePacket();
    if(packetSize) {  // if packet size is > 0
      prev = micros();
      int n = Udp.read(packetBuffer, UDP_PKT_MAX_SIZE); // read the data from UDP packet into packetBuffer
      packetBuffer[n] = '\0'; // character for end of string
      char ch1[5], ch2[5], ch3[5], ch4[5];  // 
      ch1[4] = '\0'; ch2[4] = '\0'; ch3[4] = '\0'; ch4[4] = '\0';
      for(int i=0; i<4; i++) {
        // Spliting the packets into four values of 4 characters each
        ch1[i] = packetBuffer[i];
        ch2[i] = packetBuffer[i+4];
        ch3[i] = packetBuffer[i+8];
        ch4[i] = packetBuffer[i+12];            
      }    
      // converting string/character arrays to integer
      Yaw = atoi(ch1);
      Throttle = atoi(ch2);
      Roll = atoi(ch3);
      Pitch = atoi(ch4);    
      Serial.printf("Yaw = %d, Throttle = %d, Roll = %d, Pitch = %d\n", Yaw, Throttle, Roll, Pitch);
      Serial.printf("Time taken = %d\n", micros() - prev);
    }   
  }


.. code-block:: python
  :linenos:

  import socket
  import time

  UDP_IP = "192.168.1.42"
  UDP_PORT = 8888
  MESSAGE = "1000100110021003" # sending four 4 digit long numbers

  # creating a UDP socket (UDP is connection less)
  server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  while True:
      server.sendto(MESSAGE.encode('utf-8'), (UDP_IP, UDP_PORT))
      # sleep for 1 second
      time.sleep(1)


Software
========


