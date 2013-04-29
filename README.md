electricimp driver nRF24L01
===========================

This is the nRF24L01 driver/library for the electric imp that are based on mirf and 
will communicate directly with my RF24 fork for both Arduino and Raspberry Pi.


SETUP
=====

//  nRF24  |  Imp  | Function
//  ---------------------------------------
//  CE    ->   2   |  Control RX/TX
//  CSN   ->   5   |  Chip select not
//  SCK   ->   1   |  SPI Clock
//  MOSI  ->   8   |  Master-Out
//  MISO  ->   9   |  Master-In

- Connect pins as per above
- Upload sketch to Arduino / RPi
- You should see "01234" on the Serial Monitor or LCD


KNOWN ISSUES
============
- Currently working for TX only
- RX have not been written yet


TODO
====
- RX functions
- InputPort to Planner
- RX/RX commands to Arduino


LINKS
=====
- RF24 https://github.com/stanleyseow/RF24
- Electric Imp forum http://forums.electricimp.com/discussion/999/nordic-nrf24l01-for-the-electric-imp


CONTACT
=======
- Stanley Seow ( stanleyseow@gmail.com )
- My blog : arduino-for-beginners@blogspot.com
- MY Github repo : github.com/stanleyseow/






