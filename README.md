electricimp driver nRF24L01
===========================

This is the nRF24L01 driver/library for the electric imp that are based on mirf and 
will communicate directly with my RF24 fork for both Arduino and Raspberry Pi.


SETUP
=====

- nRF   CE  ->  Imp 2 / Control RX/TX
- nRF  CSN  ->  Imp 5 / Chip select not
- nRF  SCK  ->  Imp 1 / SPI Clock
- nRF MOSI  ->  Imp 8 / Master-Out
- nRF MISO  ->  Imp 9 / Master-In

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
- My blog : http://arduino-for-beginners@blogspot.com
- My Github repo : https://github.com/stanleyseow/


CONTACT
=======
- Stanley Seow ( stanleyseow@gmail.com )








