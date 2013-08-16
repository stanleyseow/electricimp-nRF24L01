electricimp driver nRF24L01
===========================

This is the nRF24L01 driver/library for the electric imp that are based on mirf and 
will communicate directly with RF24 fork for both Arduino, attiny and Raspberry Pi.


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
- RX have not been written yet ( *** )


TODO
====
- RX functions
- RX/RX commands to Arduino
- Send/receive commands to/from Arduino


LINKS
=====

- Blog entries on this setup :- http://arduino-for-beginners.blogspot.com/2013/06/electric-imp-arduino-nrf24l01-fully.html
- RF24 https://github.com/stanleyseow/RF24
- Electric Imp forum http://forums.electricimp.com/discussion/999/nordic-nrf24l01-for-the-electric-imp
- My Github repo : https://github.com/stanleyseow/
- Colour picker : http://acko.net/blog/farbtastic-jquery-color-picker-plug-in/


CONTACT
=======
- Stanley Seow ( stanleyseow@gmail.com )
- My blog : http://arduino-for-beginners.blogspot.com








