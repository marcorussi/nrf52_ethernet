# nrf52_ethernet
Adding a KSZ8851 Ethernet interface to nRF52840 DK board.

Connect an Atmel Ethernet1 Xplained Pro board to a nRF52840 DK following the pin map below: 

| Signal | nRF52840 DK | Ethernet board |
| --- | --- | --- |
| SPI_SS_A | P0.11 | 15 |
| SPI_MOSI | P0.12 | 16 |
| SPI_MISO | P0.13 | 17 |
| SPI_SCK | P0.14 | 18 |
| INTRN | P0.05 | 9 |
| nRST | P0.01 | 6 |


**Install**

Install SEGGER JLink and nrfjprog. Clone this repo in your projects directory:

    $ git clone https://github.com/marcorussi/nrf52_ethernet.git

Modify the Makefile according to your environment paths and preferences.


**Flash**

Connect the board, make and flash it:
 
    $ make
    $ make flash


**Known issues**

This is just a quick experiment, so bear in mind that some issues could be present. Feel free to bring this project forward.
* the KSZ8851 Ethernet chip must be unpowered and powered up again to keep receiving messages after a while if incoming  messages are not managed during a debug session (MCU not running). Internal message queue gets probably full and so it should be cleared.


