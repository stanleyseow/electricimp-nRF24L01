/* 
 
 Electric Imp driver/library for the nRF24L01+                        
 Author: Stanley Seow                                         
 Email: stanleyseow@gmail.com
 Date : 14 Apr 2013
 
 Version : 0.50 ( completed radio.sent() )
 Repo : https://github.com/stanleyseow/electricimp-nRF24L01
 
 Desc : This driver uses mostly mirf libraries functions as
 it is much simpler to implement but will maintain compatibility
 with RF24 library with dynamic payload enabled and other add-on
 features.
 
 Some of the initial structure was copied from 
 https://github.com/sbolel/nrf24_imp but 
 I totally re-wrote most of the functions.
 
 I do not use much of the OOP as I am still
 not very familier with OOP stuff yet.
 
 I will put up as much debugging output as possible
 but these output display can be turn off during
 production
 
 Stanley
 
*/

// Memory Map 
const CONFIG        = 0x00;
const EN_AA         = 0x01;
const EN_RXADDR     = 0x02;
const SETUP_AW      = 0x03;
const SETUP_RETR    = 0x04;
const RF_CH         = 0x05;
const RF_SETUP      = 0x06;
const STATUS        = 0x07;
const OBSERVE_TX    = 0x08;
const CD            = 0x09;
const RX_ADDR_P0    = 0x0A;
const RX_ADDR_P1    = 0x0B;
const RX_ADDR_P2    = 0x0C;
const RX_ADDR_P3    = 0x0D;
const RX_ADDR_P4    = 0x0E;
const RX_ADDR_P5    = 0x0F;
const TX_ADDR       = 0x10;
const RX_PW_P0      = 0x11;
const RX_PW_P1      = 0x12;
const RX_PW_P2      = 0x13;
const RX_PW_P3      = 0x14;
const RX_PW_P4      = 0x15;
const RX_PW_P5      = 0x16;
const FIFO_STATUS   = 0x17;
const DYNPD         = 0x1C;
const FEATURE       = 0x1D;

// Bit Mnemonics 
const MASK_RX_DR    = 6;
const MASK_TX_DS    = 5;
const MASK_MAX_RT   = 4;
const EN_CRC        = 3;
const CRC0          = 2;
const PWR_UP        = 1;
const PRIM_RX       = 0;
const ENAA_P5       = 5;
const ENAA_P4       = 4;
const ENAA_P3       = 3;
const ENAA_P2       = 2;
const ENAA_P1       = 1;
const ENAA_P0       = 0;
const ERX_P5        = 5;
const ERX_P4        = 4;
const ERX_P3        = 3;
const ERX_P2        = 2;
const ERX_P1        = 1;
const ERX_P0        = 0;
const AW            = 0;
const ARD           = 4;
const ARC           = 0;
const PLL_LOCK      = 4;
const RF_DR         = 3;
const RF_PWR        = 1;
const LNA_HCURR     = 0;
const RX_DR         = 6;
const TX_DS         = 5;
const MAX_RT        = 4;
const RX_P_NO       = 1;
const TX_FULL       = 0;
const PLOS_CNT      = 4;
const ARC_CNT       = 0;
const TX_REUSE      = 6;
const FIFO_FULL     = 5;
const TX_EMPTY      = 4;
const RX_FULL       = 1;
const RX_EMPTY      = 0;
const DPL_P5        = 5;
const DPL_P4	    = 4;
const DPL_P3	    = 3;
const DPL_P2	    = 2;
const DPL_P1	    = 1;
const DPL_P0	    = 0;
const EN_DPL	    = 2;
const EN_ACK_PAY    = 1;
const EN_DYN_ACK    = 0;


// Instruction Mnemonics 
const R_REGISTER    = 0x00;
const W_REGISTER    = 0x20;
const REGISTER_MASK = 0x1F;
const R_RX_PAYLOAD  = 0x61;
const W_TX_PAYLOAD  = 0xA0;
const FLUSH_TX      = 0xE1;
const FLUSH_RX      = 0xE2;
const REUSE_TX_PL   = 0xE3;
const NOP           = 0xFF;

// Pipe Addresses for RX & TX 
const pipes0 = "\xE3\xF0\xF0\xF0\xF0";
const pipes1 = "\xE2\xF0\xF0\xF0\xF0";

local buff          = "1,2,3,4,5";

// RF24 Library Functions 
// Only used in one function, might removed it completely
// function _BV(x) { return (1<<(x)); }

// DOCUMENTATION: PINOUT FOR IMP to nRF24L01
//////////////////////////////////////////////////
//  nRF24  |  Imp  | Function
//  ---------------------------------------
//  CE    ->   2   |  Controles RX/TX
//  CSN   ->   5   |  Chip select not
//  SCK   ->   1   |  SPI Clock
//  MOSI  ->   8   |  Master-out-slave-in
//  MISO  ->   9   |  Master-in-slave-out



//********************************************************************************
//    RF24 CLASS (nRF24L01 Transciever
//********************************************************************************

class RF24 {
  /* Declare null variables */
  ce_pin = null;
  cs_pin = null;
  so_pin = null;
  spiSetup_flags = null;
  spiSetup_clock = null;

  rf_setup = null;
  channel = null;
  pmode = null;
  txmode = null;
  payloadSize = null;
  localAddress = null;
  remoteAddress = null;

//----------------------------------------------------------------------------

  constructor(clock)                    // RF24 Class - Constructor 
  {
    /* Initialize Instance Variables */

    rf_setup       = 0x6;               // Rate = 1Mbps, Power is Max
    channel        = 0x4c;              // CH  = 0x4c / 76
    payloadSize    = 0x20;              // Max is 32 bytes / 0x20
    localAddress   = 0;                 // LA  = 0b0000 (Unimplemented)
    remoteAddress  = 0;                 // RA  = 0b0000 (Unimplemented)
    spiSetup_flags = 0;                 // SPI Flags = 0b0000 (Master)
    spiSetup_clock = clock;             // SPI Clock Speed = input
    txmode         = 1                  // Txmode

    ce_pin = hardware.pin2;             // Chip Enable on pin2 - Active high (While RX)
    cs_pin = hardware.pin5;             // Chip Select Not on pin5 - Active low (SPI)

    cs_pin.configure(DIGITAL_OUT);    // CSN pin = digital out
    ce_pin.configure(DIGITAL_OUT);    // CE pin = digital out
    
  }

 
/*----------------------------------------------------------------------------*/
// initConfig() Tested OK
/*----------------------------------------------------------------------------*/
    function initConfig() {
     
    DeselectChip();                             // Default value for CSN, HIGH
    ChipDisable();                              // Default value for CE, LOW

    imp.sleep(0.05);                            // Delay for 5 ms

    configRegister(SETUP_RETR,(0x5<<ARD)|(0xF<<ARC));                // Set 1500uS timeouts
    configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT) ) ;     // Reset Status

    server.log("set channel:" + channel);
    configRegister(RF_CH, channel);
    
    server.log("set payloadsize:" + payloadSize);
    configRegister(RX_PW_P0, payloadSize);
    
    server.log("set data rate & power: 0x06");
    configRegister(RF_SETUP,0x6);                    // 1Mbps data rate, Max power

    server.log("Setting RX addr :" + pipes0);
    writeRegister(RX_ADDR_P0, pipes0, 5);
    server.log("Setting TX addr :" + pipes1);
    writeRegister(TX_ADDR, pipes1, 5);
    
    configRegister(EN_RXADDR, 1<<ERX_P0);
    
    server.log("Enable Dynamic Payload");    
    configRegister( FEATURE, 1<<EN_DPL);
    configRegister( DYNPD, 1<<DPL_P0 | 1<<DPL_P1 | 1<<DPL_P2 | 1<<DPL_P3 | 1<<DPL_P4 | 1<<DPL_P5 ); 
    
    pmode = txmode;
    powerTX();
    
    flushRX();                                      // Flush RX buffer
    flushTX();                                      // Flush TX buffer
 
    }
 
 
/*----------------------------------------------------------------------------*/
// spiSetup() Tested OK
/*----------------------------------------------------------------------------*/

    function spiSetup() {           
        hardware.configure(SPI_189);
        return hardware.spi189.configure( CLOCK_IDLE_LOW ,spiSetup_clock); // Return clock speed in Khz
    }
  
/*----------------------------------------------------------------------------*/
// getStatus() Tested OK
/*----------------------------------------------------------------------------*/  

  function getStatus() { 
      
    SelectChip();                                          
    hardware.spi189.write(format("%c",NOP));    // Send a NOP
    local status = hardware.spi189.read(1);     // Get the status back
    DeselectChip();  
    return status[0];                              // Return status
 }                
 
/*----------------------------------------------------------------------------*/
// send() Tested NOT YET
/*----------------------------------------------------------------------------*/
    function send(value, len) {
        
        local i=0;
        pmode = txmode;
        powerTX();

        flushTX();
      
        SelectChip();
        hardware.spi189.write(format("%c",W_TX_PAYLOAD));            // Write the address
        local status = hardware.spi189.read(1);                      // Read status after write
        
        while (len--) {
            hardware.spi189.write(format("%c",value[i]) );
server.log("Sending :" + value[i] ) ;      
server.show("Sending :" + value[i]);
            i++;
        }
        DeselectChip();
        
        ChipEnable();                                               // start transmission
        imp.sleep(0.015);                                           // wait 15 us
        ChipDisable();
        
    }
 
 
/*----------------------------------------------------------------------------*/
// configRegister() Tested OK for Channel and return STATUS
/*----------------------------------------------------------------------------*/

    function configRegister(regAddr, data) { 
        
        local address = ( W_REGISTER | ( REGISTER_MASK & regAddr ) )
//server.log("configRegister address:" + address);             // address sent to SPI 
//server.log("configRegister data:" + data);                   // data sent to SPI
        SelectChip();                                           // Chip select
        hardware.spi189.write(format("%c",address));            // Write the address
        local status = hardware.spi189.read(1);                 // Read status after write
        hardware.spi189.write(format("%c",data));               // Write the data
        DeselectChip();                                         // Chip deselect 
//server.log("configRegister status :" + status[0] ) ;    
        return status[0];        
    }
    
    
/*----------------------------------------------------------------------------*/ 
// readRegister() test OK for single register
/*----------------------------------------------------------------------------*/

  function readRegister(regAddr) {    
      
    local reg = ( R_REGISTER | ( regAddr & REGISTER_MASK) );    // Mask with R_REGISTER defines
//server.log("ReadRegister :" + reg);                         // Register to read 
    SelectChip();                                               // Chip select
    hardware.spi189.write( format("%c", reg) );                 // Reg to read with mask 
    hardware.spi189.read(1);
    hardware.spi189.write("\xFF");            
    local result =  hardware.spi189.read(1);
    DeselectChip();                                             // Chip deselect
//server.log("ReadRegister status :" + result[0] ) ;
    return result[0];
}


 
/*----------------------------------------------------------------------------*/ 
// writeRegister() Tested OK with len
/*----------------------------------------------------------------------------*/

    function writeRegister(regAddr, data, len) {               
    
        local i = 0;
        local address = ( W_REGISTER | ( REGISTER_MASK & regAddr ) );                  
//server.log("writeRegister address:" + address);             // address sent to SPI 
//server.log("writeRegister data:" + data);                   // data sent to SPI
        SelectChip();                                           // Chip select
        hardware.spi189.write(format("%c",address));            // Write the address
        local status = hardware.spi189.read(1);                 // Read status after write
        
        while (len--) {
            hardware.spi189.write(format("%c",data[i]) );
//server.log("writeRegister data :" + data[i] ) ; 
        i++;
        }

        DeselectChip();                                         // Chip deselect 
//server.log("writeRegister status:" + status[0]);    
        return status[0];
  }
  
/*----------------------------------------------------------------------------*/  
// csn / ce tested OK
/*----------------------------------------------------------------------------*/

    function SelectChip()   { cs_pin.write(0); }            // csn(0)
    function DeselectChip() { cs_pin.write(1); }            // csn(1)
    
    function ChipEnable()   { ce_pin.write(1); }            // ce(1)
    function ChipDisable()  { ce_pin.write(0); }            // ce(0)
    


/*----------------------------------------------------------------------------*/
// instructionByte() NOT Tested
/*----------------------------------------------------------------------------*/

    function instructByte(instruction) {                                              // Send instruction byte 
        SelectChip();                                                                   // Chip select
        local status = hardware.spi189.writeread(format("%c",instruction));            // Send instruction
        DeselectChip();                                                                 // Chip deselect
        return status[0];
    }

/*----------------------------------------------------------------------------*/
// powerDN() & powerUP() TestedOK
// EN_CRC - enable CRC
// CRC0   - 0 = 8bit, 1 = 16bit
// PWR_UP - 1 - power up, 0 power down

/*----------------------------------------------------------------------------*/

    function powerRX() { configRegister( CONFIG, 1<<EN_CRC | 1<<CRC0 | 1<<PWR_UP | 0<<PRIM_RX ); }
    function powerTX() { configRegister( CONFIG, 1<<EN_CRC | 1<<CRC0 | 1<<PWR_UP | 1<<PRIM_RX ); }
  
/*----------------------------------------------------------------------------*/
// flushRX() Tested OK
/*----------------------------------------------------------------------------*/
    function flushRX() { 
              
        SelectChip();
        local status = instructByte(FLUSH_RX);                                   // Flush RX buffer
        DeselectChip();
        return status;
    }             
  
/*----------------------------------------------------------------------------*/
// flushTX Tested OK
/*----------------------------------------------------------------------------*/  
    function flushTX() { 
      
        SelectChip();      
        local status = instructByte(FLUSH_TX);                                    // Flush RX buffer
        DeselectChip();        
        return status;
    }            
    
} // End of RF24 Class

/*----------------------------------------------------------------------------*/
// My local functions
/*----------------------------------------------------------------------------*/

    function watchdog() {

    // Do something here
    radio.send(buff,5);
    imp.wakeup(15, watchdog );                                             // Wakeup in 15 secs
    server.log("Watchdog running...");
    }
    
    function showChannel()
    {
        local channel = radio.readRegister(RF_CH);
        server.show("RF24 channel:" + channel );
    }

// Configure Imp 
server.log(" *** nRF24L01 for Imp *** ");
imp.configure("Imp RF24", [], []);

// Create the Radio Object 
radio <- RF24(100);
server.log("RF24 Initialization Success " + radio + "");

server.log("SPI Speed :" + radio.spiSetup() );   // Configure Imp SPI_189 communication, return clock in Khz

radio.initConfig();                             // Confgigure the radio with channel, speed, CRC, etc

showChannel();                                  // Show channel on planner

imp.sleep(2);

watchdog();                                     // My main loop, not sure this is the right way to implement this


// END
