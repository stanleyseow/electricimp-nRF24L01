/* 
 
 README file
 
 Electric Imp driver/library for the nRF24L01+                        
 Author: Stanley Seow                                         
 Email: stanleyseow@gmail.com
 Last update : 27 Dec 2013
 
 Repo : https://github.com/stanleyseow/electricimp-nRF24L01
 
 Works with Arduino/Raspberry Pi nRF24L01+ libs/repo at :-
 https://github.com/stanleyseow/RF24
 
 Desc : This driver uses mostly mirf libraries functions as
 it is much simpler to implement but will maintain compatibility
 with RF24 library with dynamic payload enabled and other add-on
 features.
 
 Some of the initial structure was copied from 
 https://github.com/sbolel/nrf24_imp but 
 I re-wrote most of the functions based on
 mirf libraries as sbolel code was not working
 as he just change the RF24 codes to imp without testing it
 
 I will put up as much debugging output as possible
 but these output display can be turn off during
 production
 
 Stanley
 
 Revisions by date :-
 
 20 Apr 2013 - Fixed SPI return address 
 29 Apr 2013 - radio.sent() working with RF24 ( nRF24_Arduino_as_hub ) in examples folder
 08 May 2013 - radio.getData working with RF24 ( nRF24_sendto_hub ) in examples folder
 19 June 2013 - Adding coding for agents in beta
 
 Todo :-

 -  Make this IMP/nRF24L01 code act as a wireless bridge ( wifi/nRF24L01 ) 
    from the cloud to atmel MCU at the end nodes
 -  Comms to agents (beta)
 
*/

// DOCUMENTATION: PINOUT FOR IMP to nRF24L01
// -----------------------------------------
//  nRF24  |  Imp  | Function
//  ---------------------------------------
//  CE    ->   2   |  Control RX/TX
//  CSN   ->   5   |  Chip select not
//  SCK   ->   1   |  SPI Clock
//  MOSI  ->   8   |  Master-out-slave-in
//  MISO  ->   9   |  Master-in-slave-out


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
const R_RX_PL_WID   = 0x60;
const R_RX_PAYLOAD  = 0x61;
const W_TX_PAYLOAD  = 0xA0;
const FLUSH_TX      = 0xE1;
const FLUSH_RX      = 0xE2;
const REUSE_TX_PL   = 0xE3;
const NOP           = 0xFF;

// Pipe Addresses for RX & TX 
// This is hub config, for node config is reverse
const pipesRX = "\xE7\xDE\xDE\xDE\xDE"; // RX_ADDR from Pipe0
const pipesTX = "\xE9\xDE\xDE\xDE\xDE"; // TX_ADDR to Pipe1

local statusCounter = 0;
buffers <- [48,49,50,51,52];

//********************************************************************************
//    RF24 CLASS (nRF24L01 Transciever
//********************************************************************************

class RF24 {
  /* Declare null variables */

  spiSetup_flags = null;
  spiSetup_clock = null;
  ce = null;
  cs = null;
  myspi = null;

  rf_setup = null;
  channel = null;
  pmode = null;
  ptx = null;
  payloadSize = null;
  localAddress = null;
  remoteAddress = null;

//----------------------------------------------------------------------------

  constructor(clock)                    // RF24 Class - Constructor 
  {
    /* Initialize Instance Variables */

    rf_setup       = 0x0f;               // Rate = 0x26 = 250Kbps, 0x0f = 2Mbps
    channel        = 0x55;              // CH  = 0x4c / 76
    payloadSize    = 0x20;              // Max is 32 bytes / 0x20
    localAddress   = 0;                 // LA  = 0b0000 (Unimplemented)
    remoteAddress  = 0;                 // RA  = 0b0000 (Unimplemented)
    spiSetup_flags = 0;                 // SPI Flags = 0b0000 (Master)
    spiSetup_clock = clock;             // SPI Clock Speed = input
    ptx            = 1                  // Txmode

    // Use the pin directly instead of a lookup
    
    ce = hardware.pin2;                // CE, active when HIGH
    cs = hardware.pin5;                // CSN, active when LOW
    myspi = hardware.spi189;           // Use SPI pin 1 - SCK, 8 - MOSI, 9 - MISO
    
    ce.configure(DIGITAL_OUT);          // CE Output pin 
    cs.configure(DIGITAL_OUT);          // CS Output pin
    
  }

 
/*----------------------------------------------------------------------------*/
// initConfig() Tested OK
// These are the initial config with all the debug output for checking
// Can be disabled during production 
/*----------------------------------------------------------------------------*/
    function initConfig() {
     
    DeselectChip();                             // Default value for CSN, HIGH
    ChipEnable();                              // Default value for CE, HIGH = RX mode
    
    powerDN();
    imp.sleep(0.05);                            // Delay for 5 ms
    powerUP();

//server.log("Set 1500us timeout");
    configRegister(SETUP_RETR,(0x5<<ARD)|(0xF<<ARC));                // Set 1500uS timeout
//server.log("Reset Status");
    configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT) ) ;     // Reset Status
//server.log("Setting RF_CH");
    configRegister(RF_CH, channel);                     // Setting channel
//server.log("Setting RF_SETUP");    
    configRegister(RF_SETUP,rf_setup);                       // 1Mbps data rate, Max power
//server.log("Setting ERX_P0");
    configRegister(EN_RXADDR, 1<<ERX_P0);               // Enable Pipe0 RX
//server.log("Setting Dynamic Payload");    
    configRegister(FEATURE, 1<<EN_DPL);
//server.log("Enable All Pipes Dynamic Payload");     
    configRegister( DYNPD, (1<<DPL_P0 | 1<<DPL_P1 | 1<<DPL_P2 | 1<<DPL_P3 | 1<<DPL_P4 | 1<<DPL_P5) ); 

//server.log("Setting TX_ADDR :" + format("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",pipesTX[0],pipesTX[1],pipesTX[2],pipesTX[3],pipesTX[4]) );    
     writeRegister(TX_ADDR, pipesTX, 5);
 
//server.log("Setting RX_ADDR_P0 :" + format("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",pipesRX[0],pipesRX[1],pipesRX[2],pipesRX[3],pipesRX[4]));    
     writeRegister(RX_ADDR_P0, pipesRX, 5);
   
    configRegister(RX_PW_P0, payloadSize);
	configRegister(RX_PW_P1, payloadSize);

// Verify all the register are set correctly
// 0x%2X is to print in hex
local rx = radio.readAddrRegister(RX_ADDR_P0,5);
local tx = radio.readAddrRegister(TX_ADDR,5);

//server.log("TX Addr     :" + format("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",tx[0],tx[1],tx[2],tx[3],tx[4]) );
//server.log("RX Addr-P0  :" + format("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",rx[0],rx[1],rx[2],rx[3],rx[4]) );
server.log("RF_CH       :" + format("0x%02X",readRegister(RF_CH) ) );
server.log("RF_SETUP    :" + format("0x%02X",readRegister(RF_SETUP) ) );
//server.log("EN_CRC      :" + (readRegister(CONFIG) & 1<<EN_CRC ) );
//server.log("CRC0        :" + (readRegister(CONFIG) & 1<<CRC0) );
//server.log("ENRX_P0     :" + (readRegister(EN_RXADDR) & 1<<ERX_P0) );
//server.log("EN_DPL      :" + (readRegister(FEATURE) & 1<<EN_DPL) );
//server.log("DYNPD       :" + (readRegister(DYNPD) & (1<<DPL_P0 | 1<<DPL_P1 | 1<<DPL_P2 | 1<<DPL_P3 | 1<<DPL_P4 | 1<<DPL_P5) ) ); 
 
    // Start receiver in RX mode
    powerRX();
    flushRX();
    
    imp.sleep(1);
    }
/*----------------------------------------------------------------------------*/
// spiSetup() Tested OK
// One liner to setup SPI and return the clock speed in Khz
/*----------------------------------------------------------------------------*/

    function spiSetup() {           
        hardware.configure(SPI_189);
        return myspi.configure( MSB_FIRST | CLOCK_IDLE_LOW ,spiSetup_clock); // Return clock speed in Khz
    }
  
/*----------------------------------------------------------------------------*/
// getStatus() Tested OK
// Get current status of the radio, see STATUS register for details
/*----------------------------------------------------------------------------*/  

  function getStatus() { 
      
    local reg = ( R_REGISTER | ( STATUS & REGISTER_MASK) );    // Mask with R_REGISTER defines
//server.log("STATUS read   :" + format("0x%02X",reg) );                         // Register to read 
    SelectChip();                                               // Chip select
    myspi.writeread( format("%c", reg) );                 // Reg to read with mask 
    local result = myspi.writeread("\xFF");            
    DeselectChip();                                             // Chip deselect
server.log("STATUS result :" + format("0x%02X",result[0]) ) ;
    if (result[0] == 0) { statusCounter++;}             // Check for zero return
    return result[0];  

 }                

/*----------------------------------------------------------------------------*/
// rxFifoEmpty() Tested OK
// Check if the RX FIFO is empty, return TRUE
/*----------------------------------------------------------------------------*/
    function rxFifoEmpty() {
        local fifoStatus = readRegister(FIFO_STATUS);
        return (fifoStatus & (1<<RX_EMPTY));            // Return true if RX FIFO is empty
    }
 
/*----------------------------------------------------------------------------*/
// dataReady() Tested OK
// Return TRUE if data is ready to be received, RX_DR and RX FIFO not empty
/*----------------------------------------------------------------------------*/
    function dataReady() {
       local status = getStatus(); 
       local result = 0;
       
       if ( status & (1<<RX_DR) ) result = 1;
       
       result = !rxFifoEmpty();                                     // RX Fifo NOT Empty
       
       if ( status & (1<<TX_DS)) configRegister(STATUS,(1<<TX_DS) ); // Handle ack payload receipt
       
       return result;
    }

/*----------------------------------------------------------------------------*/
// isSending() Tested OK
// Return TRUE while sending to radio
/*----------------------------------------------------------------------------*/
    function isSending() {
        if (ptx) {
            local status = getStatus(); 
                if( (status & (1<<TX_DS | 1<<MAX_RT) ) ){
		            powerRX();
		            return 0;
	            }   
        return 1;    
        }
    return 0;    
    }

/*----------------------------------------------------------------------------*/
// getData() Tested OK
// Get payload from the radio
/*----------------------------------------------------------------------------*/
    function getData(len) {

        SelectChip();
      
        myspi.write( format("%c",R_RX_PAYLOAD ) ); 
        local data = myspi.readstring(len);
        DeselectChip();
//server.log("getData data :" + data ) ; 
        configRegister(STATUS, (1<<RX_DR));     // Reset STATUS register
        
        return data;
    }
 
/*----------------------------------------------------------------------------*/
// getDynamicPayload() Tested OK
// Added this function to support RF24 dynamic payloads
/*----------------------------------------------------------------------------*/
    function getDynamicPayload() {
        SelectChip();        
        myspi.writeread( format("%c",R_RX_PL_WID) );                
        local size = myspi.writeread("\xFF");       
        DeselectChip();  
        return size[0];                                     // Return first index of array
    }
 
/*----------------------------------------------------------------------------*/
// send() Tested OK
/*----------------------------------------------------------------------------*/
    function send(value, len) {
        
        local i=0;
        
        while (ptx) {
        local status = getStatus();
	        if( (status & ((1 << TX_DS)|(1 << MAX_RT)) ) ){
		        ptx = 0;
		        break;
	        }
        }  // Wait until last packet is sent
        
        ChipDisable();                                          
        powerTX();                              // Change to TX mode
        
        configRegister(RX_PW_P0, len);     
        configRegister(RX_PW_P1, len);  
//server.log("Payload size :" + format("0x%02X",readRegister(RX_PW_P0) ) );
        flushTX();
      
        SelectChip();
        local status = myspi.writeread(format("%c",W_TX_PAYLOAD));               // Write the address
        
        while (len--) {
            myspi.write(format("%c",value[i]) );
//server.log("Sending :" + value[i] ) ;      
server.log("Sending :" + value[i]);
            i++;
        }
        DeselectChip();
        
        ChipEnable();           // Toggle CE high for at least 10 us
        imp.sleep(0.001);
        ChipDisable();
        
        ChipEnable();           // Start Tranmission    
            
    return status[0];
    }
 
/*----------------------------------------------------------------------------*/
// configRegister() Tested OK
/*----------------------------------------------------------------------------*/

    function configRegister(regAddr, data) { 
        
        local address = ( W_REGISTER | ( REGISTER_MASK & regAddr ) )
        
//server.log("configRegister address:" + format("0x%02X",address) );             // address sent to SPI 
//server.log("configRegister data:" + format("0x%02X",data) );                   // data sent to SPI

        SelectChip();                                           // Chip select
        myspi.write(format("%c",address));            // Write the address
        local status = myspi.read(1);                 // Read status after write
        myspi.write(format("%c",data));               // Write the data
        DeselectChip();                                         // Chip deselect 
//server.log("Current CONFIG reg:" + format("0x%02X",readRegister(CONFIG )) );        
        return status[0];        
    }
    
/*----------------------------------------------------------------------------*/ 
// readRegister() test OK 
/*----------------------------------------------------------------------------*/

  function readRegister(regAddr) {    
      
    local reg = ( R_REGISTER | ( regAddr & REGISTER_MASK) );    // Mask with R_REGISTER defines
//server.log("ReadRegister reg    :" + format("0x%02X",reg) );                         // Register to read 
    SelectChip();                                               // Chip select
    myspi.writeread( format("%c", reg) );                 // Reg to read with mask 
    local result = myspi.writeread("\xFF");            
    DeselectChip();                                             // Chip deselect
//server.log("ReadRegister result :" + format("0x%02X",result[0]) ) ;
    return result[0];
}

/*----------------------------------------------------------------------------*/ 
// readAddrRegister() tested OK
/*----------------------------------------------------------------------------*/

  function readAddrRegister(regAddr,len) {    

    local nodeAddr;
    local reg = ( R_REGISTER | ( regAddr & REGISTER_MASK) );        // Mask with R_REGISTER defines
//server.log("readAddrRegister :" + reg);                               // Register to read 
    SelectChip();                                                   // Chip select
    myspi.write( format("%c", reg) );                     // Reg to read with mask 

    nodeAddr = myspi.readstring(len);                  // Send 5 dummy bytes & get nodeAddr
    DeselectChip();                                             // Chip deselect
    return nodeAddr;
}

/*----------------------------------------------------------------------------*/ 
// writeRegister() Tested OK 
/*----------------------------------------------------------------------------*/

    function writeRegister(regAddr, data, len) {               
    
        local i = 0;
        local address = ( W_REGISTER | ( REGISTER_MASK & regAddr ) );                  
//server.log("writeRegister address:" + address);             // address sent to SPI 
//server.log("writeRegister data:" + data);                   // data sent to SPI
        SelectChip();                                           // Chip select
        myspi.write(format("%c",address));            // Write the address
        local status = myspi.read(1);                 // Read status after write

        while (len--) {
            myspi.write(format("%c",data[i]) );
//server.log("writeRegister data :" + format("0x%02X",data[i] ) ) ; 
            i++;
        }

        DeselectChip();                                         // Chip deselect 
//server.log("writeRegister status:" + status[0]);    
        return status[0];
  }
  
/*----------------------------------------------------------------------------*/  
// csn / ce tested OK
/*----------------------------------------------------------------------------*/

    function SelectChip()   { cs.write(0); }            // csn(0)
    function DeselectChip() { cs.write(1); }            // csn(1)
    
    function ChipEnable()   { ce.write(1); }            // ce(1)
    function ChipDisable()  { ce.write(0); }            // ce(0)
    


/*----------------------------------------------------------------------------*/
// instructionByte() Tested OK
/*----------------------------------------------------------------------------*/

    function instructByte(instruction) {                                                 // Send instruction byte 
        SelectChip();                                                                    // Chip select
        local status = myspi.writeread(format("%c",instruction));                        // Send instruction
        DeselectChip();                                                                  // Chip deselect
        return status[0];
    }

/*----------------------------------------------------------------------------*/
// powerRX() & powerTX() Tested OK
// 1<<PRIM_RX - PRX
// 0<<PRIM_RX - PTX
// 1<<EN_CRC - enable CRC
// 1<<CRC0   - 1 = 16bit
// PWR_UP - 1 - power up, 0 power down

/*----------------------------------------------------------------------------*/

    function powerRX() { 
        ptx = 0;
        ChipDisable();
        configRegister( CONFIG, (1<<EN_CRC | 1<<CRC0 | 1<<PWR_UP | 1<<PRIM_RX) ); 
        ChipEnable();
        configRegister( STATUS, (1<< TX_DS) | (1<< MAX_RT)); 
    }
    
    function powerTX() { 
        ptx = 1;
        configRegister( CONFIG, (1<<EN_CRC | 1<<CRC0 | 1<<PWR_UP | 0<<PRIM_RX) ); 
    }
    
    function powerDN() { configRegister( CONFIG, (1<<EN_CRC | 1<<CRC0 | 0<<PWR_UP) ); }
    function powerUP() { configRegister( CONFIG, (1<<EN_CRC | 1<<CRC0 | 1<<PWR_UP) ); }    
    
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
    
    local i;
    local payloadSize = 32;
    local data;
//server.log("Setting TX_ADDR :" + format("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",pipesTX[0],pipesTX[1],pipesTX[2],pipesTX[3],pipesTX[4]) );    
//     radio.writeRegister(TX_ADDR, pipesTX, 5);
     
//server.log("Setting RX_ADDR_P0 :" + format("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",pipesTX[0],pipesTX[1],pipesTX[2],pipesTX[3],pipesTX[4]) );               
// RX_ADDR_P0 must be set same as TX_ADDR for Auto-ACK to work
//      radio.writeRegister(RX_ADDR_P0, pipesTX, 5);

// Uncomment to disable sent
server.log("Sending...");    
    radio.send(buffers,5 ); 
    
//server.log("Setting RX_ADDR_P0 :" + format("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",pipesRX[0],pipesRX[1],pipesRX[2],pipesRX[3],pipesRX[4]));    
//     radio.writeRegister(RX_ADDR_P0, pipesRX, 5); // Always RX on Pipe0
     
     radio.powerRX();
     
    if ( !radio.isSending() && radio.dataReady() ) {
    
        payloadSize = radio.getDynamicPayload();
        if ( payloadSize > 0 && payloadSize < 33) {
        data = radio.getData(payloadSize);
server.log("Length                 :" + payloadSize)
server.log("getData                :" + data)
        statusCounter=0;        // Reset this counter, only reset nRF when 5 back to back zero encountered
}
        
    }     
    imp.wakeup(5, watchdog );                                     
    server.log("Watchdog running..."); 
    if ( statusCounter == 5 ) {     // Reset radio after five back to back zero STATUS received
        radio.powerDN; 
        imp.sleep(1); 
server.log("****** Resetting radio ********")        
        radio.powerUP; 
        radio.initConfig();
        statusCounter=0;
        
    }
    
    }
    
    function watchdog2() {
        agent.on("agentBuffers",function(value) {
        server.log("Got a buffer from agent/http");
        if ( value.len() < 33 ) {
        radio.send(value,value.len());          // Send the payload to the radio
        }
    } );
        radio.powerRX();
        imp.wakeup(1, watchdog2 );    
    }
    
// Configure Imp 
server.log(" *** nRF24L01 for Imp *** ");
//imp.configure("Imp RF24", [], []);

// Create the Radio Object 
radio <- RF24(100);
server.log("RF24 Initialization Success " + radio + "");
server.log("SPI Speed :" + radio.spiSetup() );   // Configure Imp SPI_189 communication, return clock in Khz
radio.initConfig();                             // Confgigure the radio with channel, speed, CRC, etc
imp.sleep(1);
watchdog2();                                     // My main loop, not sure this is the right way to implement this
// END
