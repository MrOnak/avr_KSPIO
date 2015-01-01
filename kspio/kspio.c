/**
 * Port of the KSPIO code (c) by 'zitronen' to AVR C
 * http://forum.kerbalspaceprogram.com/threads/66393
 *
 * Zitronen's code is released under Creative Commons Attribution (BY).
 * The same is true for my port.
 *
 * Zitronen's code was based around the Arduino library and 
 * mostly intended as a demonstration, to get people started.
 *
 * I'm porting this to AVR's variant of C to open it up to Atmel 
 * microprocessors without the need for the Arduino GUI. 
 *
 * This port is based off of the "KSPIODemo8" and expects to 
 * be communicating with v0.15.3 of the KSPIO plugin.
 *
 * @author Dominique Stender <dst@st-webdevelopment.com>
 */
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>

#include "../uart/uart.h"
#include "millis.h"

// macros
#ifndef details
#define details(name) (uint8_t*)&name,sizeof(name)
#endif
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

#ifndef KSPIO_IDLETIMER
//if no message received from KSP for more than 2s, go idle
#define KSPIO_IDLETIMER 2000
#endif
#ifndef KSPIO_CONTROLREFRESH
#define KSPIO_CONTROLREFRESH 25
#endif

// pins
#define KSPIO_CTRL_PORT DDRB
#define KSPIO_DATA_PORT PORTB
#define KSPIO_DATA_PORT_IN PINB
#define KSPIO_SASPIN PB0
#define KSPIO_RCSPIN PB1
#define KSPIO_CG1PIN PB2
#define KSPIO_THROTTLEPIN PC1
#define KSPIO_THROTTLEDB 4 //Throttle axis deadband

#define KSPIO_SAS 7
#define KSPIO_RCS 6
#define KSPIO_LIGHTS 5
#define KSPIO_GEAR 4
#define KSPIO_BRAKES 3
#define KSPIO_PRECISION 2
#define KSPIO_ABORT 1
#define KSPIO_STAGE 0

//pins for LEDs
#define KSPIO_LED_CTRL_PORT DDRD
#define KSPIO_LED_DATA_PORT PORTD
#define KSPIO_GLED PD5
#define KSPIO_YLED PD6
#define KSPIO_RLED PD7

//warnings
#define KSPIO_GWARN 9                  //9G Warning
#define KSPIO_GCAUTION 5               //5G Caution
#define KSPIO_FUELCAUTION 10.0         //10% Fuel Caution
#define KSPIO_FUELWARN 5.0             //5% Fuel warning


// structs 
typedef struct {
  uint8_t id;             //1
  float AP;               //2
  float PE;               //3
  float SemiMajorAxis;    //4
  float SemiMinorAxis;    //5
  float VVI;              //6
  float e;                //7
  float inc;              //8
  float G;                //9
  int32_t TAp;               //10
  int32_t TPe;               //11
  float TrueAnomaly;      //12
  float Density;          //13
  int32_t period;            //14
  float RAlt;             //15
  float Alt;              //16
  float Vsurf;            //17
  float Lat;              //18
  float Lon;              //19
  float LiquidFuelTot;    //20
  float LiquidFuel;       //21
  float OxidizerTot;      //22
  float Oxidizer;         //23
  float EChargeTot;       //24
  float ECharge;          //25
  float MonoPropTot;      //26
  float MonoProp;         //27
  float IntakeAirTot;     //28
  float IntakeAir;        //29
  float SolidFuelTot;     //30
  float SolidFuel;        //31
  float XenonGasTot;      //32
  float XenonGas;         //33
  float LiquidFuelTotS;   //34
  float LiquidFuelS;      //35
  float OxidizerTotS;     //36
  float OxidizerS;        //37
  uint32_t MissionTime;   //38
  float deltaTime;        //39
  float VOrbit;           //40
  uint32_t MNTime;        //41
  float MNDeltaV;         //42
  float Pitch;            //43  
  float Roll;             //44  
  float Heading;          //45  
} vesselData_t;
 
typedef struct {
  uint8_t id;
  uint8_t M1;
  uint8_t M2;
  uint8_t M3;
} handShakePacket_t;

typedef struct {
  uint8_t id;
  uint8_t MainControls;               //SAS RCS Lights Gear Brakes Precision Abort Stage 
  uint8_t Mode;                       //0 = stage, 1 = docking, 2 = map
  uint16_t ControlGroup;          //control groups 1-10 in 2 bytes
  uint8_t AdditionalControlByte1;     //other stuff
  uint8_t AdditionalControlByte2;
  int16_t Pitch;                          //-1000 -> 1000
  int16_t Roll;                           //-1000 -> 1000
  int16_t Yaw;                            //-1000 -> 1000
  int16_t TX;                             //-1000 -> 1000
  int16_t TY;                             //-1000 -> 1000
  int16_t TZ;                             //-1000 -> 1000
  int16_t Throttle;                       //    0 -> 1000
} controlPacket_t;

// variables
uint32_t kspio_deadtime, kspio_deadtimeOld, kspio_controlTime, kspio_controlTimeOld;
uint32_t kspio_now;
uint8_t kspio_connected = 0;
uint8_t kspio_caution = 0, kspio_warning = 0, kspio_id;

// UART variables
uint8_t kspio_rxLen;
uint8_t * kspio_address;
uint8_t kspio_buffer[256]; //address for temporary storage and parsing buffer
uint8_t kspio_structSize;
uint8_t kspio_rxArrayInx;  //index for RX parsing buffer
uint8_t kspio_calcCS;	   //calculated Chacksum


vesselData_t kspio_vData;
handShakePacket_t kspio_hPacket;
controlPacket_t kspio_cPacket;

// functions 

//---- utilities --------------------------------------------------------------
void kspio_indicators() {
  kspio_caution = 0;
  kspio_warning = 0;

  kspio_caution += kspio_vData.G > KSPIO_GCAUTION;
  kspio_warning += kspio_vData.G > KSPIO_GWARN;
  kspio_caution += kspio_vData.LiquidFuelS / kspio_vData.LiquidFuelTotS*100 < KSPIO_FUELCAUTION;
  kspio_warning += kspio_vData.LiquidFuelS / kspio_vData.LiquidFuelTotS*100 < KSPIO_FUELWARN;

  if (kspio_caution != 0) {
    sbi(KSPIO_LED_DATA_PORT, KSPIO_YLED);
  } else {
    cbi(KSPIO_LED_DATA_PORT, KSPIO_YLED);
  }

  if (kspio_warning != 0) {
    sbi(KSPIO_LED_DATA_PORT, KSPIO_RLED);
  } else {
    cbi(KSPIO_LED_DATA_PORT, KSPIO_RLED);
  }
}

void kspio_LEDSAllOff() {
  cbi(KSPIO_LED_DATA_PORT, KSPIO_GLED);
  cbi(KSPIO_LED_DATA_PORT, KSPIO_YLED);
  cbi(KSPIO_LED_DATA_PORT, KSPIO_RLED);
}


void kspio_initLEDS() {
  KSPIO_LED_CTRL_PORT |= (1 << KSPIO_GLED) | (1 << KSPIO_YLED) | (1 << KSPIO_RLED);
  sbi(KSPIO_LED_DATA_PORT, KSPIO_GLED);
  sbi(KSPIO_LED_DATA_PORT, KSPIO_YLED);
  sbi(KSPIO_LED_DATA_PORT, KSPIO_RLED);
}

/**
 *
 */
void kspio_initTXPackets() {
  kspio_hPacket.id = 0;  
  kspio_cPacket.id = 101;
}

//---- UART stuff -------------------------------------------------------------
/**
 * contains stuff borrowed from EasyTransfer lib
 */
uint8_t kspio_boardReceiveData() {
  uint8_t i;
  uint16_t c;
  unsigned char chr;
  
  if (kspio_rxLen == 0 && uart_available() > 3) {
	c = uart_getc();
	chr = (unsigned char) c;
	
	while (chr != 0xBE) {	
	  if (uart_available() == 0) {
		return 0;
      }
	  
      c   = uart_getc();         // read character from UART
      chr = (unsigned char) c;   // low byte is the actual character, store that in chr
	}

    c   = uart_getc();         
    chr = (unsigned char) c;

	if (chr == 0xEF) {	
      c = uart_getc(); kspio_rxLen = (unsigned char) c;
      c = uart_getc(); kspio_id    = (unsigned char) c;
      kspio_rxArrayInx = 1;

      switch(kspio_id) {
        case 0:	
          kspio_structSize = sizeof(kspio_hPacket);   
          kspio_address    = (uint8_t*)&kspio_hPacket;     
          break;
        case 1:	
          kspio_structSize = sizeof(kspio_vData);   
          kspio_address    = (uint8_t*)&kspio_vData;     
          break;
      }	  
	}

    //make sure the binary structs on both Arduinos are the same size.
    if (kspio_rxLen != kspio_structSize) {
      kspio_rxLen = 0;
      return 0;
    }  	
  }
  
  
  if (kspio_rxLen != 0) {
	while (uart_available() && kspio_rxArrayInx <= kspio_rxLen) {
      c = uart_getc();	
	  kspio_buffer[kspio_rxArrayInx++] = (unsigned char) c;
	}
    kspio_buffer[0] = kspio_id;

    if (kspio_rxLen == (kspio_rxArrayInx-1)) {
      //seem to have got whole message
      //last uint8_t is CS
      kspio_calcCS = kspio_rxLen;
      for (i = 0; i<kspio_rxLen; i++) {
        kspio_calcCS^=kspio_buffer[i];
      } 

      if (kspio_calcCS == kspio_buffer[kspio_rxArrayInx-1]) {//CS good
        memcpy(kspio_address, kspio_buffer, kspio_structSize);
        kspio_rxLen = 0;
        kspio_rxArrayInx = 1;
        return 1;
      } else {
        //failed checksum, need to clear this out anyway
        kspio_rxLen = 0;
        kspio_rxArrayInx = 1;
        return 0;
      }
    }
  }

  return 0;
}


/**
 *
 */
void kspio_boardSendData(uint8_t * address, uint8_t len) {
  uint8_t CS = len;
  uint8_t i;
  uart_putc(0xBE);
  uart_putc(0xEF);
  uart_putc(len);
  
  for(i = 0; i < len; i++){
    CS^=*(address+i);
	uart_putc(*(address+i));
  }

  uart_putc(CS);
}

//---- handshake --------------------------------------------------------------
/**
 *
 */
void kspio_handshake() {
  sbi(KSPIO_LED_DATA_PORT, KSPIO_GLED);

  kspio_hPacket.id = 0;
  kspio_hPacket.M1 = 3;
  kspio_hPacket.M2 = 1;
  kspio_hPacket.M3 = 4;

  kspio_boardSendData(details(kspio_hPacket));
}

//---- input ------------------------------------------------------------------
/**
 * Will query the serial port and update the vessel information
 * if new data was fetched.
 * 
 * Will return the following:
 * -1 - if no new data was present
 * 0 - if a handshake took place (switching to a new vessel etc)
 * 1 - if new data was fetched
 */
int8_t kspio_input() {
  kspio_now = millis();
  int8_t returnValue = -1;

  if (kspio_boardReceiveData()) {
    kspio_deadtimeOld = kspio_now;
    returnValue = kspio_id;
    
    switch (kspio_id) {
      case 0: //Handshake packet
        kspio_handshake();
        break;
      case 1:
        kspio_indicators();
        break;
    }

    //We got some data, turn the green led on
	sbi(KSPIO_LED_DATA_PORT, KSPIO_GLED);
    kspio_connected = 1;
  } else { //if no message received for a while, go idle
    kspio_deadtime = kspio_now - kspio_deadtimeOld; 
    if (kspio_deadtime > KSPIO_IDLETIMER) {
      kspio_deadtimeOld = kspio_now;
      kspio_connected = 0;
      kspio_LEDSAllOff();
    }    
  }
  
  return returnValue;
}

//---- output -----------------------------------------------------------------
/**
 * configures the pins for SAS, RCS and CG1 as inputs with pullup
 */
void kspio_controlsInit() {
  KSPIO_CTRL_PORT &= ~(1 << KSPIO_SASPIN);
  KSPIO_DATA_PORT_IN |= (1 << KSPIO_SASPIN);
  KSPIO_CTRL_PORT &= ~(1 << KSPIO_RCSPIN);
  KSPIO_DATA_PORT_IN |= (1 << KSPIO_RCSPIN);
  KSPIO_CTRL_PORT &= ~(1 << KSPIO_CG1PIN);
  KSPIO_DATA_PORT_IN |= (1 << KSPIO_CG1PIN);
}

/**
 * sets the Nth bit of the main controls to HI when s == 1, LO if s == 0
 */
void kspio_mainControls(uint8_t n, uint8_t s) {
  if (s) {
    kspio_cPacket.MainControls |= (1 << n);       // forces nth bit of x to be 1.  all other bits left alone.
  } else {
    kspio_cPacket.MainControls &= ~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
  }
}

/**
 * sets the Nth bit of the main controls to HI when s == 1, LO if s == 0
 */
void kspio_controlGroups(uint8_t n, uint8_t s) {
  if (s) {
    kspio_cPacket.ControlGroup |= (1 << n);       // forces nth bit of x to be 1.  all other bits left alone.
  } else {
    kspio_cPacket.ControlGroup &= ~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
  }
}

/**
 *
 */
void kspio_controls() {
  if (kspio_connected) {
	// this is how you do main controls
    if (KSPIO_DATA_PORT_IN & (1 << KSPIO_SASPIN)) {
      kspio_mainControls(KSPIO_SAS, 1);
    } else {
      kspio_mainControls(KSPIO_SAS, 0);
	}

    if (KSPIO_DATA_PORT_IN & (1 << KSPIO_RCSPIN)) {
      kspio_mainControls(KSPIO_RCS, 1);
    } else {
      kspio_mainControls(KSPIO_RCS, 0);
	}

    // this is how you do control groups
	if (KSPIO_DATA_PORT_IN & (1 << KSPIO_CG1PIN)) { 
      kspio_controlGroups(1, 1);
    } else {
      kspio_controlGroups(1, 0);      
	}
	
    //This is an example of reading analog inputs to an axis, with deadband and limits
	// @todo since I'm not using this it is NOT ported to AVR
//    kspio_cPacket.Throttle = constrain(map(analogRead(THROTTLEPIN),THROTTLEDB,1024-THROTTLEDB,0,1000),0, 1000);

    kspio_boardSendData(details(kspio_cPacket));
  }
}

/**
 *
 */
void kspio_output() {
  kspio_now = millis();
  kspio_controlTime = kspio_now - kspio_controlTimeOld; 
  
  if (kspio_controlTime > KSPIO_CONTROLREFRESH){
    kspio_controlTimeOld = kspio_now;
    kspio_controls();
  }   
}

/**
 *
 */
void kspio_init() {
  millisInit();		// configures TIMER0
  
  kspio_initLEDS();
  kspio_initTXPackets();
  kspio_controlsInit();

  sbi(KSPIO_LED_DATA_PORT, KSPIO_GLED);
  sbi(KSPIO_LED_DATA_PORT, KSPIO_YLED);
  sbi(KSPIO_LED_DATA_PORT, KSPIO_RLED);
  _delay_ms(1000);
  
  kspio_LEDSAllOff();  
}

