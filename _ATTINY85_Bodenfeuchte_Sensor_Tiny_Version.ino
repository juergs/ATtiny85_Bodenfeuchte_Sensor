/************************************************************************
*  433MHz - Bodenfeuchte-Sensor [FHEM]
*  Thanks to TantaJu@https://forum.fhem.de/index.php/topic,57460.0.html
*  used original Mysensor-Sketch as template
*  juergs, 16.10.2016, initial version.
*  juergs, 22.10.2016, updated.
*  juergs, 30.10.2016, updated.
*  *********************************************************************
*  ATMEL ATTINY 25/45/85 / ARDUINO
*
*                      +-\/-+
*     Reset(D 5) PB5  1|    |8  Vcc
*     Ain3 (D 3) PB3  2|    |7  PB2 (D 2) Ain1 
*     Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1 
*                GND  4|    |5  PB0 (D 0) pwm0 *INT0*
*                      +----+
*
*  Install: ATtiny-Models in Arduino IDE: http://highlowtech.org/?p=1695
* ----------------------------------------------------------------------------------------------------------------------------------------
*
*  ATTINY:
*    https://cpldcpu.wordpress.com/2014/04/25/the-nanite-85/
*    https://thewanderingengineer.com/2014/08/11/pin-change-interrupts-on-attiny85/
*    http://thegaragelab.com/a-software-uart-for-the-attiny85/
*    https://github.com/thegaragelab/tinytemplate
*    https://github.com/thegaragelab/microboot
*    http://thegaragelab.com/microboot-a-simple-bootloader/
*	 http://shelvin.de/den-mittelwert-fortlaufend-bilden-mit-einer-funktion/
*	 http://shelvin.de/arduino-in-den-sleep_mode_pwr_down-schlaf-modus-setzen/
*	 http://donalmorrissey.blogspot.de/2010/04/putting-arduino-diecimila-to-sleep-part.html
*	 https://code.google.com/archive/p/narcoleptic/
*	 http://playground.arduino.cc/Main/LibraryList#Testing
*	 http://www.engblaze.com/hush-little-microprocessor-avr-and-arduino-sleep-mode-basics/
*	 http://donalmorrissey.blogspot.de/2010/04/sleeping-arduino-part-5-wake-up-via.html
+	 http://milesburton.com/Main_Page?title=Dallas_Temperature_Control_Library
+	 http://www.pjrc.com/teensy/td_libs_OneWire.html
*	 http://en.cppreference.com/w/cpp/language/integer_literal
*
* ----------------------------------------------------------------------------------------------------------------------------------------
*   Note: Naroleptic-Lib ist verändert!
*		// -- extra mod to add support attiny85 in Narcoleptic.cpp
*		#if defined(__AVR_ATtiny85__)
*		#define WDTCSR WDTCR
*		#endif	 
* ----------------------------------------------------------------------------------------------------------------------------------------
*
*		sprintf-Beispiel:
*		-----------------
*		char line[17];
*		// Hardware ID:
*		sprintf(line, "%02X%02X%02X%02X%02X%02X%02X%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);
*
* ----------------------------------------------------------------------------------------------------------------------------------------
*
* The LaCrosse-protocol (TX2)seems to be:
*
*     Bits 0-3: header
*     Bits 4-11: device ID, changes when replacing the batteries. Unlike in the post linked above, bit 11 does not appear to be a checksum.
*     Bits 12-15: either 1111 for automatic transmission (once every 60 seconds) or 1011 for manual transmission (using the button in the battery compartment). Manual transmission does not update the weather station.
*     Bits 16-27: encode the temperature. The system of encoding decimal digits seems to be ditched in favor of a more elegant one: apply a NOT (change 1 to 0 and 0 to 1), convert to base 10, divide by 10 (into a float), subtract 50, and the result is the temperature in C.
*     Bits 28-35: encode the relative humidity. Apply a NOT, convert to base 10, and the result is the relative humidity in %.
*     Bits 36-43: appear to encode a checksum (though I plan to double-check if this is not the dew point, also reported by the weather station).

*     Example:
*     HHHH 1000 0010 1111 1101 0010 1111 1101 0011 1010 0100
*     encoding T=22.0C and RH=44%
*
*****************************************************************************************************************************************************
*/

/*
#define USE_LOCAL_ARDUINO_LIBRARIES
*/

#ifdef USE_LOCAL_ARDUINO_LIBRARIES
	#include "LaCrosse.h"
	#include "Narcoleptic.h"
	#include "OneWire.h"
#else
	#include <attiny_bodenfeuchte_sensor\LaCrosse.h>
	#include <attiny_bodenfeuchte_sensor\Narcoleptic.h>
	#include <attiny_bodenfeuchte_sensor\OneWire.h>
#endif

#include <avr/power.h>
//#include "D:\Program Files (x86)\Arduino\arduino-1.6.12\hardware\tools\avr\avr\include\avr\power.h"

#define SN "Bodenfeuchte-Sensor-433-ATtiny85-Version"
#define SV "1.3 vom 30.10.2016"

//--- conditionals
//--- zum aktivieren Kommentierung entfernen 
//#define USE_WITH_NANO            
#define USE_SEPARATE_BATTERIE_ID 
#define USE_WITH_DALLAS_SENSOR          
#define USE_WITH_LED
//#define USE_SERIAL_OUTPUT_NANO
//#define USE_SERIAL_OUTPUT_ATTINY
#define DEBUG_TEST_MODUS	0		// 0 = Off, 1 = On 

#ifdef USE_WITH_NANO 
	#define DALLAS_SENSOR_PIN         10    //   PIN5 = PB0 = D0 - Achtung: MOSI, Jumper zum Programmieren entfernen.
	#define BODENFEUCHTE_SENSOR_PIN   2     //
	#define BODENFEUCHTE_POWER_PIN    11    //   
	#define TX_433_PIN                12    //   PIN6 = PB1 = D1 - Achtung: MISO.   PIN_SEND in LaCrosse.cpp
#else
	//--- ATTiny
	#define DALLAS_SENSOR_PIN         0     //   PIN5 = PB0 = D0 
	#define BODENFEUCHTE_SENSOR_PIN   2     //	 PIN7 = PB2 = D2
	#define BODENFEUCHTE_POWER_PIN    1     //   PIN6 = PB1 = D1 
	#define TX_433_PIN                3     //   PIN2 = PB3 = D3	[### TODO: PIN_SEND in LaCrosse.cpp ebenfall setzen!!!]
	#define PIN_FREI				  4	    //   PIN3 = PB4 = D4
#endif 

#define SENSORID_BODENFEUCHTE       102     //   Temperatur + Bodenfeuchte (int)
#define SENSORID_BATTERIE           103     //   VCC + Bodenfeuchte (float)
#define SLEEP_MINUTES				  1     //  +++ powerdown mode duration +++

#define OW_ROMCODE_SIZE               8

//--- use with leds? 
#define LED_ONTIME		            100  // Number of cycles for LEDs to stay on, when only temporary 								        
#define POWERAMV					  4    //--- Output line to provide power for the AMV
#define INPUTFREQ					  2    //--- IRQ input for frequency count. Must be 2 or 3

#define CHILD_ID					  0    //--- Child-Number for Humidity
#define CHILD_TEMP                    1	   //--- Child-Number for Temperature, if attached

#define BATTPOWER                     1	   //--- ADC Input for Battery-Power, comment out if no Step-Up is used

#define MAIN_PERIOD                 162    //--- Zeitfenster um Impulse zu zählen
#define SETTLE_TIME                 1000   //--- waiting time in ms between powering up AMV and stable frequency

#ifdef USE_WITH_LED
	#define LED_MAIN			PIN_FREI   // Mains for LEDs, must be set LOW for LEDs to work
#endif 

									//--- globals
unsigned int              led_startup;
float                     controller_VCC		= 0.0;
long                      vcc_reading			= 0;
volatile unsigned int     pulsecount			= 0;	//--- Counter for pulses
unsigned int              average;						//---  floating average filter
byte                      led_temporary			= 1;    // Are LEDs jumpered, to stay on=0, otherwise 1 
unsigned int              loop_counter			= 0;	// Verzögerungs-Timeout-Berechnung
														//--- TX-outputs:
volatile float            bodenfeuchte			= 0.0;
volatile float            batteriespannung		= 0.0;
long					  randomNumber			= 0;

long test = 0 ; 

//--- serial msgbuffer 
char *  msgbuf[100];

#ifdef USE_WITH_DALLAS_SENSOR
	//--- Temperatursensor-Instanz
	OneWire  dallas(DALLAS_SENSOR_PIN);	// on arduino port pin 2 (a 4.7K resistor is necessary, between Vcc and DQ-Pin   1=GND 2=DQ 3=Vcc )
#endif

//---------------------------------------------------------------------
//---------------------------------------------------------------------
//--- Prototypes
float DoBodenFeuchteMeasurement(void);
float ReadSingleOneWireSensor(OneWire ds);
long  readVcc(void);
void  blink(byte pin, int delay_ms); 
//---------------------------------------------------------------------
//---------------------------------------------------------------------
void setup()
{
	/*setup code here*/
	pinMode(TX_433_PIN, OUTPUT);
	pinMode(DALLAS_SENSOR_PIN, OUTPUT);
	digitalWrite(DALLAS_SENSOR_PIN, HIGH);

	//--- make power-line for AMV output and low for idle 
	pinMode(BODENFEUCHTE_POWER_PIN, OUTPUT);
	digitalWrite(BODENFEUCHTE_POWER_PIN, LOW);

	//--- preset SensorId & TX instance 
	LaCrosse.bSensorId = SENSORID_BODENFEUCHTE;
	LaCrosse.setTxPinMode(OUTPUT);

#ifdef USE_WITH_LED
	pinMode(PIN_FREI, OUTPUT);
	//--- using LED on D4, if exists
	blink(PIN_FREI, LED_ONTIME);
	blink(PIN_FREI, LED_ONTIME);
	blink(PIN_FREI, LED_ONTIME);
#else
	//--- to save power
	pinMode(PIN_FREI, INPUT);
#endif 

	delay(2000); 
}
//---------------------------------------------------------------------
//	* * *   M A I N - L O O P  * * *  
//---------------------------------------------------------------------
void loop()
{
	//--- set attiny back to normal operation mode
	pinMode(DALLAS_SENSOR_PIN, OUTPUT);
	LaCrosse.setTxPinMode(OUTPUT);
	power_adc_enable(); 

	delay(500); // ms, needed for settling DS18B20 

#ifdef USE_WITH_LED
	digitalWrite(PIN_FREI, HIGH);  //--- Led On
#endif // USE_WITH

	//--- Betriebsspannung auslesen  
	vcc_reading = readVcc();

	//float controllerVCC = 1.1 * 1023 / vcc_reading; 
	float controllerVCC = vcc_reading / 1000.0;

	//--- Bodenfeuchte auslesen 
	bodenfeuchte = DoBodenFeuchteMeasurement();

#ifdef USE_WITH_DALLAS_SENSOR
	float theta = ReadSingleOneWireSensor(dallas);
#else
	float theta = 0.0; 
#endif 

	// *** Bodenfeuchte-Zaehlerstand senden 
	//     transfer measured values through LaCrosse-TX2-instance
	//*****************************************
	LaCrosse.bSensorId = SENSORID_BODENFEUCHTE;
	//*****************************************
	LaCrosse.t = theta;         //--- alias temperature;  	
	LaCrosse.sendTemperature();	
	LaCrosse.sleep(1);        /* 1 second, no power-reduction! see impact on powersave */
	LaCrosse.h = bodenfeuchte / 100;    //--- alias humidity for 100% scaling;  
	LaCrosse.sendHumidity();	
	LaCrosse.sleep(1);        /* 1 second, no power-reduction! see impact on powersave */

	// *** Batteriespannung senden 
#ifdef USE_SEPARATE_BATTERIE_ID 
	//*****************************************
	LaCrosse.bSensorId = SENSORID_BATTERIE;
	//*****************************************
#endif 
	LaCrosse.t = float(bodenfeuchte) / 1000;   // for 10er scaling
	LaCrosse.sendTemperature();	
	LaCrosse.sleep(1);					//--- 1 second, no power-reduction! see impact on powersave 	
	LaCrosse.h = controllerVCC;			//--- alias humidity;
	LaCrosse.sendHumidity();
	LaCrosse.sleep(1);					//--- 1 second, no power-reduction! 

#ifdef USE_WITH_LED
	digitalWrite(PIN_FREI, LOW);  //--- Led Off
#endif // USE_WITH


#if DEBUG_TEST_MODUS == 1
	delay(3000);     //--- as test repetition in millis					 
#else 

	//--- preserve more power during sleep phase 
	pinMode(DALLAS_SENSOR_PIN, INPUT);	
	LaCrosse.setTxPinMode(INPUT);

	//--- switch AD-converter off
	power_adc_disable(); 

	//--- fall to deep powersave-sleep, see notes in comments and 
	Narcoleptic.delay_minutes(SLEEP_MINUTES);	
#endif 
}

//---------------------------------------------------------------------
void myinthandler(void)   //--- async interrupt handler
{		
	pulsecount++;	
}
//---------------------------------------------------------------------
float DoBodenFeuchteMeasurement(void)
{
	//--- [1] startUp Multivibrator
	pinMode(BODENFEUCHTE_SENSOR_PIN, INPUT);
	pinMode(BODENFEUCHTE_POWER_PIN, OUTPUT);
	digitalWrite(BODENFEUCHTE_POWER_PIN, HIGH);   //--- power up sensor circuit  

	//--- [2] wait for settle
	delay(SETTLE_TIME);

	//--- [3] Prepare measurement
	pulsecount = 0;

	// Sensor Pulses attached to INT0 Pin 5 = D0
	attachInterrupt(0, myinthandler, FALLING); // IRQ D2  high to low

	//--- [4] wait for measurement to finalize, wait for interrupt pulse actions
	delay(MAIN_PERIOD);

	//--- [5] store actual counter value
	//---     register counts per period (frequency) and calculate IIR
	unsigned long _pulses = pulsecount;
	pulsecount = 0;

	//--- simple floating average
	average += _pulses;
	if (average != _pulses)    //--- during startup both are equal
		average >>= 1;

	//---[6] stop measuring and AMV
	digitalWrite(BODENFEUCHTE_POWER_PIN, LOW);

	//--- Sensor Pulses attached to INT0 Pin 5 = D0
	detachInterrupt(0);

	pinMode(BODENFEUCHTE_SENSOR_PIN, INPUT);

	//--- [7] as result set reading  for TX 
	return ((float)average);
}
//-------------------------------------------------------------------------
float ReadSingleOneWireSensor(OneWire ds)
{
	//--- 18B20 stuff
	byte i;
	byte type_s;
	byte data[12];
	byte addr[8];
	float celsius = 12.3;

	if (!ds.search(addr))
	{
		ds.reset_search();
		delay(250);
		return celsius;
	}

	if (OneWire::crc8(addr, 7) != addr[7])
	{
		return celsius;
	}

	//--- the first ROM byte indicates which chip
	switch (addr[0])
	{
	case 0x10:
		//Serial.println("  Chip = DS18S20");  // or old DS1820
		type_s = 1;
		break;
	case 0x28:
		// Serial.println("  Chip = DS18B20");
		type_s = 0;
		break;
	case 0x22:
		// Serial.println("  Chip = DS1822");
		type_s = 0;
		break;
	default:
		// Serial.println("Device is not a DS18x20 family device.");
		return celsius;
	}

	ds.reset();
	ds.select(addr);
	ds.write(0x44,1 );  // start conversion, use alternatively ds.write(0x44,1) with parasite power on at the end

	delay(1000);     // maybe 750ms is enough, maybe not
					 // we might do a ds.depower() here, but the reset will take care of it.

	ds.reset();		 //--- DS18B20 responds with presence pulse
	//--- match ROM 0x55, sensor sends ROM-code command ommitted here.
	ds.select(addr);
	ds.write(0xBE);         //--- read scratchpad
	for (i = 0; i < 9; i++)
	{
		//--- we need 9 bytes, 9th byte is CRC, first 8 are data
		data[i] = ds.read();
	}

	//--- Convert the data to actual temperature
	//--- because the result is a 16 bit signed integer, it should
	//--- be stored to an "int16_t" type, which is always 16 bits
	//--- even when compiled on a 32 bit processor.
	int16_t raw = (data[1] << 8) | data[0];
	if (type_s)
	{
		raw = raw << 3;     //--- 9 bit resolution default
		if (data[7] == 0x10)
		{
			//--- "count remain" gives full 12 bit resolution
			raw = (raw & 0xFFF0) + 12 - data[6];
		};
	}
	else
	{
		byte cfg = (data[4] & 0x60);
		// at lower res, the low bits are undefined, so let's zero them
		if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
		else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
		else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
											  //// default is 12 bit resolution, 750 ms conversion time
	};

	celsius = (float)raw / 16.0;		//fahrenheit = celsius * 1.8 + 32.0;

	//---- Check if any reads failed and exit early (to try again).  
	if (isnan(celsius))
	{
		//--- signalize error condition 
		celsius = -99.9;
	};
	return celsius;
}
//---------------------------------------------------------------------
//--- Helpers 
//---------------------------------------------------------------------
long readVcc()
{
	//--- read 1.1V reference against AVcc
	//--- set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
	ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	ADMUX = _BV(MUX3) | _BV(MUX2);
#else
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

	delay(2); // Wait for Vref to settle

	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA, ADSC)); // measuring

	uint8_t low = ADCL; // must read ADCL first - it then locks ADCH  
	uint8_t high = ADCH; // unlocks both

	long result = (high << 8) | low;

	/***************************************************************************************
	*  Berechnung/Skalierung mit manueller Messung der Betriebsspannung:
	*
	*        internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
	*                       = 1.1 * 5126				 / 5258 => 1.09 ==> 1.09*1023*1000 = 1097049
	****************************************************************************************/

	result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

								//result = 1097049L / result; // korrigierter Wert bei 3V3 müsste fuer jeden Controller bestimmt werden, obiger Wert scheint allgemeiner zu sein.

	return result; // Vcc in millivolts
}
//-------------------------------------------------------------------------
void blink(byte pin, int delay_ms)
{
	pinMode(pin, OUTPUT);
	digitalWrite(pin, HIGH);
	delay(delay_ms);
	digitalWrite(pin, LOW);
	delay(delay_ms);
}
//-------------------------------------------------------------------------
//---------------------------------------------------------------------
// <eof>
//---------------------------------------------------------------------

