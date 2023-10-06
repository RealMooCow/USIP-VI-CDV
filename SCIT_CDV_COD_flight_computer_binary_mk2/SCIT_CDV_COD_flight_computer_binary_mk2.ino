
// Need to update some libraries
#include <Ard2499.h>

#include <Arduino_AVRSTL.h>
#include <SD.h> // go together when instal the Adafruit_GPS.h / forget the SSD3361 in the libraries folder
#include "SPI.h"
// we won't use AMS or DHT sensors for the eclipse
//#include <AMS.h>
//#include "DHT.h"
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_BME680.h>
#include <Adafruit_LIS2MDL.h>

// use AVR 1.8.2

// pins
#define LED1 6
#define LED2 7
#define EN A2
#define CAL A3

/*
"PMTK command is MediaTek proprietary data transfer protocol for GNSS (global
navigation satellite system). It enables configuring the parameters of the
GNSS chipset, aiding assistance position information and receive notifications
from the GNSS chip."

$ = preamble
PMTK = TalkerID
three digit number after PMTK = packet type. e.g. 314, 220, 605, etc
remaining digits separated by commas are the datafield. '*' marks the end of a datafield
remaining digits are Checksum

For full info: https://www.rhydolabz.com/wiki/?p=16770
*/

/// GPS: NMEA (National Marine Electronics Association) sentences sent as commands
#define PMTK_SET_NMEA_OUTPUT_GGAONLY "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" //GPS- Global Positioning System Fix Data// 1 mean "on" and 0 mean "off"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" //GPS - Recommended Minimum Specific GPS
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28" //GPS - All data
#define PMTK_SET_NMEA_UPDATE_1SEC "$PMTK220,1000*1B" // code for GPS start from $ to * - the rest is the checksum
#define PMTK_SET_NMEA_UPDATE_2SEC "$PMTK220,2000*1B"
#define PMTK_SET_NMEA_UPDATE_5SEC "$PMTK220,5000*1B"
#define PMTK_Q_RELEASE "$PMTK605*31"
#define PACKETHEADER "xdata="
#define GPS_UTC_ADJ -5  // Time zone
#define GPSSERIAL Serial1		// RX1/TX1
#define IMETSERIAL Serial2	// RX2/TX2
#define TESTSERIAL Serial3	// RX3/TX3
#define STD_HPA (1013.25)		// 29.92 - leave alone // Standard pressure
#define SEL_HPA (1012.00)		// LAUNCH SITE PRESSURE
#define STD_MSL 5486				// FL180  -express the altitude in feet
#define CHIPSELECT 10				// datalogger // Pin 10 is for chip select (CS) pin for a datalogger device
// remember we're replacing the DHT
//#define DHTTYPE DHT11  			// DHT version
//#define DHTPIN_INT 11				// any digital pins -
//#define DHTPIN_EXT 12				// - will work
#define PMAXBITS 87					// max packet length accepted by iMet 

// telemetry packet hash flags
#define ERRVAL 99						// on err detect, use this. to avoid using vals from undef. pointer addresses
#define SREGLEN 6						// how many sensors do we have?	
#define EREGLEN 8						// error register size
#define NOEXCEPT 0					// all-ok code
#define	PRECDEF 2						// default float precision
#define CHARIND 0						// char indicator code
#define INTIND 20						// int indicator code
#define SEPERATOR 'F'					// END DATA BEGIN STATUS BIT
#define PRECOVR 'E'					// PRECISION OVERRIDE
#define NEGSIG 'D'					// NEGATIVE
#define	CHARSIG 'C'					// CHAR
#define	INTSIG 'B'					// INT
#define ERRSIG 'A'					// ERROR

// timing function
#define INIT_CYC 30			// seconds initializer b4 obt. GPS fix (Before get valid GPS)
#define H_BOUND 20			// tropopause, km - thermodynamic gradient-stratification layer, that marks the end of the troposphere (bound)
#define H_LOWER 6				// below tropo (lower bound) in km
#define H_UPPER 4.63		// above tropo
#define E_0 8.85*pow(10, -12)	// electrical const - pow is to raise power _ 10^(-12)
#define REF_COND pow(10, -14)	// w/o coef
#define LEADING_COEF 5	// 'a' coef or time multiplier
#define MIN_TIME 5			// return this if T(alt) less than this
#define EXTEND_CYCLE		// hold new T(alt) til cycle completes to update,
												// rather than concurrently (thfr. reducing cycle
												// times during ascent)
#define MEAS_STAGE_1 2	// s
#define MEAS_STAGE_2 2	// s
#define RELAY_DELAY 5					// ms 	physical relay close
#define STD_SAMPLE_HZ 0.5			// 0.5 - 1 sample per 2 seconds
#define RAPID_SAMPLE_HZ 30		// sample at
#define RAPID_SAMPLE_MS 10000	// for // duration in milliseconds for which a rapid sample is taken 

#define GPSMAG_INTERVAL 2000	// time to update GPS, T(alt), transmit magn
#define SENSOR_INTERVAL 6000	// fetch sensor data, transmit iMet

/// CHANGE ME BACK TO 30000

#define FLUSH_INTERVAL	10000	// ensure buffer pushed - avoid data loss

// debug toggles
#define ECHOXDATA						// echo xdata packet on main serial
#define ECHOGPS							// echo GPS sentences on main serial
// #define DEBUGRECORD
#define SKIPGPSINIT
//#define TIMINGINSPECTOR			// disable most functionality, echo 


#if defined TIMINGINSPECTOR
#define MARKER(printable) { Serial.print(printable); Serial.print("\t"); return; } // MARKER() similar to user-built-in function in MATLAB
#else
#define MARKER(printable) // this one does not have the macro ({...}) with it , so nothing with happen
#endif
#if defined ECHOXDATA
#define ECHO(printable) Serial.print(printable) //User-Built-in function - simple so no need {}
#define ECHOLN(printable) Serial.println(printable) //Built-in function - simple --> no need {}
#else
#define ECHO(printable) //same as above, not work as not have macro
#define ECHOLN(printable) //same as above, nothing happen as not have macro
#endif


#define seconds() (millis()/1000) // millis - built-in fuction to get number of milliseconds elapsed since the program started running
// seconds() a user-built-in function to get seconds since program started 
// seconds() work as long as millis() work

File logger;
#define LOG(printable) { logger.print(printable); logger.print(','); }

uint32_t write_clock, sensor_clock, measure_clock, sample_clock, gpsmag_clock, sample_rapid_timer, safe_eject;
const int BOARD_ID = 4;

char buffer[20]; //char_characters
int var_iter = 0, len_iter = 0;


/*  sensor index:
			0 	INT   	accel				LSM303AGR
			1 	INT   	magn				LSM303AGR
			2	INT   	press, temp, hum	BME680
			3,4	INT,EXT	press				LPS25
			5,6	INT,EXT	abs. press			AMS 5812-0300-A
 */
// I2C addresses
//const int sensors[7] = {25, 30, 119, 93, 92, 120, 121}; //USIP IV
const int sensors[7] = {25, 30, 119, 93, 92}; //Eclipse

// sensor names
// USIP IV: const char* keys[SREGLEN] = {"LPS_I ", "LPS_E ", "LSM_I ", "BME_E ", "DHT_E ", "DHT_I "};
const char* keys[SREGLEN] = {"LPS_I ", "LPS_E ", "LSM_I ", "BME_E "}; //Eclipse

// [LPS_I, LPS_E, LSM_I, BME_E, DHT_E, DHT_I]
bool fails[SREGLEN]; //not include DHT for Eclipsee

const char* stat_c[EREGLEN] = {"OK", "FAIL ", "PLXC ", "NGPF ", "VDOP ", "DISC ", "WRITE", "PRECINV"};
bool warns[EREGLEN]; // each element in the array can store a true or false value.

Adafruit_GPS GPS(&GPSSERIAL); //create a new instance GPS and refer to existing GPSSERIAL
HardwareSerial &IMET = IMETSERIAL; // create a new instance IMET and refer to IMETSERIAL // Class_name instance_name(reference)
//DHT related 
//DHT dht_int(DHTPIN_INT, DHTTYPE);
//DHT dht_ext(DHTPIN_EXT, DHTTYPE);
Adafruit_LIS2MDL lsm = Adafruit_LIS2MDL(sensors[1]); // just sensor ID //create a new instance lsm, sensors[1] is refered to line 134
Adafruit_LPS25 lps_int, lps_ext; // create 2 new instances (names) 
Adafruit_BME680 bme; //create 1 new instance
sensors_event_t t_int, t_ext,p_int, p_ext, magn; //5 instances 
bool lps_int_started = false, lps_ext_started = false, bme_started = false, lsm_started = false, sd_started = false;
bool sampling_priority = false, probe1_posit = false, rapid_sample = false;
double sample_rate = STD_SAMPLE_HZ; // hz //double similar to float, higher precision

Ard2499 adc;
byte confChan = 0;

void reset(double &sval) {}
// error codes
// [0]OK [1](MASTER) FAIL [2]PLXC [3]NGPF [4]VDOP [5]DISC [6]WRITE [7]PRECINV
void warn(int errc) { 
	if (!warns[1]) {		// master fail
		warns[1] = true;
		len_iter++;
 	}
	if (!warns[errc]) len_iter++;
	warns[errc] = true; 
}

void sens_err(int target) {
	if (!fails[target]) len_iter++; //!fails[target] checks if the element at index target in the fails array is false.
	warn(5); //the sens_err function calls the warn function with the argument 5.
}

template<typename N> // floatlike
void reset(N &sval) { sval = ERRVAL; } // simple helper for pack expansion

// sensor fail IDs 
// [0]LPS_I [1]LPS_E [2]LSM_I [3]BME_E [4]DHT_E [5]DHT_I
// For Eclipse: [0]LPS_I [1]LPS_E [2]LSM_I [3]BME_E
void fail(int target) { // no svals -> init fail
	sens_err(target);
	fails[target] = true;
	return;
}

std::initializer_list<int> temp; // void returns, only used for parameter pack expansion
template<typename ... T>
void fail(int target, T&... svals) {
	sens_err(target);
	fails[target] = true;
	temp = {((void)reset(svals),0)...};
	return;
}

#define NUMVARS 16       //////// save the processor some counting
// transmitting variables
//DHT related
//float D_h_ext, D_h_int, D_t_ext, D_t_int;
float L_p_int, L_t_int, L_p_ext, L_t_ext;
float B_t_ext, B_p_ext, B_h_ext, B_g_ext, B_a_ext;
float mag_x, mag_y, mag_z;
float vdop;
char gpsbuf;

int prec[NUMVARS] = {}, vlen[NUMVARS] = {};

long status = 0, sfail = 0; 
int cyc_errs = 0, cyc_sfail = 0, cycle_freq = INIT_CYC;

long readVcc() {	// check battery?
	ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA,ADSC)); // measuring
	uint8_t low  = ADCL;// must read ADCL first - it then locks ADCH
	uint8_t high = ADCH; // unlocks both
	long result = (high<<8) | low;
	result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
	return result; // Vcc in millivolts
}

void indicate_safe(bool ok) {
	digitalWrite(LED1, ok ? HIGH : LOW);
  // If the argument ok is true, it turns on LED1 and turns off LED2.
//If the argument ok is false, it turns off LED1 and turns on LED2.
	digitalWrite(LED2, ok ? LOW : HIGH); 
}

// toggle red/green LEDs
void safe_poweroff(bool safe) {		
	safe_eject = millis();
	indicate_safe(1);
}


// helper functions to ensure correct format to imet

bool check(int str_len, int len_bit) {
	return (len_iter + str_len + len_bit) > PMAXBITS ? false : len_iter += str_len + len_bit; 
}

void debug_record(int vlen, int prec) { Serial.print("\t\t\tvlen: "); Serial.print(vlen); Serial.print("\t\t\tprec: "); Serial.println(prec); }

// append any data type to XDATA, datalog
char* to_record(int num) { // (-)237 → (2D) 02 37
	if (num == ERRVAL) { // sensor fail
		prec[var_iter] = ERRVAL;
		vlen[var_iter] = 1;
		if (!check(1, 1)) { warn(2); return "";}
		IMET.print("A");
		ECHO("A");
		LOG("ERR");
		#ifdef DEBUGRECORD
			debug_record(vlen[var_iter], prec[var_iter]);
		#endif
		var_iter++;
		return "";
	}
	LOG(num);
	char* val = itoa(num, buffer, 10);
	if (val[0] == '-') {prec[var_iter] = -20; val++; }
	else { prec[var_iter] = INTIND; }
	vlen[var_iter] = strlen(val);
	if (!check(vlen[var_iter], (num < 0) ? 3 : 2)) { warn(2); return val;};
	ECHO(val);
	#ifdef DEBUGRECORD
		Serial.print("\t\t\tvlen: "); Serial.print(vlen[var_iter]); Serial.print("\t\t\tprec: "); Serial.println(prec[var_iter]);
	#endif
	IMET.print(val);
	var_iter++;
	return val;
}
char* to_record(long int num) { // (-)237 → (2D) 02 37
	if (num == ERRVAL) { // sensor fail
		prec[var_iter] = ERRVAL;
		vlen[var_iter] = 1;
		if (!check(1, 1)) { warn(2); return "";}
		IMET.print("A");
		ECHO("A");
		LOG("ERR");
		#ifdef DEBUGRECORD
		debug_record(vlen[var_iter], prec[var_iter]);
		#endif
		var_iter++;
		return "";
	}
	LOG(num);
	char* val = ltoa(num, buffer, 10);
	if (val[0] == '-') {prec[var_iter] = -20; val++; }
	else { prec[var_iter] = INTIND; }
	vlen[var_iter] = strlen(val);
	if (!check(vlen[var_iter], (num < 0) ? 3 : 2)) { warn(2); return val;};
	ECHO(val);
	#ifdef DEBUGRECORD
	Serial.print("\t\t\tvlen: "); Serial.print(vlen[var_iter]); Serial.print("\t\t\tprec: "); Serial.println(prec[var_iter]);
	#endif
	IMET.print(val);
	var_iter++;
	return val;
}
bool to_record(double target, int precision = PRECDEF) { // (-)6.345 → (2D) 06 F3 03 45
	if (target == ERRVAL) {    // sensor fail
		prec[var_iter] = ERRVAL;
		vlen[var_iter] = 1;
		if (!check(1, 1)) { warn(2); return "";}
		IMET.print("A");
		ECHO("A");
		LOG("ERR");
		#ifdef DEBUGRECORD
			debug_record(vlen[var_iter], prec[var_iter]);
		#endif
		var_iter++;
		return false;
	}
	if (precision != PRECDEF) {
		if (precision > 9) { warn(7); precision = 9; }
		if (!precision || precision < 0) { warn(7);	precision = PRECDEF; }
		prec[var_iter] = precision;
	}
	LOG(target);
	if (target < 0) { prec[var_iter] *= -1; target = abs(target); }
	
	long final = ((long)(target * pow(10, precision) + 0.5));
	vlen[var_iter] += strlen(ltoa(final, buffer, 10));
	if (!check(vlen[var_iter], (prec[var_iter] < 0) ? ((precision != PRECDEF) ? 4 : 2) : ((precision != PRECDEF) ? 3 : 1))) { warn(2); return false;};

	ECHO(abs(final));
	IMET.print(abs(final));

	#ifdef DEBUGRECORD
		Serial.print("\t\t\tvlen: "); Serial.print(vlen[var_iter]); Serial.print("prec: "); Serial.println(prec[var_iter]);
	#endif
	var_iter++;
	return true;
}
// 1 letter = 2 hex bits. twice as long - is it truly dynamic enough to transmit?
char* to_record(char *str) {
	LOG(str);
	vlen[var_iter] = strlen(str) * 2;
	prec[var_iter] = CHARIND;
	if (!check(vlen[var_iter], 2)) { warn(2); return str;} ;
	for (int i = 0; i < strlen(str); i++) {
		ECHO((str[i], HEX));
		IMET.print(str[i], HEX);
	}
	#ifdef DEBUGRECORD
		Serial.print("\t\t\tvlen: "); Serial.print(vlen[var_iter]); Serial.print("\t\t\tprec: "); Serial.println(prec[var_iter]);
	#endif
	var_iter++;
	return str;
}
// AVOID String class if possible - memory fragmentation
char* to_record(String str) { 
	LOG(str);
	vlen[var_iter] = str.length() * 2;
	prec[var_iter] = CHARIND;
	str.toCharArray(buffer, 200);
	if (!check(vlen[var_iter], 2)) { warn(2); return buffer;};
	ECHO(buffer);
	#ifdef DEBUGRECORD
		Serial.print("\t\t\tvlen: "); Serial.print(vlen[var_iter]); Serial.print("\t\t\tprec: "); Serial.println(prec[var_iter]);
	#endif
	IMET.print(buffer);
	var_iter++;
	return buffer;
}

void sensor_check() {
		if (!lps_int_started) fail(0, L_p_int, L_t_int);
		if (!lps_ext_started) fail(1, L_p_ext, L_t_ext);
		if (!lsm_started) fail(2, mag_x, mag_y, mag_z);
		if (!bme_started) fail(3, B_t_ext, B_p_ext, B_h_ext, B_g_ext, B_a_ext);
		//if (isnan(D_h_ext) || isnan(D_t_ext)) fail(4, D_h_ext, D_t_ext);
		//if (isnan(D_h_int) || isnan(D_t_int)) fail(5, D_h_int, D_t_int);
		if (!sd_started) warn(6);
}

void open_packet() {
	// begin packet
	IMET.print(PACKETHEADER);
	ECHO(PACKETHEADER);
}

void close_packet() {
	ECHO(" ");
	ECHO(SEPERATOR);
	ECHO(SEPERATOR);
	ECHO(" ");
	

	IMET.print(SEPERATOR);
	IMET.print(SEPERATOR);
	if (warns[1]) {
		for (int err = 1; err < EREGLEN; err++) {
			if (warns[err]) {
				status += err * (int)(pow(10, cyc_errs) + 0.5);
				cyc_errs++;
			}
		}
		// code 6..1 shall immediately precede sfail if present
		if (warns[5]) { // SENSOR DISC
			for (int sens = 0; sens < SREGLEN; sens++) {
				if (fails[sens]) {
					sfail += sens * (long)(pow(10, cyc_sfail) + 0.5);
					cyc_sfail++;
				}
			}
		}
		ECHO(status);
		LOG(status);
		if (cyc_sfail) { ECHO(sfail); LOG(sfail); } // avoid false pasitive 0
		logger.print('\n');

		IMET.print(status); IMET.print(sfail);
		if (warns[5]) for (int i = 0; i < SREGLEN; i++) fails[i] = false;
		for (int i = 0; i < EREGLEN; i++) warns[i] = false;
		status = NOEXCEPT;
		sfail = cyc_errs = cyc_sfail = 0;
	}
	else {
		len_iter++;
		ECHO(NOEXCEPT);
		IMET.print(NOEXCEPT);

	}
	#ifdef ECHOXDATA
		Serial.print(SEPERATOR);
		for (int var = 0; var < NUMVARS; var++) {
			if (!vlen[var]) break;
			if (prec[var] == ERRVAL) { Serial.print(ERRSIG); continue; }
			if (prec[var] != PRECDEF) {
				if (prec[var] < 0) { Serial.print(NEGSIG); prec[var] = abs(prec[var]); }
				if (prec[var] == CHARIND) { Serial.print(CHARSIG); }
				else if (prec[var] == INTIND) { Serial.print(INTSIG); }
				else if (prec[var] != PRECDEF) { Serial.print(PRECOVR); Serial.print(prec[var]); }; // allow for -2 → 2 skipping, etc.
				// 				^^^^^^^^^^^^^^^^^^ this is already the if condition ???
			}
			Serial.print(vlen[var]);
		}
	#endif

	IMET.print(SEPERATOR);
	for (int var = 0; var < NUMVARS; var++) {
		if (!vlen[var]) break;
		if (prec[var] == ERRVAL) { IMET.print(ERRSIG); continue; }
		if (prec[var] != PRECDEF) {
			if (prec[var] < 0) { IMET.print(NEGSIG); prec[var] = abs(prec[var]); }
			if (prec[var] == CHARIND) { IMET.print(CHARSIG); }
			else if (prec[var] == INTIND) { IMET.print(INTSIG); }
			else if (prec[var] != PRECDEF) { IMET.print(PRECOVR); IMET.print(prec[var]); };         // allow for -2 → 2 skipping, etc.
		}
		IMET.print(vlen[var]);
	}
	
	for (int i = 0; i < NUMVARS; i++) { prec[i] = 2; vlen[i] = 0;	}

	if (len_iter % 2)	IMET.print(SEPERATOR); // even it out.
	IMET.print("\r\n"); // terminates the packet
	
	#ifdef ECHOXDATA
		if (len_iter % 2) Serial.print(SEPERATOR); 
		Serial.print("\niteration packet length: "); Serial.println(len_iter);
		if (len_iter % 2) Serial.println("+1 bit of padding added");
	#endif
}

#ifdef EXTEND_CYCLE
int new_cycle_hold = 0;
#endif
#ifdef TIMINGINSPECTOR
int last_tick = 0;
#endif

bool stage_done[3] = {0};

bool sample() {
	ECHOLN("\nSAMPLE");
	sample_clock = millis();

	// for (int i = 0; i < 6; i++) {
	// 	byte newChannel;
		
	// 	switch(i) {
	// 		case 0:
	// 			newChannel = LTC2499_CHAN_DIFF_2P_3N;
	// 			break;
	// 		case 1:
	// 			newChannel = LTC2499_CHAN_SINGLE_2P;
	// 			break;
	// 		case 2:
	// 			newChannel = LTC2499_CHAN_SINGLE_3P;
	// 			break;
	// 		case 3:
	// 			newChannel = LTC2499_CHAN_SINGLE_4P;
	// 			break;
	// 		case 4:
	// 			newChannel = LTC2499_CHAN_SINGLE_5P;
	// 			break;
	// 		case 5:
	// 		default:
	// 			newChannel = LTC2499_CHAN_SINGLE_0P;
	// 			break;
	// 	}
	// 	Serial.print("[Channel ");
	// 	Serial.print(i);
	// 	Serial.print("]: ");
	// 	Serial.print((double)(4.096 - ((adc.ltc2499ReadAndChangeChannel(newChannel) * 4.096) / 16777215ul)));
	// 	Serial.print(" V\n");
	// }
	bool toggle = false;

	double calibration = 0.009;
	// do this on the ground; for testing purposes only

	for (int i = 0; i < 2; i++) {
		byte newChannel = toggle ? LTC2499_CHAN_DIFF_0P_1N : LTC2499_CHAN_DIFF_2P_3N;
		// double convReading = adc.ltc2499ReadVoltage();
		double reading = adc.ltc2499ReadAndChangeChannel(newChannel);
		Serial.print("\n[Diff ");
		Serial.print(toggle ? "2+ 3-" : "0+ 1-");
		Serial.print("]\nreading: ");
		Serial.print(reading);
		// Serial.print("\t  volt fx: ");				
		// Serial.print(convReading,4);
		// Serial.print("\t  eq1: ");				
		// Serial.print((double)(4.096 - ((reading * 4.096) / 16777215ul)));
		// Serial.print("\t  eq2: ");				
		// Serial.print((double)(4.096 - ((reading * 2.048) / 16777215ul)));
		Serial.print("\t  eq3: ");				
		Serial.print((double)(((reading * 2.048) / 16777215ul)) + calibration,4);
		adc.ltc2499ChangeChannel(newChannel);
		toggle = !toggle;
	}



	#

	// ECHO("S ");
}

// final sample - reset for next measurement
bool last_sample() {
	sample_clock = millis();
	sample_rate = STD_SAMPLE_HZ;
	sampling_priority = false;
	rapid_sample = false;

	measure_clock = seconds(); 			// leave here or move to second 0?
	#ifdef EXTEND_CYCLE							//
		cycle_freq = new_cycle_hold;	//	leave here: more time between cycles
	#endif													// 	move to begin. of cycle: shorter delay, may overlap
	for (int i = 0; i < 3; i++) stage_done[i] = false;
	MARKER("BURST COMPLETE");
	ECHOLN("\nFINAL BURST SAMPLE");

	cycle_freq = 60;
	ECHO("CYCLE FREQUENCY SET TO "); ECHO(cycle_freq); ECHOLN(" s");
}

int upd_cyc_freq(int alt = 0) { 	// KM
	// replace "false" with condition for bad gps alt confidence
	// e.g. vdop > 200 or !GPS.fix
	if (false && bme_started) alt = bme.readAltitude(alt > STD_MSL ? STD_HPA : SEL_HPA);
	return LEADING_COEF*((E_0/(REF_COND*exp(alt/(alt < H_BOUND ? H_LOWER : H_UPPER)))));
}

void cycle_stage_0() {		
	sampling_priority = stage_done[0] = true;
	MARKER("MEASUREMENT [0s]");
	ECHOLN("\n[0s] MEASUREMENT CYCLE\n");
	digitalWrite(CAL, HIGH);	// open relays to GND & multiplexer
	digitalWrite(EN, HIGH); 
}

void cycle_stage_2(bool &probe1_posit) {
	stage_done[1] = true;
	MARKER("MEASUREMENT [2s]");
	ECHOLN("\n[2s] MEASUREMENT CYCLE\n");
	digitalWrite(A0, probe1_posit ? HIGH : LOW);	// lines to A0, A1 interpreted
	digitalWrite(A1, probe1_posit ? LOW : HIGH);	// by AD7502 as binary
	probe1_posit = !probe1_posit;
}

void cycle_stage_4() {
	stage_done[2] = true;
	sample_rate = RAPID_SAMPLE_HZ;
	sample_rapid_timer = millis();
	rapid_sample = true;
	MARKER("MEASUREMENT [4s]");
	ECHOLN("\n[4s] MEASUREMENT CYCLE\n");
	digitalWrite(CAL, LOW);
	delay(RELAY_DELAY); 			// min. 4ms required for relays to physically close
	digitalWrite(EN, LOW);		// close multiplexer
	// immediately sample 30Hz for 10s, then reset
}


// =====================================================

void setup() {
	Serial.begin(115200);
	IMET.begin(9600);
	GPS.begin(9600);
	while(!Serial) delay(10);
	Wire.begin();
  //DHT related
	//dht_int.begin();
	//dht_ext.begin();
	adc.begin(ARD2499_ADC_ADDR_ZZZ, ARD2499_EEP_ADDR_ZZ);
	adc.ltc2499ChangeConfiguration(LTC2499_CONFIG2_60_50HZ_REJ);
	adc.ltc2499ChangeChannel(LTC2499_CHAN_SINGLE_0P);


	const int out_pin_n = 7; 
	int output_pins[out_pin_n] = { CHIPSELECT, EN, CAL, A0, A1, LED1, LED2};
	for (int i = 0; i < out_pin_n; i++) pinMode(output_pins[i], OUTPUT);	

	for (int i = 0; i < NUMVARS; i++) { prec[i] = 2; vlen[i] = 0;	}

	ECHO("\n[START]\n");
	ECHO("Initializing sensors... ");
	if (lsm.begin()) lsm_started = true;
	if (lps_int.begin_I2C(sensors[3])) lps_int_started = true;
	if (lps_ext.begin_I2C(sensors[4])) lps_ext_started = true;
	lps_int.setDataRate(LPS25_RATE_1_HZ); lps_ext.setDataRate(LPS25_RATE_1_HZ);
	if (bme.begin()) bme_started = true;
	delay(500);
	lsm.enableAutoRange(true);
	bme.setTemperatureOversampling(BME680_OS_8X);
	bme.setHumidityOversampling(BME680_OS_2X);
	bme.setPressureOversampling(BME680_OS_4X);
	bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
	bme.setGasHeater(320, 150);         // 320*C for 150 ms
	ECHOLN("DONE");
	ECHO("Initializing datalogger...");
	if (SD.begin(CHIPSELECT)) sd_started = true;
	char filename[] = "LOG00.CSV";
	for (uint8_t i = 0; i < 100; i++) {
		#ifdef TIMINGINSPECTOR
		new_cycle_hold = upd_cyc_freq(20);
		break;
		#endif
	  filename[3] = i/10 + '0';
	  filename[4] = i%10 + '0';
	  if (!SD.exists(filename)) {
		  logger = SD.open(filename, FILE_WRITE);
			//DHT related 
      //USIP IV: LOG("TIME,GPS FIX,GPS ALT,T(ALT),D_h_ext,D_h_int,L_t_ext,L_p_ext,L_t_int,L_p_int,B_t_ext,B_p_ext,B_h_ext,B_g_ext,B_a_ext,status code,mag_x,mag_y,mag_z,samples 1,2,3,status code");
			LOG("TIME,GPS FIX,GPS ALT,T(ALT),L_t_ext,L_p_ext,L_t_int,L_p_int,B_t_ext,B_p_ext,B_h_ext,B_g_ext,B_a_ext,status code,mag_x,mag_y,mag_z,samples 1,2,3,status code"); //Eclipse
      logger.print('\n');
		  break;
	  }
  }
	if (!logger) sd_started = false;
	ECHOLN(sd_started ? "DONE" : "FAIL");
	ECHO("Initializing GPS... ");
	#ifndef SKIPGPSINIT
	delay(4000);
	#endif
	// GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5SEC);
	delay(1000);
	ECHOLN("DONE");

	write_clock = sensor_clock = measure_clock = sample_clock = gpsmag_clock = sample_rapid_timer = 0;
}


/////////////////////////////////////////////////////////////
/////////////////////////// LOOP ////////////////////////////
/////////////////////////////////////////////////////////////
	
void loop() {
	

	// delay(2000);
	#ifdef TIMINGINSPECTOR
	if (seconds() > last_tick) { 
		Serial.print("\n["); Serial.print(seconds()); Serial.print("]  T: "); Serial.print(cycle_freq); Serial.print("\t");
		last_tick = seconds();
	}
	#endif

	// // check gps line every single tick
	// gpsbuf = GPS.read();
	// #ifdef ECHOGPS 
	// 	Serial.write(gpsbuf);
	// #endif

	// if (GPS.newNMEAreceived()) {
	// 	if (!GPS.parse(GPS.lastNMEA())) {
	// 		// return;		//just wait for complete NMEA
	// 	}
	// }

	//////////////////////////////////////////////////////////////////
	/// FINAL: rapid samples complete - return to regular sample cycle
	if (millis() - sample_clock > 1000/sample_rate) {
		if (rapid_sample && millis() - sample_rapid_timer > RAPID_SAMPLE_MS) last_sample();
		else sample();
	}

	//////////////////////////////////////////////////////
	/// MEASURE: probe measurements - take priority
	if (!stage_done[0] && (seconds() - measure_clock > cycle_freq))
		cycle_stage_0();
	if (!stage_done[1] && (seconds() - measure_clock > cycle_freq + MEAS_STAGE_1))
		cycle_stage_2(probe1_posit);
	if (!stage_done[2] && (seconds() - measure_clock > cycle_freq + MEAS_STAGE_1 + MEAS_STAGE_2))
		cycle_stage_4();
	
	//////////////////////////////////////////////////////
	/// [Q 2sec] PACKET A:  GPS UPD, MAGNETOMETER, SAMPLES
	if (!sampling_priority && millis() - gpsmag_clock > GPSMAG_INTERVAL) {
		if ((millis() - sensor_clock > SENSOR_INTERVAL - 2000) || (millis() - sensor_clock < 2000)) {
    
	}
	else {
    
  
	// 	gpsmag_clock = millis();
	// 	var_iter = 0, len_iter = 3;                 // FF to delimit status & lenbit
	// 	MARKER("PING GPS, MAGN, SAMPLE");
		
		// if (!check(sample data size, sample data length)) IMET.print("\r\n");
		#ifndef EXTEND_CYCLE
		// cycle_freq = upd_cyc_freq(GPS.altitude / 1000);
			cycle_freq = upd_cyc_freq(10);
			cycle_freq = cycle_freq < MIN_TIME ? MIN_TIME : cycle_freq;
		#else
		// new_cycle_hold = upd_cyc_freq(GPS.altitude / 1000);
			new_cycle_hold = upd_cyc_freq(10);
			new_cycle_hold = new_cycle_hold < MIN_TIME ? MIN_TIME : new_cycle_hold;
		#endif

  //  why is all this commented out lmao?? - mohamed

	// 	// vdop = GPS.HDOP * 1.38 * 23; // +- m
	// 	// inferred from HDOP, * 23m - GPGGA doesn't include VDOP
	// 	// The_Distributions_of_HDOP_and_VDOP_in_GNSS_and_a_Corresponding_New_Algorithm_of_Fast_Selecting_Satellites
	// 	// 10.1007/978-3-642-29175-3_37

	// 	// if (vdop > 100) warn(4);
	// 	// if (!GPS.fix) warn(3);
		
	// 	// lsm.getEvent(&magn);
	// 	// mag_x = magn.magnetic.x;
	// 	// mag_y = magn.magnetic.y;
	// 	// mag_z = magn.magnetic.z;
		
	// 	// sensor_check(); // check sensors for status, zero out fails

	// 	// #ifdef ECHOXDATA
	// 	// Serial.print("LSM magn  x: "); Serial.print(mag_x);
	// 	// Serial.print("  y: "); Serial.print(mag_y);
	// 	// Serial.print("  z: "); Serial.println(mag_z);		//uT

	// 	// Serial.print("\nGPS ALT:  "); Serial.println(GPS.altitude);
	// 	// Serial.print("GPS TIME: "); Serial.print((GPS.hour + GPS_UTC_ADJ) < 0 ? 24 + GPS.hour + GPS_UTC_ADJ : GPS.hour + GPS_UTC_ADJ); Serial.print(':'); Serial.print(GPS.minute < 10 ? "0" : ""); Serial.print(GPS.minute); Serial.print(':'); Serial.print((GPS.seconds + (int)GPS.secondsSinceTime()) < 10 ? "0" : ""); Serial.println(GPS.seconds + (int)GPS.secondsSinceTime());
	// 	// Serial.print("FIX:  "); Serial.print(GPS.fix ? "TRUE, " : "FALSE, "); Serial.print(GPS.satellites); Serial.println(" sats");
	// 	// Serial.print("VDOP actual: +-"); Serial.println(vdop);

	// 	// Serial.print("T(alt): "); Serial.print(cycle_freq); Serial.println(" s");
	// 	// Serial.println();
	// 	// #endif

	// 	// logger.print((GPS.hour + GPS_UTC_ADJ) < 0 ? 24 + GPS.hour + GPS_UTC_ADJ : GPS.hour + GPS_UTC_ADJ); logger.print(':'); logger.print(GPS.minute < 10 ? "0" : ""); logger.print(GPS.minute); logger.print(':'); logger.print((GPS.seconds + (int)GPS.secondsSinceTime()) < 10 ? "0" : ""); LOG(GPS.seconds + (int)GPS.secondsSinceTime());
	// 	// LOG(GPS.fix ? GPS.satellites : 0);
	// 	// LOG(GPS.altitude); LOG(cycle_freq);
		
	// 	// LOG(",,,,,,,,,,,");

	// 	// open_packet();
	// 	// to_record(mag_x);
	// 	// to_record(mag_y);
	// 	// to_record(mag_z);

	// 	// to_record(392396L);
	// 	// to_record(782132L);
	// 	// to_record(532894L);

	// 	// close_packet();

		}
	}

	// ///////////////////////////////////////////////////////
	// /// [Q 30sec] PACKET B:  GPS UPD, MAGNETOMETER, SAMPLES
	// if (!sampling_priority && millis() - sensor_clock > SENSOR_INTERVAL) {
	// 	sensor_clock = millis();
	// 	MARKER("SENSOR SNAPSHOT");
		
	// 	var_iter = 0, len_iter = 3; // FF to delimit status & lenbit

	// 	//  reading from sensors. any data type
	// 	unsigned long endMeas = bme.beginReading();
	// 	D_h_ext = dht_ext.readHumidity();
	// 	D_h_int = dht_int.readHumidity();
	// 	D_t_ext = dht_ext.readTemperature();
	// 	D_t_int = dht_int.readTemperature();
		
	// 	lps_int.getEvent(&p_int, &t_int);
	// 	lps_ext.getEvent(&p_ext, &t_ext);
	// 	L_p_int = p_int.pressure;
	// 	L_t_int = t_int.temperature;
	// 	L_p_ext = p_ext.pressure;
	// 	L_t_ext = t_ext.temperature;

	// 	if ((endMeas == 0) || (!bme.endReading())) fail(3, B_t_ext, B_p_ext, B_h_ext, B_g_ext, B_a_ext);
	// 	else {
	// 		B_t_ext = bme.temperature;
	// 		B_p_ext = bme.pressure / 100.0;
	// 		B_h_ext = bme.humidity;
	// 		B_g_ext = bme.gas_resistance / 1000.0;
	// 		B_a_ext = bme.readAltitude(GPS.altitude > STD_MSL ? STD_HPA : SEL_HPA);
	// 	}

	// 	sensor_check();

	// 	#ifdef ECHOXDATA
	// 	Serial.print("\n\n["); Serial.print((GPS.hour + GPS_UTC_ADJ) < 0 ? 24 + GPS.hour + GPS_UTC_ADJ : GPS.hour + GPS_UTC_ADJ); Serial.print(':'); Serial.print(GPS.minute < 10 ? "0" : ""); Serial.print(GPS.minute); Serial.print(':'); Serial.print((GPS.seconds + (int)GPS.secondsSinceTime()) < 10 ? "0" : ""); Serial.print(GPS.seconds + (int)GPS.secondsSinceTime()); Serial.println("]");
	// 	Serial.println("\t temp\tpres\thum");
	// 	Serial.print("DHT_ext  "); Serial.print(D_t_ext); Serial.print("\t\t"); Serial.println(D_h_ext);
	// 	Serial.print("DHT_int  "); Serial.print(D_t_int); Serial.print("\t\t"); Serial.println(D_h_int);
	// 	Serial.print("LPS_ext  "); Serial.print(L_t_ext); Serial.print("\t"); Serial.println(L_p_ext);
	// 	Serial.print("LPS_int  "); Serial.print(L_t_int); Serial.print("\t"); Serial.println(L_p_int);
	// 	Serial.print("BME_ext  "); Serial.print(B_t_ext); Serial.print("\t"); Serial.print(B_p_ext); Serial.print("\t"); Serial.println(B_h_ext);
	// 	Serial.print("\tgas res. "); Serial.print(B_g_ext);         // KOhms
	// 	Serial.print("\taltim  "); Serial.println(B_a_ext);
	// 	Serial.println();
	// 	#endif

	// 	logger.print((GPS.hour + GPS_UTC_ADJ) < 0 ? 24 + GPS.hour + GPS_UTC_ADJ : GPS.hour + GPS_UTC_ADJ); logger.print(':'); logger.print(GPS.minute < 10 ? "0" : ""); logger.print(GPS.minute); logger.print(':'); logger.print((GPS.seconds + (int)GPS.secondsSinceTime()) < 10 ? "0" : ""); LOG(GPS.seconds + (int)GPS.secondsSinceTime());
	// 	LOG(GPS.fix ? GPS.satellites : 0);
	// 	LOG(GPS.altitude); LOG(cycle_freq);
		  
	// 	open_packet();

	// 	// send to iMet
	// 	// to_record(D_t_ext);
	// 	to_record(D_h_ext);
	// 	// to_record(D_t_int);
	// 	to_record(D_h_int);
	// 	to_record(L_t_ext);
	// 	to_record(L_p_ext);
	// 	to_record(L_t_int);
	// 	to_record(L_p_int);
	// 	to_record(B_t_ext);
	// 	to_record(B_p_ext);
	// 	to_record(B_h_ext);
	// 	to_record(B_g_ext);
	// 	to_record(B_a_ext);

	// 	close_packet();

	// }

	// //////////////////////////////////////////////////////
	// /// FLUSH: data from cache -> card
	// /// moderate; I/O, power & time - BLOCKING!
	// if (!sampling_priority && (millis() - write_clock) > FLUSH_INTERVAL) {
	// 	safe_poweroff(false);
	// 	write_clock = millis(); 
	// 	MARKER("DATALOGGER FLUSH");
	// 	logger.flush();
	// 	ECHOLN("DATALOGGER FLUSH");
	// 	// watch for SPI LED
	// 	if (logger.getWriteError()) warn(6);
	// 	safe_poweroff(true);
	// }

	// /// indicate optimal time to remove SD, avoid middle of write
	// if (safe_eject && (millis() - safe_eject) > FLUSH_INTERVAL / 2.5) {
	// 	indicate_safe(0);
	// 	safe_eject = 0;
	// }
	
}
