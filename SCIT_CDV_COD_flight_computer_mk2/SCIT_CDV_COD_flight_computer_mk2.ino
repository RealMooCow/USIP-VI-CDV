#include <SD.h>
#include "SPI.h"
// #include <AMS.h>
#include "DHT.h"
#include <Wire.h>
#include <Ard2499.h>
#include <Adafruit_GPS.h>
// #include <Arduino_AVRSTL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LPS2X.h> // is this the right sensor?
#include <Adafruit_BME680.h>
#include <Adafruit_LIS2MDL.h>

// use AVR 1.8.2

/// GPS: NMEA (National Marine Electronics Association) sentences sent as commands
#define PMTK_SET_NMEA_OUTPUT_GGAONLY "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_UPDATE_1SEC "$PMTK220,1000*1B"
#define PMTK_SET_NMEA_UPDATE_2SEC "$PMTK220,2000*1B"
#define PMTK_SET_NMEA_UPDATE_5SEC "$PMTK220,5000*1B"
#define PMTK_Q_RELEASE "$PMTK605*31"

/// SERIAL:
#define PACKETHEADER "xdata="
#define GPS_UTC_ADJ -5
#define GPSSERIAL Serial1		// RX1/TX1
#define LORASERIAL Serial2	// RX2/TX2
#define IMETSERIAL Serial3	// RX3/TX3

/// SENSORS:
#define STD_HPA (1013.25)		// 29.92 - leave alone
#define SEL_HPA (1012.00)		// LAUNCH SITE PRESSURE
#define STD_MSL 5486				// FL180
#define DHTTYPE DHT11  			// DHT version
#define DHTPIN_INT 11				// any digital pins -
#define DHTPIN_EXT 12				// - will work
#define PMAXBITS 87					// max packet length accepted by iMet 

/// PINS:		ensure all are set to output
#define CHIPSELECT 10				// datalogger board default cs pin
#define MULTIPLEXER_EN 13
#define LED1 6
#define LED2 7
#define EN A4
#define CAL A3

#define OUT_PINS 8 
int output_pins[OUT_PINS] = { CHIPSELECT, A0, A1, EN, CAL, LED1, LED2, MULTIPLEXER_EN };

/// TELEMETRY: packet hash flags
#define ERRVAL 99						// on err detect, use this. to avoid using vals from undef. pointer addresses
#define MARGIN 750					// ms keep A & B packets from clashing
#define SREGLEN 6						// how many sensors do we have?	
#define EREGLEN 8						// error register size
#define NOEXCEPT 0					// all-ok code
#define	PRECDEF 2						// default float precision
#define SEPERATOR 'F'				// END DATA BEGIN STATUS BIT
#define ERRSIG 'E'					// ERROR
#define NEGSIG 'D'					// prepended to var_len if neg

/// CONDUCTIVITY: timing function + ADC settings
#define INIT_CYC 10			// seconds initializer b4 obt. GPS fix
#define H_BOUND 20			// tropopause, km
#define H_LOWER 6				// below tropo
#define H_UPPER 4.63		// above tropo
#define E_0 8.85*pow(10, -12)	// electrical const
#define REF_COND pow(10, -14)	// w/o coef
#define LEADING_COEF 5	// 'a' coef or time multiplier
#define MIN_TIME 5			// return this if T(alt) less than this
// #define EXTEND_CYCLE		// hold new T(alt) til cycle completes to update,
												// rather than concurrently (thfr. reducing cycle
												// times during ascent)
#define MEAS_STAGE_1 2	// s
#define MEAS_STAGE_2 2	// s
#define RELAY_DELAY 5					// ms 	physical relay close
#define STD_SAMPLE_HZ 1			// 0.5 - 1 sample per 2 seconds
#define RAPID_SAMPLE_HZ 30		// sample at
#define RAPID_SAMPLE_MS 10000	// for

#define ADCMAG_INTERVAL 1000	// transmit magn + ADC
#define SENSOR_INTERVAL 4000	// update GPS -> T(alt), fetch sensor data, transmit iMet
/// CHANGE ME BACK TO 30000

#define FLUSH_INTERVAL	10000	// ensure buffer pushed - avoid data loss
															// too often though, high power / I/O bus consumption

// debug toggles
#define ECHO_DEBUG						// echo xdata packet on main serial
#define ECHOGPS								// echo GPS sentences on main serial
// #define DEBUGRECORD
// #define SKIPGPSINIT
//#define TIMINGINSPECTOR			// disable most functionality, echo 
#define ENABLELORA						// enable transmitting to LoRa radio

#if defined TIMINGINSPECTOR
	#define MARKER(printable) { Serial.print(printable); Serial.print("\t"); return; }
#else
	#define MARKER(printable)
#endif

#if defined ECHO_DEBUG
	#define ECHO(printable) Serial.print(printable)
	#define ECHOB(printable, base) Serial.print(printable, base)
	#define ECHOLN(printable) Serial.println(printable)
#else
	#define ECHO(printable)
	#define ECHOLN(printable)
#endif

#define seconds() (millis()/1000)

// File logger;
#define LOG(printable) { logger.print(printable); logger.print(','); }
#define LOG_APPEND(printable) logger.print(printable);

uint32_t write_clock, sensor_clock, measure_clock, sample_clock, adcmag_clock, sample_rapid_timer, safe_eject;
const int BOARD_ID = 4;

char buffer[20];
int variable_iter = 0, length_iter = 0;


/*  sensor index:
			0 	INT   	accel				LSM303AGR
			1 	INT   	magn				LSM303AGR
			2		INT   	press, temp, hum	BME680
			3,4	INT,EXT	press				LPS25
			5,6	INT,EXT	abs. press			AMS 5812-0300-A
 */
// I2C addresses
const int sensors[7] = {25, 30, 119, 93, 92, 120, 121};

// sensor names
const char* keys[SREGLEN] = {"LPS_I ", "LPS_E ", "LSM_I ", "BME_E ", "DHT_E ", "DHT_I "};

// [LPS_I, LPS_E, LSM_I, BME_E, DHT_E, DHT_I]
bool fails[SREGLEN];

const char* stat_c[EREGLEN] = {"OK", "FAIL ", "PLXC ", "NGPF ", "VDOP ", "DISC ", "WRITE", "PRECINV"};
bool warns[EREGLEN];

Ard2499 adc;
Adafruit_GPS GPS(&GPSSERIAL);
HardwareSerial &RADIO = LORASERIAL;
HardwareSerial &IMET = IMETSERIAL;
DHT dht_int(DHTPIN_INT, DHTTYPE);
DHT dht_ext(DHTPIN_EXT, DHTTYPE);
Adafruit_LIS2MDL lsm = Adafruit_LIS2MDL(sensors[1]); // just sensor ID
Adafruit_LPS25 lps_int, lps_ext;
Adafruit_BME680 bme;
sensors_event_t t_int, t_ext,p_int, p_ext, magn;
bool lps_int_started = false, lps_ext_started = false, bme_started = false, lsm_started = false, sd_started = false;
bool sampling_priority = false, probe1_posit = false, rapid_sample = false;
double sample_rate = STD_SAMPLE_HZ; // hz

#if defined ECHO_DEBUG				// returns # bytes
#define IMET_TX(printable) IMET.print(printable); ECHO(printable);
#define IMET_TXB(printable, base) IMET.print(printable, base); ECHOB(printable, base); 
#define IMET_TXLN(printable) IMET.println(printable); ECHOLN(printable); 
#else
#define IMET_TX(printable) IMET.print(printable);
#define IMET_TXB(printable, base) IMET.print(printable, base);
#define IMET_TXLN(printable) IMET.println(printable);
#endif

void reset(double &sval) {}
// error codes
// [0]OK [1](MASTER) FAIL [2]PLXC [3]NGPF [4]VDOP [5]DISC [6]WRITE [7]PRECINV
void warn(int errc) { 
	if (!warns[1]) {		// master fail
		warns[1] = true;
		length_iter++;
 	}
	if (!warns[errc]) length_iter++;
	warns[errc] = true; 
}

void sens_err(int target) {
	if (!fails[target]) length_iter++;
	warn(5);
}

template<typename N> // floatlike
void reset(N &sval) { sval = ERRVAL; } // simple helper for pack expansion

// sensor fail IDs 
// [0]LPS_I [1]LPS_E [2]LSM_I [3]BME_E [4]DHT_E [5]DHT_I
void fail(int target) { // no svals -> init fail
	sens_err(target);
	fails[target] = true;
	return;
}

void fail(int target, float &sval) {
	sens_err(target);
	fails[target] = true;
	reset(sval);
}
void fail(int target, float &sval, float &sval1) {
	sens_err(target);
	fails[target] = true;
	reset(sval); reset(sval1);
}
void fail(int target, float &sval, float &sval1, float &sval2) {
	sens_err(target);
	fails[target] = true;
	reset(sval); reset(sval1); reset(sval2);
}
void fail(int target, float &sval, float &sval1, float &sval2, float &sval3) {
	sens_err(target);
	fails[target] = true;
	reset(sval); reset(sval1); reset(sval2); reset(sval3);
}
void fail(int target, float &sval, float &sval1, float &sval2, float &sval3, float &sval4) {
	sens_err(target);
	fails[target] = true;
	reset(sval); reset(sval1); reset(sval2); reset(sval3); reset(sval4);
}

// std::initializer_list<int> temp; // void returns, only used for parameter pack expansion
// template<typename ... T>
// void fail(int target, T&... svals) {
// 	sens_err(target);
// 	fails[target] = true;
// 	temp = {((void)reset(svals),0)...};
// 	return;
// }

#define NUMVARS 16       //////// needs to be constant
// transmitting variables
float D_h_ext, D_h_int, D_t_ext, D_t_int;
float L_p_int, L_t_int, L_p_ext, L_t_ext;
float B_t_ext, B_p_ext, B_h_ext, B_g_ext, B_a_ext;
float mag_x, mag_y, mag_z;
float vdop;
char gpsbuf;

int prec[NUMVARS] = {}, vlen[NUMVARS] = {};

long status = 0, sfail = 0; 
int cyc_errs = 0, cyc_sfail = 0, cycle_freq = INIT_CYC;


void indicate_safe(bool ok) {
	digitalWrite(LED1, ok ? HIGH : LOW);
	digitalWrite(LED2, ok ? LOW : HIGH);
}

// toggle red/green LEDs
void safe_poweroff(bool safe) {		
	safe_eject = millis();
	indicate_safe(1);
}


// helper functions to ensure correct format to imet

bool check(int str_len, int len_bit) {
	return (length_iter + str_len + len_bit) > PMAXBITS ? false : length_iter += str_len + len_bit; 
}

void debug_record(int vlen, int prec) { Serial.print("\t\t\tvlen: "); Serial.print(vlen); Serial.print("\t\t\tprec: "); Serial.println(prec); }


//////////////////////////////////////////////////////////////////////////////////////////
///////////////////// OLD: FOR REFERENCE ONLY ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// append any data type to XDATA, datalog
// char* to_record(int num) { // (-)237 → (2D) 02 37
// 	if (num == ERRVAL) { // sensor fail
// 		prec[variable_iter] = ERRVAL;
// 		vlen[variable_iter] = 1;
// 		if (!check(1, 1)) { warn(2); return "";}
// 		IMET.print("A");
// 		ECHO("A");
// 		LOG("ERR");
// 		#ifdef DEBUGRECORD
// 			debug_record(vlen[variable_iter], prec[variable_iter]);
// 		#endif
// 		variable_iter++;
// 		return "";
// 	}
// 	LOG(num);
// 	char* val = itoa(num, buffer, 10);
// 	if (val[0] == '-') {prec[variable_iter] = -20; val++; }
// 	else { prec[variable_iter] = INTIND; }
// 	vlen[variable_iter] = strlen(val);
// 	if (!check(vlen[variable_iter], (num < 0) ? 3 : 2)) { warn(2); return val;};
// 	ECHO(val);
// 	#ifdef DEBUGRECORD
// 		Serial.print("\t\t\tvlen: "); Serial.print(vlen[variable_iter]); Serial.print("\t\t\tprec: "); Serial.println(prec[variable_iter]);
// 	#endif
// 	IMET.print(val);
// 	variable_iter++;
// 	return val;
// }
// char* to_record(long int num) { // (-)237 → (2D) 02 37
// 	if (num == ERRVAL) { // sensor fail
// 		prec[variable_iter] = ERRVAL;
// 		vlen[variable_iter] = 1;
// 		if (!check(1, 1)) { warn(2); return "";}
// 		IMET.print("A");
// 		ECHO("A");
// 		LOG("ERR");
// 		#ifdef DEBUGRECORD
// 		debug_record(vlen[variable_iter], prec[variable_iter]);
// 		#endif
// 		variable_iter++;
// 		return "";
// 	}
// 	LOG(num);
// 	char* val = ltoa(num, buffer, 10);
// 	if (val[0] == '-') {prec[variable_iter] = -20; val++; }
// 	else { prec[variable_iter] = INTIND; }
// 	vlen[variable_iter] = strlen(val);
// 	if (!check(vlen[variable_iter], (num < 0) ? 3 : 2)) { warn(2); return val;};
// 	ECHO(val);
// 	#ifdef DEBUGRECORD
// 	Serial.print("\t\t\tvlen: "); Serial.print(vlen[variable_iter]); Serial.print("\t\t\tprec: "); Serial.println(prec[variable_iter]);
// 	#endif
// 	IMET.print(val);
// 	variable_iter++;
// 	return val;
// }
// bool to_record(double target, int precision = PRECDEF) { // (-)6.345 → (2D) 06 F3 03 45
// 	if (target == ERRVAL) {    // sensor fail
// 		prec[variable_iter] = ERRVAL;
// 		vlen[variable_iter] = 1;
// 		if (!check(1, 1)) { warn(2); return "";}
// 		IMET.print("A");
// 		ECHO("A");
// 		LOG("ERR");
// 		#ifdef DEBUGRECORD
// 			debug_record(vlen[variable_iter], prec[variable_iter]);
// 		#endif
// 		variable_iter++;
// 		return false;
// 	}
// 	if (precision != PRECDEF) {
// 		if (precision > 9) { warn(7); precision = 9; }
// 		if (!precision || precision < 0) { warn(7);	precision = PRECDEF; }
// 		prec[variable_iter] = precision;
// 	}
// 	LOG(target);
// 	if (target < 0) { prec[variable_iter] *= -1; target = abs(target); }
	
// 	long final = ((long)(target * pow(10, precision) + 0.5));
// 	vlen[variable_iter] += strlen(ltoa(final, buffer, 10));
// 	if (!check(vlen[variable_iter], (prec[variable_iter] < 0) ? ((precision != PRECDEF) ? 4 : 2) : ((precision != PRECDEF) ? 3 : 1))) { warn(2); return false;};

// 	ECHO(abs(final));
// 	IMET.print(abs(final));

// 	#ifdef DEBUGRECORD
// 		Serial.print("\t\t\tvlen: "); Serial.print(vlen[variable_iter]); Serial.print("prec: "); Serial.println(prec[variable_iter]);
// 	#endif
// 	variable_iter++;
// 	return true;
// }
// // 1 letter = 2 hex bits. twice as long - is it truly dynamic enough to transmit?
// char* to_record(char *str) {
// 	LOG(str);
// 	vlen[variable_iter] = strlen(str) * 2;
// 	prec[variable_iter] = CHARIND;
// 	if (!check(vlen[variable_iter], 2)) { warn(2); return str;} ;
// 	for (int i = 0; i < strlen(str); i++) {
// 		ECHO((str[i], HEX));
// 		IMET.print(str[i], HEX);
// 	}
// 	#ifdef DEBUGRECORD
// 		Serial.print("\t\t\tvlen: "); Serial.print(vlen[variable_iter]); Serial.print("\t\t\tprec: "); Serial.println(prec[variable_iter]);
// 	#endif
// 	variable_iter++;
// 	return str;
// }
// // AVOID String class if possible - memory fragmentation
// char* to_record(String str) { 
// 	LOG(str);
// 	vlen[variable_iter] = str.length() * 2;
// 	prec[variable_iter] = CHARIND;
// 	str.toCharArray(buffer, 200);
// 	if (!check(vlen[variable_iter], 2)) { warn(2); return buffer;};
// 	ECHO(buffer);
// 	#ifdef DEBUGRECORD
// 		Serial.print("\t\t\tvlen: "); Serial.print(vlen[variable_iter]); Serial.print("\t\t\tprec: "); Serial.println(prec[variable_iter]);
// 	#endif
// 	IMET.print(buffer);
// 	variable_iter++;
// 	return buffer;
// }

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////// OLD: FOR REFERENCE ONLY ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


/// transmit int & floatlikes to iMet encoded in HEX
/// to transmit floatlike, MUST specify precision
template <typename T>		// int-like
void to_record(T reading) {
	if (reading == ERRVAL) {	// sensor fail
		LOG("FAIL");
		vlen[variable_iter] = ERRVAL;
		IMET_TX(0);
		length_iter += 1;
		variable_iter++;
		return;
	}
	LOG(reading);
	vlen[variable_iter] = IMET_TXB(abs(reading), HEX);
	vlen[variable_iter] * (reading < 0 ? -1 : 1);
	length_iter += vlen[variable_iter];
	variable_iter++;
}

void to_record(double reading, int precision = PRECDEF) {
	if (reading == ERRVAL) {	// sensor fail
		LOG("FAIL");
		vlen[variable_iter] = ERRVAL;
		IMET_TX(0);
		length_iter += 1;
		variable_iter++;
		return;
	}
	LOG(reading);
	vlen[variable_iter] = IMET_TXB((long)(abs(reading) * pow(10, precision)), HEX)
	vlen[variable_iter] * (reading < 0 ? -1 : 1);	
	length_iter += vlen[variable_iter];
	variable_iter++;
}
	
void sensor_check() {
		if (!lps_int_started) fail(0, L_p_int, L_t_int);
		// if (!lps_ext_started) fail(1, L_p_ext, L_t_ext);
		if (!lsm_started) fail(2, mag_x, mag_y, mag_z);
		if (!bme_started) fail(3, B_t_ext, B_p_ext, B_h_ext, B_g_ext, B_a_ext);
		// if (isnan(D_h_ext) || isnan(D_t_ext)) fail(4, D_h_ext, D_t_ext);
		// if (isnan(D_h_int) || isnan(D_t_int)) fail(5, D_h_int, D_t_int);
		if (!sd_started) warn(6);
}

void status_length_segment() {
	if (warns[1]) {
		for (int err = 1; err < EREGLEN; err++) {
			if (warns[err]) {
				status += err * (int)(pow(10, cyc_errs) + 0.5);	// e.g. 543 
				cyc_errs++;
			}
		}
		// code 6..1 shall immediately precede sfail if present
		if (warns[5]) { // SENSOR DISC/ERROR
			for (int sens = 0; sens < SREGLEN; sens++) {
				if (fails[sens]) {
					sfail += sens * (long)(pow(10, cyc_sfail) + 0.5);
					cyc_sfail++;
				}
			}
		}
		IMET_TX(status);

		LOG(status);
		if (cyc_sfail) { ECHO(sfail); LOG(sfail); } // avoid false pasitive 0

		if (warns[5]) for (int i = 0; i < SREGLEN; i++) fails[i] = false;
		for (int i = 0; i < EREGLEN; i++) warns[i] = false;
		status = NOEXCEPT;
		sfail = cyc_errs = cyc_sfail = 0;
	}
	else {
		length_iter += IMET_TX(NOEXCEPT);
	}
	
	ECHO(" ");
	length_iter +=	IMET_TX(SEPERATOR);
	length_iter +=	IMET_TX(SEPERATOR);
	ECHO(" ");
	for (int var = 0; var < NUMVARS; var++) {
		if (!vlen[var]) break;
		if (vlen[var] == ERRVAL) { length_iter += IMET_TX(ERRSIG); continue; }
		if (vlen[var] < 0) { length_iter += IMET_TX(NEGSIG); vlen[var] = abs(vlen[var]); }
		length_iter += IMET_TX(vlen[var]);
	}

	for (int i = 0; i < NUMVARS; i++) { prec[i] = 2; vlen[i] = 0;	}
	if (length_iter % 2)	IMET_TX(SEPERATOR); // even it out.
	IMET_TX("\r\n"); // terminates the packet
	
	#ifdef ECHO_DEBUG
		ECHO("\niteration packet length: "); ECHO(length_iter); ECHOLN(length_iter % 2 ? "   +1 bit of padding added" : "");
	#endif
}

void open_packet(char packet_ident) {
	variable_iter = 0, length_iter = 0;
	// begin packet
	length_iter += IMET_TX(PACKETHEADER);
	length_iter += IMET_TX(packet_ident);
}

void close_packet(char packet_ident) {
	ECHO(" ");
	length_iter += IMET_TX(SEPERATOR);
	length_iter += IMET_TX(SEPERATOR);
	ECHO(" ");
	status_length_segment();
	
	LOG_APPEND('\n');
}

#ifdef EXTEND_CYCLE
int new_cycle_hold = 0;
#endif
#ifdef TIMINGINSPECTOR
int last_tick = 0;
#endif

bool stage_done[3] = {0};

#define ADCPAIRS 6
long samples[ADCPAIRS] = {0};

bool sample() {
	sample_clock = millis();
	ECHO("S ");
	
	ECHO(GPS.altitude); ECHO(" m ");
	if(rapid_sample) ECHO(" RAPID ");

	double calibration = 0.009;
	// for debugging only. not applied to acutal data.

	adc.ltc2499ChangeChannel(LTC2499_CHAN_DIFF_0P_1N);
	RADIO.print(GPS.altitude); RADIO.print(",");
	for (int i = 0; i < ADCPAIRS; i++) {
		byte newChannel;
		
		switch(i) {
			case 0:
				newChannel = LTC2499_CHAN_DIFF_12P_13N;
				break;
			case 1:
				newChannel = LTC2499_CHAN_DIFF_4P_5N;
				break;
			case 2:
				newChannel = LTC2499_CHAN_DIFF_6P_7N;
				break;
			case 3:
				newChannel = LTC2499_CHAN_DIFF_8P_9N;
				break;
			case 4:
				newChannel = LTC2499_CHAN_DIFF_10P_11N;
				break;
			case 5:
			default:
				newChannel = LTC2499_CHAN_DIFF_0P_1N;
				break;
		}

		ECHO("[");
		ECHO(i * 2);
		ECHO("+");
		ECHO(i * 2 + 1);
		ECHO("-] ");
		samples[i] = adc.ltc2499ReadAndChangeChannel(newChannel);
		ECHOB((double)(((samples[i] * 2.048) / 16777215ul)) + calibration,4);
		// not base, just decimal places
		RADIO.print(samples[i] < 0 ? "-" : "");
		RADIO.print(abs(samples[i]),HEX); RADIO.print(",");
		ECHO(" V ");
	}
	RADIO.println();
	ECHOLN();
}

// final sample - reset for next measurement
bool last_rapid_sample() {
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
	ECHO("CYC FREQ SET TO "); ECHOLN(cycle_freq);
}

int upd_cyc_freq(int alt = 0) {
	// replace "false" with condition for bad gps alt confidence
	// e.g. vdop > 200 or !GPS.fix
	// if (false && bme_started) alt = bme.readAltitude(alt > STD_MSL ? STD_HPA : SEL_HPA);
	return LEADING_COEF*((E_0/(REF_COND*exp(alt/(alt < H_BOUND ? H_LOWER : H_UPPER)))));
}

void cycle_stage_0() {		
	sampling_priority = stage_done[0] = true;
	MARKER("MEASUREMENT [0s]");
	ECHOLN("\n[0s] MEASUREMENT CYCLE\n");
	digitalWrite(MULTIPLEXER_EN, HIGH);
	digitalWrite(CAL, HIGH);	// open relays to GND & multiplexer
	digitalWrite(EN, HIGH); 
}

void cycle_stage_2(bool &probe1_posit) {
	stage_done[1] = true;
	MARKER("MEASUREMENT [2s]");
	ECHOLN("\n[2s] MEASUREMENT CYCLE\n");
	digitalWrite(A0, HIGH);	// lines to A0, A1 interpreted
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
	digitalWrite(A0, LOW);
	digitalWrite(A1, LOW);
	delay(RELAY_DELAY); 			// min. 4ms required for relays to physically close
	digitalWrite(EN, LOW);		// close multiplexer
	// immediately sample 30Hz for 10s, then reset
	digitalWrite(MULTIPLEXER_EN, LOW);
}


// =====================================================

void setup() {
	for (int i = 0; i < OUT_PINS; i++) pinMode(output_pins[i], OUTPUT);	
	digitalWrite(LED1, HIGH);
	digitalWrite(LED2, HIGH);


	Serial.begin(115200);
	IMET.begin(9600);
	RADIO.begin(9600);
	GPS.begin(9600);
	// while(!Serial) delay(10);
	Wire.begin();
	dht_int.begin();
	dht_ext.begin();
	adc.begin(ARD2499_ADC_ADDR_ZZZ, ARD2499_EEP_ADDR_ZZ);
	digitalWrite(LED1, LOW);
	adc.ltc2499ChangeConfiguration(LTC2499_CONFIG2_60_50HZ_REJ);

	for (int i = 0; i < NUMVARS; i++) { prec[i] = 2; vlen[i] = 0;	}

	ECHOLN("\n[START]");
	ECHO("Sync w/ ground station...");
	ECHOLN(RADIO.println("SYNC") ? "DONE" : "FAIL");
	ECHO("Initializing sensors... ");
	if (lsm.begin()) lsm_started = true;
	if (lps_int.begin_I2C(sensors[3])) lps_int_started = true;
	// if (lps_ext.begin_I2C(sensors[4])) lps_ext_started = true;
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
	  filename[3] = i/10 + '0';
	  filename[4] = i%10 + '0';
	  if (!SD.exists(filename)) {
		  logger = SD.open(filename, FILE_WRITE);
			LOG_APPEND("A,TIME,GPS ALT,V1G,V2G,V12,CALV,IMON,TEMP,mag_x,mag_y,mag_z,status code,sens_fail\n");
			LOG_APPEND("A RAPID,TIME,GPS ALT,V1G,V2G,V12,CALV,IMON,TEMP,mag_x,mag_y,mag_z,status code,sens_fail\n");
			LOG_APPEND("B,TIME,GPS ALT,GPS FIX,T(ALT),L_t_int,L_p_int,B_t_ext,B_p_ext,B_h_ext,B_g_ext,B_a_ext,status code,sens_fail\n");
		  break;
	  }
  }
	// if (!logger) sd_started = false;
	ECHOLN(!logger ? "FAIL" : "DONE");
	ECHO("Initializing GPS... ");
	#ifndef SKIPGPSINIT
	delay(4000);
	#endif
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
	GPS.sendCommand(PGCMD_ANTENNA);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5SEC);
	delay(1000);
	ECHOLN("DONE");

	write_clock = sensor_clock = measure_clock = 
	sample_clock = adcmag_clock = sample_rapid_timer = 0;

	
	digitalWrite(LED1, LOW);
	digitalWrite(LED2, LOW);
	digitalWrite(A0, LOW);
	digitalWrite(A1, LOW);
	digitalWrite(CAL, LOW);
	digitalWrite(EN, LOW);
}

int GPS_read = 0;
// =================================================

void loop() {
	#ifdef TIMINGINSPECTOR
	if (seconds() > last_tick) { 
		Serial.print("\n["); Serial.print(seconds()); Serial.print("]  T: "); Serial.print(cycle_freq); Serial.print("\t");
		last_tick = seconds();
	}
	#endif

	GPS_read = 0;
	// check gps line every single tick
	while (GPS.available() && !sampling_priority && GPS_read < 20) {
		gpsbuf = GPS.read();
		GPS_read++;
		#ifdef ECHOGPS 
			Serial.write(gpsbuf);
		#endif
	}

	if (GPS.newNMEAreceived()) {
		if (!GPS.parse(GPS.lastNMEA())) {
			// return;		//just wait for complete NMEA
		}
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
	/// SAMPLE: PACKET A:  MAGNETOMETER, ADC
	/// if rapid_sample & RAPID_SAMPLE_MS elapsed - return to regular sample cycle	- 	release priority
	if (millis() - sample_clock > 1000/sample_rate) {
		if (!rapid_sample && millis() - sensor_clock < MARGIN) return;
		sample_clock = millis();
		if (rapid_sample && millis() - sample_rapid_timer > RAPID_SAMPLE_MS) last_rapid_sample();
		else sample();
		MARKER("A: SAMPLE ADCS, MAGNETOMETER");
		
		// if (!check(sample data size, sample data length)) IMET.print("\r\n");
		
		lsm.getEvent(&magn);
		mag_x = magn.magnetic.x;
		mag_y = magn.magnetic.y;
		mag_z = magn.magnetic.z;
		
		sensor_check(); // check sensors for status, zero out fails

		#ifdef ECHO_DEBUG
			Serial.print("EMF	 x: "); Serial.print(mag_x); Serial.print("uT  ");
			Serial.print("y: "); Serial.print(mag_y); Serial.print("uT  ");
			Serial.print("z: "); Serial.print(mag_z); Serial.println("uT  ");
		#endif
		LOG(rapid_sample ? "A RAPID" : "A");
		

		// LOG_APPEND((GPS.hour + GPS_UTC_ADJ) < 0 ? 24 + GPS.hour + GPS_UTC_ADJ : GPS.hour + GPS_UTC_ADJ); LOG_APPEND(':'); LOG_APPEND(GPS.minute < 10 ? "0" : ""); LOG_APPEND(GPS.minute); LOG_APPEND(':'); LOG_APPEND((GPS.seconds + (int)GPS.secondsSinceTime()) < 10 ? "0" : ""); LOG(GPS.seconds + (int)GPS.secondsSinceTime());
		LOG_APPEND((GPS.hour + GPS_UTC_ADJ) < 0 ? 24 + GPS.hour + GPS_UTC_ADJ : GPS.hour + GPS_UTC_ADJ); LOG_APPEND(':'); LOG_APPEND(GPS.minute < 10 ? "0" : ""); LOG_APPEND(GPS.minute); LOG_APPEND(':'); LOG_APPEND((GPS.seconds) < 10 ? "0" : ""); LOG(GPS.seconds);

		LOG(GPS.altitude);

		// MODE,TIME,GPS ALT,V1G,V2G,V12,CALV,IMON,TEMP,mag_x,mag_y,mag_z,status code,sens_fail
		open_packet('A');
		for (int i = 0; i < ADCPAIRS; i++) to_record(samples[i]);
		to_record(mag_x, 2);
		to_record(mag_y, 2);
		to_record(mag_z, 2);
		close_packet('A');
		
	}

	///////////////////////////////////////////////////////
	/// SENSORS: PACKET B:
	if (!sampling_priority && millis() - sensor_clock > SENSOR_INTERVAL) {
		if (millis() - sample_clock < MARGIN) return;
		sensor_clock = millis();
		MARKER("POLL SENSORS, UPDATE PROBE FREQ(GPS)");
		
		Serial.print("GPS	 ALT: "); Serial.println(GPS.altitude);
		// Serial.print("GPS TIME: "); Serial.print((GPS.hour + GPS_UTC_ADJ) < 0 ? 24 + GPS.hour + GPS_UTC_ADJ : GPS.hour + GPS_UTC_ADJ); Serial.print(':'); Serial.print(GPS.minute < 10 ? "0" : ""); Serial.print(GPS.minute); Serial.print(':'); Serial.print((GPS.seconds + (int)GPS.secondsSinceTime()) < 10 ? "0" : ""); Serial.println(GPS.seconds + (int)GPS.secondsSinceTime());
		Serial.print("GPS TIME: "); Serial.print((GPS.hour + GPS_UTC_ADJ) < 0 ? 24 + GPS.hour + GPS_UTC_ADJ : GPS.hour + GPS_UTC_ADJ); Serial.print(':'); Serial.print(GPS.minute < 10 ? "0" : ""); Serial.print(GPS.minute); Serial.print(':'); Serial.print((GPS.seconds) < 10 ? "0" : ""); Serial.println(GPS.seconds);
		Serial.print("GPS  FIX: "); Serial.print(GPS.fix ? "TRUE, " : "FALSE, "); Serial.print(GPS.satellites); Serial.println(" sats");
		Serial.print("VDOP act: +-"); Serial.println(vdop);

		Serial.print("T(alt): "); Serial.print(cycle_freq); Serial.println(" s");
		Serial.println();

		#ifndef EXTEND_CYCLE
			// cycle_freq = upd_cyc_freq(GPS.altitude / 1000);
			cycle_freq = upd_cyc_freq(22);
			cycle_freq = cycle_freq < MIN_TIME ? MIN_TIME : cycle_freq;
		#else
			// new_cycle_hold = upd_cyc_freq(GPS.altitude / 1000);
			new_cycle_hold = upd_cyc_freq(22);
			new_cycle_hold = (new_cycle_hold < MIN_TIME) ? MIN_TIME : new_cycle_hold;
		#endif

		vdop = GPS.HDOP * 1.38 * 23;         // +- m
		// inferred from HDOP, * 23m - GPGGA doesn't include VDOP
		// The_Distributions_of_HDOP_and_VDOP_in_GNSS_and_a_Corresponding_New_Algorithm_of_Fast_Selecting_Satellites
		// 10.1007/978-3-642-29175-3_37
		if (vdop > 100) warn(4);
		if (!GPS.fix) warn(3);

		LOG('B');
		// LOG_APPEND((GPS.hour + GPS_UTC_ADJ) < 0 ? 24 + GPS.hour + GPS_UTC_ADJ : GPS.hour + GPS_UTC_ADJ); LOG_APPEND(':'); LOG_APPEND(GPS.minute < 10 ? "0" : ""); LOG_APPEND(GPS.minute); LOG_APPEND(':'); LOG_APPEND((GPS.seconds + (int)GPS.secondsSinceTime()) < 10 ? "0" : ""); LOG(GPS.seconds + (int)GPS.secondsSinceTime());
		LOG_APPEND((GPS.hour + GPS_UTC_ADJ) < 0 ? 24 + GPS.hour + GPS_UTC_ADJ : GPS.hour + GPS_UTC_ADJ); LOG_APPEND(':'); LOG_APPEND(GPS.minute < 10 ? "0" : ""); LOG_APPEND(GPS.minute); LOG_APPEND(':'); LOG_APPEND((GPS.seconds) < 10 ? "0" : ""); LOG(GPS.seconds);
		LOG(GPS.altitude);
		LOG(GPS.fix ? GPS.satellites : 0);
		LOG(cycle_freq);

		//  reading from sensors. any data type
		unsigned long endMeas = bme.beginReading();
		// D_h_ext = dht_ext.readHumidity();
		// D_h_int = dht_int.readHumidity();
		// D_t_ext = dht_ext.readTemperature();
		// D_t_int = dht_int.readTemperature();
		
		lps_int.getEvent(&p_int, &t_int);
		// lps_ext.getEvent(&p_ext, &t_ext);
		L_p_int = p_int.pressure;
		L_t_int = t_int.temperature;
		// L_p_ext = p_ext.pressure;
		// L_t_ext = t_ext.temperature;

		if ((endMeas == 0) || (!bme.endReading())) fail(3, B_t_ext, B_p_ext, B_h_ext, B_g_ext, B_a_ext);
		else {
			B_t_ext = bme.temperature;
			B_p_ext = bme.pressure / 100.0;
			B_h_ext = bme.humidity;
			B_g_ext = bme.gas_resistance / 1000.0;
			B_a_ext = bme.readAltitude(GPS.altitude > STD_MSL ? STD_HPA : SEL_HPA);
		}

		sensor_check();

		#ifdef ECHO_DEBUG
		// Serial.print("\n\n["); Serial.print((GPS.hour + GPS_UTC_ADJ) < 0 ? 24 + GPS.hour + GPS_UTC_ADJ : GPS.hour + GPS_UTC_ADJ); Serial.print(':'); Serial.print(GPS.minute < 10 ? "0" : ""); Serial.print(GPS.minute); Serial.print(':'); Serial.print((GPS.seconds + (int)GPS.secondsSinceTime()) < 10 ? "0" : ""); Serial.print(GPS.seconds + (int)GPS.secondsSinceTime()); Serial.println("]");
		Serial.print("\n\n["); Serial.print((GPS.hour + GPS_UTC_ADJ) < 0 ? 24 + GPS.hour + GPS_UTC_ADJ : GPS.hour + GPS_UTC_ADJ); Serial.print(':'); Serial.print(GPS.minute < 10 ? "0" : ""); Serial.print(GPS.minute); Serial.print(':'); Serial.print((GPS.seconds) < 10 ? "0" : ""); Serial.print(GPS.seconds); Serial.println("]");
		Serial.println("\t temp\tpres\thum");
		// Serial.print("DHT_ext  "); 		Serial.print(D_t_ext); Serial.print("\t\t"); 	Serial.println(D_h_ext);
		// Serial.print("DHT_int  "); 		Serial.print(D_t_int); Serial.print("\t\t"); 	Serial.println(D_h_int);
		// Serial.print("LPS_ext  "); 		Serial.print(L_t_ext); Serial.print("\t"); 		Serial.println(L_p_ext);
		Serial.print("LPS_int  "); 		Serial.print(L_t_int); Serial.print("\t"); 		Serial.println(L_p_int);
		Serial.print("BME_ext  "); 		Serial.print(B_t_ext); Serial.print("\t"); 		Serial.print(B_p_ext); Serial.print("\t"); Serial.println(B_h_ext);
		Serial.print("\tgas res. "); 	Serial.print(B_g_ext);         // KOhms
		Serial.print("\taltim  "); 		Serial.println(B_a_ext);
		Serial.println();
		#endif

		open_packet('B');

		// send to iMet, datalogger
		// to_record(D_t_ext, 2);
		// to_record(D_h_ext, 2);
		// to_record(D_t_int, 2);
		// to_record(D_h_int, 2);
		// to_record(L_t_ext, 2);
		// to_record(L_p_ext, 2);
		to_record(L_t_int, 2);
		to_record(L_p_int, 2);
		to_record(B_t_ext, 2);
		to_record(B_p_ext, 2);
		to_record(B_h_ext, 2);
		to_record(B_g_ext, 2);
		to_record(B_a_ext, 2);

		close_packet('B');
	}

	//////////////////////////////////////////////////////
	/// FLUSH: data from cache -> card
	/// moderate; I/O, power & time - BLOCKING!
	if (!sampling_priority && (millis() - write_clock) > FLUSH_INTERVAL) {
		safe_poweroff(false);
		write_clock = millis(); 
		MARKER("DATALOGGER FLUSH");
		logger.flush();
		ECHOLN("DATALOGGER FLUSH");
		// watch for SPI LED
		if (logger.getWriteError()) warn(6);
		safe_poweroff(true);
	}

	/// indicate optimal time to remove SD, avoid middle of write
	if (safe_eject && (millis() - safe_eject) > FLUSH_INTERVAL / 2.5) {
		indicate_safe(0);
		safe_eject = 0;
	}
	
}
