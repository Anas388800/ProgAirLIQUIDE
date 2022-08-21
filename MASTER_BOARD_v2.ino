 /*
 * Autor : SALAMANI Anas 
 * Last Version : 09/07/2021
 */
#include <Wire.h>
#define SAMPLES 512             //Bins of data must be a power of 2
#define SAMPLING_FREQUENCY 10032 //Hz
#define BUF_SIZE 0x400
#define BUF_SIZE_MERGED 0xC00

uint16_t force[BUF_SIZE];
uint16_t master[BUF_SIZE];
uint16_t slave[BUF_SIZE];
uint16_t buf_v_m_s[BUF_SIZE_MERGED];

const int analogInPin0 = A0;        // Analog pin 
const int analogInPin1 = A1;        // Analog pin 
const int analogInPin2 = A3;        // Analog pin

unsigned long sampling_period_us;
unsigned long useconds_sampling;


int    state = false;              //
int    magnitude = 0;              //
int    frequency = 57;             // Departure Frequency 
String user_freq;                  // Frequency sent by User throught RaspberryPi
String user_mgn ;                   //
int    number_of_ticks = 0;        // Auto Reload Register
int    j;
int  val;           /* generic value read from serial */
byte x ;
int start = 0;

int max_voltage_accepted = 5;
float gain_aop = 11.81;
int gain_bop = 4;

int MAX_dac = 6;
 

String mot;
String value;
