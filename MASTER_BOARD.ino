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


#define    n_sample               176
uint32_t  Sinewave[n_sample] = {
     
0x0800,0x0849,0x0892,0x08dc,0x0925,0x096d,0x09b5,0x09fd,0x0a44,0x0a8a,0x0acf,0x0b13,0x0b57,0x0b99,0x0bda,0x0c1a,0x0c58,0x0c95,0x0cd1,0x0d0a, 
0x0d43,0x0d79,0x0dae,0x0de1,0x0e11,0x0e40,0x0e6d,0x0e98,0x0ec0,0x0ee7,0x0f0b,0x0f2c,0x0f4b,0x0f68,0x0f83,0x0f9b,0x0fb0,0x0fc3,0x0fd4,0x0fe1,
0x0fec,0x0ff5,0x0ffb,0x0ffe,0x0fff,0x0ffd,0x0ff8,0x0ff1,0x0fe7,0x0fdb,0x0fcc,0x0fba,0x0fa6,0x0f8f,0x0f76,0x0f5a,0x0f3c,0x0f1c,0x0ef9,0x0ed4,
0x0eac,0x0e83,0x0e57,0x0e29,0x0df9,0x0dc7,0x0d94,0x0d5e,0x0d27,0x0cee,0x0cb3,0x0c77,0x0c39,0x0bfa,0x0bba,0x0b78,0x0b35,0x0af1,0x0aac,0x0a67,
0x0a20,0x09d9,0x0991,0x0949,0x0900,0x08b7,0x086e,0x0824,0x07db,0x0791,0x0748,0x06ff,0x06b6,0x066e,0x0626,0x05df,0x0598,0x0553,0x050e,0x04ca,
0x0487,0x0445,0x0405,0x03c6,0x0388,0x034c,0x0311,0x02d8,0x02a1,0x026b,0x0238,0x0206,0x01d6,0x01a8,0x017c,0x0153,0x012b,0x0106,0x00e3,0x00c3,
0x00a5,0x0089,0x0070,0x0059,0x0045,0x0033,0x0024,0x0018,0x000e,0x0007,0x0002,0x0000,0x0001,0x0004,0x000a,0x0013,0x001e,0x002b,0x003c,0x004f,
0x0064,0x007c,0x0097,0x00b4,0x00d3,0x00f4,0x0118,0x013f,0x0167,0x0192,0x01bf,0x01ee,0x021e,0x0251,0x0286,0x02bc,0x02f5,0x032e,0x036a,0x03a7,
0x03e5,0x0425,0x0466,0x04a8,0x04ec,0x0530,0x0575,0x05bb,0x0602,0x064a,0x0692,0x06da,0x0723,0x076d,0x07b6,0x0800
};

void setup(){
  Wire.begin();                                                 // Inializise i2c peripheral
  Serial.begin(250000);                                         // Inializise Serial peripheral
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY)); // period between two samples 100us
  dac_setup();                                                  // Configuration of the Dac
  number_of_ticks = freqToTc(frequency);                        // convert frequence value to value of counter 
  Init_timer();                                                 // Configuration of the timer 

  setup_pio_TIOA0();                     // Configuration of External trigger synchronized on the counter overflow
                                         // to provides a 50-50 PWM signal
  
}

void setup_pio_TIOA0()  // configuration of external signal to create a PWM 
{
  PIOB->PIO_PDR   = PIO_PB25B_TIOA0; // disable PIO control
  PIOB->PIO_IDR   = PIO_PB25B_TIOA0; // disable PIO interrupts
  PIOB->PIO_ABSR |= PIO_PB25B_TIOA0; // switch to B peripheral consider as I/O
}

void loop(){

  while (Serial.available()) {
    /*
     * La carte recois une commande de l'interface homme-machine 
     */
    delay(2);  //delay to allow byte to arrive in input buffer
    char c = Serial.read();
    mot += c;
  }

  if (mot.length() >0) {
    /*
     * Code recu :
     * ----------
     * 
     *  Code 0 : Desactive l'Algorithme de vibration
     *  Code 1 : Active l'Algorithme de vibration
     *  Code 2 : Changement d'amplitude du Master
     *  Code 3 : Changement de frequence du Master
     *  Code 4 : Recuperation de Vibration/ Master / Slave
     *  Code 5 : Modifie gain de l'Algorithme
     */

  
    if(mot[0]=='0'){
      byte x = 0;                           // 0 : desactivate the algo
      Wire.beginTransmission(4);            // transmit to device #4
      Wire.write(x);                        // sends one byte  
      Wire.endTransmission();               // stop transmitting
    }
    
    if(mot[0]=='1'){
      byte x = 1;
      Wire.beginTransmission(4); // transmit to device #4
      Wire.write(x);             // sends one byte  
      Wire.endTransmission();    // stop transmitting
    }
    
    if(mot[0]=='2'){
      start = 1;
      value = mot.substring(1,mot.length());
      magnitude = value.toInt();
    }
    
    if(mot[0]=='3'){
      value = mot.substring(1,mot.length());
      frequency = value.toInt();
      number_of_ticks = freqToTc(frequency);
      Init_timer();        
    }

    if(mot[0]=='4'){
      for(int i=0; i<BUF_SIZE; i++){
        useconds_sampling = micros();
        force[i] = analogRead(analogInPin0);
        master[i] = analogRead(analogInPin1);
        slave[i] = analogRead(analogInPin2);
        while(micros() < (useconds_sampling + sampling_period_us)){}  //wait...       
      }
      memcpy(buf_v_m_s,                force, BUF_SIZE*sizeof(uint16_t));
      memcpy(buf_v_m_s + BUF_SIZE,     master, BUF_SIZE*sizeof(uint16_t));
      memcpy(buf_v_m_s + (BUF_SIZE*2) ,slave, BUF_SIZE*sizeof(uint16_t));
      Serial.write((uint8_t*)buf_v_m_s,BUF_SIZE_MERGED*2);
    }

    if(mot[0]=='5'){
      value = mot.substring(1,mot.length());
      byte x = value.toInt();
      Wire.beginTransmission(4); // transmit to device #4
      Wire.write(x);             // sends one byte  
      Wire.endTransmission();    // stop transmitting
    }
    
    mot="";   // vide la variable stockant le code 
  } 
}

void Init_timer(){
   /* turn on the timer clock in the power management controller */
  pmc_set_writeprotect(false);       // disable write protection for pmc registers
  pmc_enable_periph_clk(ID_TC7);     // enable peripheral clock TC7

  /* we want wavesel 01 with RC */
  TC_Configure(/* clock */TC2,/* channel */1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1); 
  TC_SetRC(TC2, 1, number_of_ticks);
  TC_Start(TC2, 1);

  // enable timer interrupts on the timer
  TC2->TC_CHANNEL[1].TC_IER=TC_IER_CPCS;   // IER = interrupt enable register
  TC2->TC_CHANNEL[1].TC_IDR=~TC_IER_CPCS;  // IDR = interrupt disable register

  /* Enable the interrupt in the nested vector interrupt controller */
  /* TC7_IRQn where 7 is the timer number * timer channels (3) + the channel number (=(2*3)+1) for timer3 channel2 */
  NVIC_EnableIRQ(TC7_IRQn);
}

int freqToTc(int freq_hz){
  int tc_cntr = 0;                                 // Timer counter value
  int ARR ;                                         
  if( freq_hz == 0 ) return 25;                    // in case ..
  ARR = (42000000 / (freq_hz * n_sample)) ;        // value of Auto Reload Register, for example : 57Hz return 4605  
  tc_cntr = ARR;                                   // return the value of  the timer counter 
  return tc_cntr;                                    
}

void TC7_Handler(){
  
  static uint8_t i;
  // We need to get the status to clear it and allow the interrupt to start again
  TC_GetStatus(TC2, 1);
  state = !state;
  
  if (start == 1){                                                    // When Supply is ON
    DACC_INTERFACE->DACC_CDR = ((Sinewave[i++]*magnitude)/gain_aop);  // Write a sample of the sinewave LUT 
    if (i == 176){                                                    // Once last value is reached, it start again
      i=0;
    }
  }
  else if (start == 0 ){                                              // When Supply is OFF
    DACC_INTERFACE->DACC_CDR = 0;                                     // output 0V
  }
  
  
}

/*void dac_setup(){
  pmc_enable_periph_clk (DACC_INTERFACE_ID) ; // start clocking DACC
  DACC->DACC_IDR = 0xFFFFFFFF ;               // no interrupts
  DACC->DACC_CHER = DACC_CHER_CH1 ;              // enable DACC chan0
}
*/

void dac_setup ()
{

  //PMC->PMC_PCER1 = PMC_PCER1_PID38;                  // DACC power ON
  pmc_enable_periph_clk (DACC_INTERFACE_ID) ;          // start clocking 
  DACC->DACC_IDR = 0xFFFFFFFF ;                        // no interrupts
  DACC->DACC_CR = DACC_CR_SWRST ;                      // Reset DACC
  DACC->DACC_MR |=  DACC_MR_USER_SEL_CHANNEL1;         // select channel 1
  DACC->DACC_CHER = DACC_CHER_CH1;                     // enable channel 1 = DAC1

}
