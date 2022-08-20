/****************************************************************************************************/
/*  1 MHz ADC conversions of 1 analog input (A0) triggered by Timer Counter 0 channel 2 TIOA2       */
/*  1 MHz DAC output on channel 1 (DAC1) triggered by Timer Counter 0 channel 2 TIOA2               */
/****************************************************************************************************/

#include <Wire.h>

#define LEN_VIBR 176                      // Number of sample that corresponding to one period Fe 10.032kHz F = 57Hz
int vibr[LEN_VIBR] = {};                  // Buffer that store one period of vibration
bool flag = true;                         // Flag that notify the beginning of the Algo
int * source = (int *) (vibr) +1;
int * destination = (int *) (vibr);
size_t size = LEN_VIBR * sizeof( int );  
int *p =(int*)(&vibr+1)-1;                // Last value of the buffer that store the new value of Vib

int next_samp;                            // Sample that follow to vibration buff
int sum;                                  // 
int v_master;                             // Master signal received 
int v_slave;                              // Slave signal sent to DAC
float gain = 0.1;                         // Gain of Rep Con
int algo_statue = 0;                      // Alorithm disabled by default
float x = 0;                              // Value received by I2C

int minDAC = 640;                         // 1/6 of 3.33V
int maxDAC = 3430;                        // 5/6 of 3.33V

void setup()
{
  adc_setup();
  dac_setup();
  tc_setup();
  Serial.begin(9600);
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
}

void loop(){
}


void receiveEvent(int howMany)
{ 
 /*
  * 2 pieces of information can arrive :     
  *      - the activation / deactivation of the algorithm (either x = 0 either x = 1)
  *      - the value of the applied gain of the algorithm (x > 1)
 */
  x = Wire.read();    
  
  if (x > 1){
    gain = x/100;
  }
  else{
    algo_statue = x; 
  }
  Serial.println(x);
}


void ADC_Handler () {

  
  static uint8_t i;

  if (algo_statue == 1){                // Repetitiv control enable
    if (i < LEN_VIBR){ 
     vibr[i++] = ADC->ADC_CDR[7]*gain;  // Acquisition of a period multiplied by a gain
    }

    /*
     * The DAC has a different voltage range from the ADC.
     *   The DAC provides a voltage between 1/6 and 5/6 of 3.3v
     *   We must therefore readjust the range to make it coincide with the ADC with the map() function
     *   
     *   memove(destination, source, size)
     *    - destination: allows you to specify the address of the memory block to receive the data to be copied.
     *    - source: allows you to define the address of the memory block to duplicate.
     *    - size: indicates the number of bytes to duplicate.
     */
    else {                 // Once the bufer is full             
      next_samp = ADC->ADC_CDR[7];                              // store the next vibration sample (177th)
      v_master = map(ADC->ADC_CDR[6],minDAC ,maxDAC ,0 ,4095);  // retrieve the master sample
      v_slave = vibr[0] + (next_samp*gain) + v_master;          // slave signal calculation
      memmove( destination, source, size );                     // We shift the array elements by one index to the left
      *p = next_samp * gain;                                    // The first element is deleted, the new sample is inserted at the end of the array
      DACC->DACC_CDR = v_slave;                                 // We generate the slave signal
    }
  }
  
   
  else if (algo_statue == 0){                                       // Repetitiv control disable,  
    i=0;
    DACC->DACC_CDR = map(ADC->ADC_CDR[6],minDAC ,maxDAC ,0 ,4095) ; // the slave signal copies the master signal
  }
  
}


/*************  Configuration of the analog to digital converter  *******************/
void adc_setup() {
  
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;                    // ADC power ON

  ADC->ADC_CR = ADC_CR_SWRST;                           // Reset ADC
  ADC->ADC_MR |=  ADC_MR_TRGEN_EN                       // Hardware trigger select
                  | ADC_MR_TRGSEL_ADC_TRIG3             // Trigger by TIOA2
                  | ADC_MR_PRESCAL(1);

  ADC->ADC_ACR = ADC_ACR_IBCTL(0b01);                   // For frequencies > 500 KHz
  adc_set_resolution(ADC,ADC_10_BITS);
  ADC->ADC_IER = ADC_IER_EOC7;                          // End Of Conversion interrupt enable for channel 7
  NVIC_EnableIRQ((IRQn_Type)ADC_IRQn);                  // Enable ADC interrupt
  ADC->ADC_CHER = ADC_CHER_CH7 | ADC_CHER_CH6 ;         // Enable Channels 7;6 = A0;A1
}

/*************  Configuration of the digital to analog converter  *******************/
void dac_setup ()
{

  PMC->PMC_PCER1 = PMC_PCER1_PID38;                   // DACC power ON

  DACC->DACC_CR = DACC_CR_SWRST ;                     // Reset DACC
  DACC->DACC_MR = DACC_MR_TRGEN_EN                    // Hardware trigger select
                  | DACC_MR_TRGSEL(0b011)             // Trigger by TIOA2
                  | DACC_MR_USER_SEL_CHANNEL1         // select channel 1
                  | DACC_MR_REFRESH (1)
                  | DACC_MR_STARTUP_8
                  | DACC_MR_MAXS;

  DACC->DACC_CHER = DACC_CHER_CH1;                   // enable channel 1 = DAC1

}


/*************  Timer Counter 0 Channel 2 to generate PWM pulses thru TIOA2  ************/
/*
 * The period T of the wave obtained (which will be our sampling period) is:
                                     T = RC⋅τ     with : τ = 1/42000000
                                                         RC = 4200  
                                     T = 10 000 Hz
                                     
 */
void tc_setup() {

  PMC->PMC_PCER0 |= PMC_PCER0_PID29;                       // TC2 power ON : Timer Counter 0 channel 2 IS TC2

  TC0->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1   // MCK/2 = 42MHz , clk on rising edge 
                              | TC_CMR_WAVE                // Waveform mode
                              | TC_CMR_WAVSEL_UP_RC        // UP mode with automatic trigger on RC Compare
                              | TC_CMR_ACPA_CLEAR          // Clear TIOA2 on RA compare match
                              | TC_CMR_ACPC_SET;           // Set TIOA2 on RC compare match

  TC0->TC_CHANNEL[2].TC_RC = 4200;                         // Frequency = (Mck/2)/TC_RC  Hz = 10 kHz
  TC0->TC_CHANNEL[2].TC_RA = TC0->TC_CHANNEL[2].TC_RC / 2; // Have a square timing signal

  TC0->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;// TC2 counter enable and Software trigger

}
