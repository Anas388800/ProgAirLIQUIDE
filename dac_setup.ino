void dac_setup ()
{

  //PMC->PMC_PCER1 = PMC_PCER1_PID38;                  // DACC power ON
  pmc_enable_periph_clk (DACC_INTERFACE_ID) ;          // start clocking 
  DACC->DACC_IDR = 0xFFFFFFFF ;                        // no interrupts
  DACC->DACC_CR = DACC_CR_SWRST ;                      // Reset DACC
  DACC->DACC_MR |=  DACC_MR_USER_SEL_CHANNEL1;         // select channel 1
  
}
