void dac_setup ()
{

  //PMC->PMC_PCER1 = PMC_PCER1_PID38;                  // DACC power ON
  pmc_enable_periph_clk (DACC_INTERFACE_ID) ;          // start clocking 
  DACC->DACC_IDR = 0xFFFFFFFF ;                        // no interrupts
}
