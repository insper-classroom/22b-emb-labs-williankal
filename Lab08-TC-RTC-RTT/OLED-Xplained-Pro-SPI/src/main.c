#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"

#include "gfx_mono_text.h"

#include "sysfont.h"

// LED placa (aleat?rio)
#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_IDX 8
#define LED_IDX_MASK (1 << LED_IDX)

// LED 1 (OLED)
#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX 0
#define LED1_IDX_MASK (1 << LED1_PIO_IDX)

// LED 2 (OLED)
#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX 30
#define LED2_IDX_MASK (1 << LED2_PIO_IDX)

// LED 3 (OLED)
#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO_IDX 2
#define LED3_IDX_MASK (1 << LED3_PIO_IDX)

// Bot�o
#define BUT_PIO PIOA
#define BUT_PIO_ID ID_PIOA
#define BUT_IDX 11
#define BUT_IDX_MASK (1 << BUT_IDX)

// BOT�O 1
#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX 28
#define BUT1_IDX_MASK (1u << BUT1_PIO_IDX) // esse j� est� pronto.

// BOT�O 2
#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_IDX_MASK (1u << BUT2_PIO_IDX) // esse j� est� pronto.

// BOT�O 3
#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_IDX_MASK (1u << BUT3_PIO_IDX) // esse j� est� pronto.

volatile flag = 1;

typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
void leitor(void);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void pin_toggle(Pio *pio, uint32_t mask);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName) {
    printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
    for (;;) {
    }
}

/* This function is called by FreeRTOS idle task */
extern void vApplicationIdleHook(void) {
    pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/* This function is called by FreeRTOS each tick */
extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    /* Force an assert. */
    configASSERT((volatile void *)NULL);
}

void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
	* Isso � realizado pela leitura do status do perif�rico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED1_PIO, LED1_IDX_MASK);  
}

void TC2_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
	* Isso � realizado pela leitura do status do perif�rico
	**/
	volatile uint32_t status = tc_get_status(TC0, 2);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED_PIO, LED_IDX_MASK);  
}

void RTT_Handler(void) {
  uint32_t ul_status;
  ul_status = rtt_get_status(RTT);

  /* IRQ due to Alarm */
  if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
      pin_toggle(LED2_PIO, LED2_IDX_MASK);
	  RTT_init(4, 16, RTT_MR_ALMIEN);
   }  
}

void RTC_Handler(void) {
    uint32_t ul_status = rtc_get_status(RTC);
	
    /* seccond tick */
    if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {	
	// o c�digo para irq de segundo vem aqui
		leitor();
    }
	
    /* Time or date alarm */
    if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
    	pin_toggle(LED3_PIO, LED3_IDX_MASK);
    }

    rtc_clear_status(RTC, RTC_SCCR_SECCLR);
    rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
    rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
    rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
    rtc_clear_status(RTC, RTC_SCCR_CALCLR);
    rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void pin_toggle(Pio *pio, uint32_t mask) {
	pio_clear(pio, mask);		// Coloca a sa�da em 0
	delay_ms(100);				// Aguarda 100ms
	pio_set(pio, mask);			// Coloca a sa�da em 1
	delay_ms(100);				// Aguarda 100ms
}

void freq(void) {
	/* Leitura do valor atual do RTC */           
    uint32_t current_hour, current_min, current_sec;
    uint32_t current_year, current_month, current_day, current_week;
    rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
    rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
	
    /* configura alarme do RTC para daqui 20 segundos */                                                                   
    rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);                              
    rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 20);
}

// Inicializa botao SW0 do kit com interrupcao
void io_init(void) {
    // Leds
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_configure(LED_PIO, PIO_OUTPUT_1, LED_IDX_MASK, PIO_DEFAULT);
    pmc_enable_periph_clk(LED1_PIO_ID);
    pio_configure(LED1_PIO, PIO_OUTPUT_1, LED1_IDX_MASK, PIO_DEFAULT);
    pmc_enable_periph_clk(LED2_PIO_ID);
    pio_configure(LED2_PIO, PIO_OUTPUT_1, LED2_IDX_MASK, PIO_DEFAULT);
    pmc_enable_periph_clk(LED3_PIO_ID);
    pio_configure(LED3_PIO, PIO_OUTPUT_1, LED3_IDX_MASK, PIO_DEFAULT);

	 // Inicializa clock do perif�rico PIO responsavel pelo botao
    pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);

    // Configura PIO para lidar com o pino do bot�o como entrada com pull-up
    pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUT1_PIO, BUT1_IDX_MASK, 60);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUT2_PIO, BUT2_IDX_MASK, 60);
    pio_configure(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUT3_PIO, BUT3_IDX_MASK, 60);
    pio_handler_set(BUT1_PIO,
                    BUT1_PIO_ID,
                    BUT1_IDX_MASK,
                    PIO_IT_EDGE,
                    freq);
    // Ativa interrup��o e limpa primeira IRQ gerada na ativacao
    pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);
    pio_get_interrupt_status(BUT1_PIO);	
    NVIC_EnableIRQ(BUT1_PIO_ID);
    NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 0
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrup�c�o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
  	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

  uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  
  if (rttIRQSource & RTT_MR_ALMIEN) {
	uint32_t ul_previous_time;
  	ul_previous_time = rtt_read_timer_value(RTT);
  	while (ul_previous_time == rtt_read_timer_value(RTT));
  	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
  }

  /* config NVIC */
  NVIC_DisableIRQ(RTT_IRQn);
  NVIC_ClearPendingIRQ(RTT_IRQn);
  NVIC_SetPriority(RTT_IRQn, 4);
  NVIC_EnableIRQ(RTT_IRQn);

  /* Enable RTT interrupt */
  if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
  else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
		  
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

void leitor(void) {	
	gfx_mono_draw_rect(0, 0, 128, 32, GFX_PIXEL_CLR);

	/* Leitura do valor atual do RTC */           
	uint32_t current_hour, current_min, current_sec;
	rtc_get_time(RTC, &current_hour, &current_min, &current_sec);

	/* Imprime na tela o valor atual do RTC */
	char str[20];
	sprintf(str, "%02d:%02d:%02d", current_hour, current_min, current_sec);
	gfx_mono_draw_string(str, 0, 0, &sysfont);
}

int main (void) {
	board_init();
	sysclk_init();
	delay_init();

	WDT->WDT_MR = WDT_MR_WDDIS;

	io_init();

	gfx_mono_ssd1306_init();

	/** Configura RTC */                                                                        
    calendar rtc_initial = {2022, 3, 19, 12, 15, 45 ,1};                                            
    RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);

	TC_init(TC0, ID_TC1, 1, 2);
	tc_start(TC0, 1);

	TC_init(TC0, ID_TC2, 2, 5);
	tc_start(TC0, 2);

	RTT_init(4, 16, RTT_MR_ALMIEN);


	while (1) {		
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
