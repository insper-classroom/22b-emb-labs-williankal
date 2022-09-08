#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include <string.h>

// LED1
#define LED1_PIO           PIOA
#define LED1_PIO_ID        ID_PIOA
#define LED1_PIO_IDX       0
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)
// LED2
#define LED2_PIO           PIOC
#define LED2_PIO_ID        ID_PIOC
#define LED2_PIO_IDX       30
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)
// LED3
#define LED3_PIO           PIOB
#define LED3_PIO_ID        ID_PIOB
#define LED3_PIO_IDX       2
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)


// Botao1 OLED
#define BUT1_PIO			PIOD
#define BUT1_PIO_ID		    ID_PIOD
#define BUT1_PIO_IDX		28
#define BUT1_PIO_IDX_MASK   (1u << BUT1_PIO_IDX)

// Botao2 OLED
#define BUT2_PIO			PIOC
#define BUT2_PIO_ID		    ID_PIOC
#define BUT2_PIO_IDX		31
#define BUT2_PIO_IDX_MASK   (1u << BUT2_PIO_IDX)

// Botao3 OLED
#define BUT3_PIO			PIOA
#define BUT3_PIO_ID		    ID_PIOA
#define BUT3_PIO_IDX		19
#define BUT3_PIO_IDX_MASK   (1u << BUT3_PIO_IDX)

// LED placa
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_PIO_IDX_MASK (1 << LED_IDX)

// Botão placa
#define BUT_PIO      PIOA
#define BUT_PIO_ID   ID_PIOA
#define BUT_IDX  11
#define BUT_IDX_MASK (1 << BUT_IDX)

volatile char but_flag = 0;
volatile char but1_flag = 0;
volatile char but2_flag = 0;
volatile char but3_flag = 0;


/* funcao de callback/ Handler */
void but_callBack(void){
	but_flag = 1;
}
/* funcao de callback/ Handler */
void but1_callBack(void){
	but1_flag = 1;
}
/* funcao de callback/ Handler */
void but2_callBack(void){
	but2_flag = !but2_flag;
}
/* funcao de callback/ Handler */
void but3_callBack(void){
	but3_flag = 1;
}


void init(void){
	
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);
	pmc_enable_periph_clk(ID_PIOD);
	
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_PIO_IDX_MASK, PIO_DEBOUNCE);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEBOUNCE);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEBOUNCE);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_PIO_IDX_MASK, PIO_DEBOUNCE);
	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP);
	
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_IDX_MASK, PIO_IT_FALL_EDGE, but_callBack);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but1_callBack);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but2_callBack);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but3_callBack);

	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	
	pio_get_interrupt_status(BUT_PIO);
	pio_get_interrupt_status(BUT2_PIO);
	pio_get_interrupt_status(BUT1_PIO);
	pio_get_interrupt_status(BUT3_PIO);
	  
	  // Configura NVIC para receber interrupcoes do PIO do botao
	  // com prioridade 4 (quanto mais próximo de 0 maior)
	  NVIC_EnableIRQ(BUT_PIO_ID);
	  NVIC_EnableIRQ(BUT1_PIO_ID);
	  NVIC_EnableIRQ(BUT2_PIO_ID);
	  NVIC_EnableIRQ(BUT3_PIO_ID);
	  NVIC_SetPriority(BUT_PIO_ID, 4); 
	  NVIC_SetPriority(BUT1_PIO_ID, 2);
	  NVIC_SetPriority(BUT2_PIO_ID, 2);
	  NVIC_SetPriority(BUT3_PIO_ID, 2);// Prioridade 4
}

void pisca_led(){
	int t = 300;
	
	for (int i=0;i<30;i++){
		
		if(but1_flag){
			t-=100;
			gfx_mono_draw_string("+", 90,0, &sysfont);
			delay_ms(500);
			gfx_mono_draw_string(" ", 90,0, &sysfont);
			but1_flag = 0;		
	}
		if(but3_flag){
			t += 100;
			gfx_mono_draw_string("-", 20,0, &sysfont);
			delay_ms(500);
			gfx_mono_draw_string(" ", 20,0, &sysfont);
			but3_flag = 0;
	}
	
		while(but2_flag){
			pio_clear(PIOA, LED1_PIO_IDX_MASK);
			pio_clear(PIOC, LED2_PIO_IDX_MASK);
			pio_clear(PIOB, LED3_PIO_IDX_MASK);
			gfx_mono_draw_string("    STOP  ", 0,16, &sysfont);
		}
		
		but2_flag = 0;
		float progress_count = i*13/30;
		char progress_bar[14];
		char progress_complete[14];
		strcpy (progress_bar," ");
		strcpy (progress_complete,"||||||||||||||");
		strncat (progress_bar, progress_complete, progress_count);
		
		
		gfx_mono_draw_string("             ", 0,16, &sysfont);
		gfx_mono_draw_string(progress_bar, 0,16, &sysfont);
		
		char str[120];
		gfx_mono_draw_string("Ms: ", 0, 1, &sysfont);
		sprintf(str, "%d", t);
		gfx_mono_draw_string(str, 40, 1, &sysfont);
         sprintf(str, "%d", t);
         gfx_mono_draw_string(str, 40, 2, &sysfont);

		pio_clear(LED_PIO, LED_PIO_IDX_MASK);
		delay_ms(t);
		pio_set(LED_PIO, LED_PIO_IDX_MASK);
		delay_ms(t);
	}
	gfx_mono_draw_string("    LAB 3-PIO    ", 60, 0, &sysfont);
}

int main (void)
{
	init();
	board_init();
	sysclk_init();
	delay_init();

  // Init LED
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("Frequencia: ", 60, 0, &sysfont);

	//sprintf(str, "%d", cnt); 
	//gfx_mono_draw_string(str, 0, 0, &sysfont);

  
  

  /* Insert application code here, after the board has been initialized. */
	while(1) {

			if(but_flag){
				pisca_led();
				but_flag = 0;
			}
			
			pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);		
	}
}
