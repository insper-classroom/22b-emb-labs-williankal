#include "asf.h"


/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define LED_PIO           PIOC                 // periferico que controla o LED
#define LED_PIO_ID        ID_PIOC                   // ID do periférico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED

#define BUT_PIO			  PIOA
#define BUT_PIO_ID		  ID_PIOA
#define BUT_PIO_IDX		  11
#define BUT_PIO_IDX_MASK  (1u << BUT_PIO_IDX)

// Brincando com o oled1
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

// BT1
#define BUT1_PIO			PIOD
#define BUT1_PIO_ID		    ID_PIOD
#define BUT1_PIO_IDX		28
#define BUT1_PIO_IDX_MASK   (1u << BUT1_PIO_IDX)

// BT2
#define BUT2_PIO			PIOC
#define BUT2_PIO_ID		    ID_PIOC
#define BUT2_PIO_IDX		31
#define BUT2_PIO_IDX_MASK   (1u << BUT2_PIO_IDX)

// BT3
#define BUT3_PIO			PIOA
#define BUT3_PIO_ID		    ID_PIOA
#define BUT3_PIO_IDX		19
#define BUT3_PIO_IDX_MASK   (1u << BUT3_PIO_IDX)


/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE            (1u << 3)


void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_SODR = ul_mask;
}


/**
 * \brief Set a low output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio -> PIO_CODR = ul_mask;

}

/**
 * \brief Configure PIO internal pull-up.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 * \param ul_pull_up_enable Indicates if the pin(s) internal pull-up shall be
 * configured.
 */
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask,
        const uint32_t ul_pull_up_enable){
	if (ul_pull_up_enable) {
		p_pio->PIO_PUER = ul_mask;
	} 
	else {
		p_pio->PIO_PUDR = ul_mask;
	}
}
 
 /**
 * \brief Configure one or more pin(s) or a PIO controller as inputs.
 * Optionally, the corresponding internal pull-up(s) and glitch filter(s) can
 * be enabled.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure as input(s).
 * \param ul_attribute PIO attribute(s).
 */
void _pio_set_input(Pio *p_pio, const uint32_t ul_mask,
        const uint32_t ul_attribute)
{
	pio_pull_up(p_pio, ul_mask, ul_attribute);
	if (ul_attribute & PIO_PULLUP) {
		p_pio -> PIO_IFER = ul_mask;
		} 
	else {
		p_pio -> PIO_IFDR = ul_mask;
	}		
}

/**
 * \brief Configure one or more pin(s) of a PIO controller as outputs, with
 * the given default value. Optionally, the multi-drive feature can be enabled
 * on the pin(s).
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure.
 * \param ul_default_level Default level on the pin(s).
 * \param ul_multidrive_enable Indicates if the pin(s) shall be configured as
 * open-drain.
 * \param ul_pull_up_enable Indicates if the pin shall have its pull-up
 * activated.
 */
void _pio_set_output(Pio *p_pio, const uint32_t ul_mask,
        const uint32_t ul_default_level,
        const uint32_t ul_multidrive_enable,
        const uint32_t ul_pull_up_enable){
		
		pio_pull_up(p_pio, ul_mask, ul_pull_up_enable);
		
		if (ul_default_level) {
			p_pio->PIO_SODR = ul_mask;
			}
		 else {
			p_pio->PIO_CODR = ul_mask;
		}
		
		if (ul_multidrive_enable) {
			p_pio->PIO_MDER = ul_mask;
			} 
		else {
			p_pio->PIO_MDDR = ul_mask;
		}
		p_pio -> PIO_OER = ul_mask;
		p_pio -> PIO_PER = ul_mask;
}

void init(void)
{
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);
	pmc_enable_periph_clk(ID_PIOD);
	
	_pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
	_pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, PIO_DEFAULT | _PIO_DEBOUNCE);
	_pio_pull_up(BUT_PIO, BUT_PIO_IDX_MASK, 1);
	
	_pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	_pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
	_pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);
	
	_pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, PIO_DEFAULT| _PIO_DEBOUNCE);
	_pio_pull_up(BUT1_PIO, BUT1_PIO_IDX_MASK, 1);
	_pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, PIO_DEFAULT| _PIO_DEBOUNCE);
	_pio_pull_up(BUT2_PIO, BUT2_PIO_IDX_MASK, 1);
	_pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK, PIO_DEFAULT| _PIO_DEBOUNCE);
	_pio_pull_up(BUT3_PIO, BUT3_PIO_IDX_MASK, 1);
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/




// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
	init();
	sysclk_init();
	delay_init();

	while (1){
		_pio_set(PIOC, LED_PIO_IDX_MASK);
		_pio_set(PIOA, LED1_PIO_IDX_MASK);
		_pio_set(PIOB, LED3_PIO_IDX_MASK);
		_pio_set(PIOC, LED2_PIO_IDX_MASK);
		
		if (pio_get(PIOA, PIO_INPUT, BUT_PIO_IDX_MASK) == 0){
			for(int i=0; i<5; i++){
				_pio_set(PIOC, LED_PIO_IDX_MASK);      // Coloca 1 no pino LED
				delay_ms(200);                        // Delay por software de 200 ms
				_pio_clear(PIOC, LED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
				delay_ms(200);                        // Delay por software de 200 ms
			}
		}
		
		if (pio_get(PIOA, PIO_INPUT, BUT3_PIO_IDX_MASK) == 0){
			for(int i=0; i<5; i++){
				_pio_set(PIOB, LED3_PIO_IDX_MASK);      // Coloca 1 no pino LED
				delay_ms(200);                        // Delay por software de 200 ms
				_pio_clear(PIOB, LED3_PIO_IDX_MASK);    // Coloca 0 no pino do LED
				delay_ms(200);                        // Delay por software de 200 ms
			}
		}
		
		if (pio_get(PIOD, PIO_INPUT, BUT1_PIO_IDX_MASK) == 0){
			for(int i=0; i<5; i++){
				_pio_set(PIOA, LED1_PIO_IDX_MASK);
				delay_ms(200);
				_pio_clear(PIOA, LED1_PIO_IDX_MASK);
				delay_ms(200);
			}
		}
		
		if (pio_get(PIOC, PIO_INPUT, BUT2_PIO_IDX_MASK) == 0){
			for(int i=0; i<5; i++){
				_pio_set(PIOC, LED2_PIO_IDX_MASK);      // Coloca 1 no pino LED
				delay_ms(200);                        // Delay por software de 200 ms
				_pio_clear(PIOC, LED2_PIO_IDX_MASK);    // Coloca 0 no pino do LED
				delay_ms(200);                        // Delay por software de 200 ms
			}
		}
	}
	return 0;
}
