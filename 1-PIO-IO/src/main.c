#include "asf.h"

#define LED_PIO_TESTE           PIOC                 // periferico que controla o LED
// #
#define LED_PIO_ID        ID_PIOC                 // ID do periférico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_PIO_IDX_MASK_TESTE  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED

#define BUT_PIO36 PIOA
#define BUT_PIO_ID ID_PIOA
#define BUT_PIO_IDX 10
#define BUT_PIO36_IDX_MASK (1u << BUT_PIO_IDX) // esse já está pronto.

// Configuracoes do botao OLED
// Para encontrar os IDS DO BOTAO(PIN ON EXT CONNECTOR UTILIZAR A OLEDX1)
// Descobrindo os pins do bota ex led 1 -> 7 led 2 -> 8, utilizar o same70 tabela (4.3.1) com os pins descobrindo por fim
// seu pio_idx e id ex: led 1 pin 7 -> PA 0, led2 pin 8 -> PC 30[pioc e idx 30]

// LED 1
#define LED1_PIO           PIOA
#define LED1_PIO_ID        ID_PIOA
#define LED1_PIO_IDX       0
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)

// LED 2
#define LED2_PIO           PIOC
#define LED2_PIO_ID        ID_PIOC
#define LED2_PIO_IDX       30
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)

// LED 3
#define LED3_PIO           PIOB
#define LED3_PIO_ID        ID_PIOB
#define LED3_PIO_IDX       2
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)

// BOTAO 1
#define BUT1_PIO			PIOD
#define BUT1_PIO_ID		    ID_PIOD
#define BUT1_PIO_IDX		28
#define BUT1_PIO_IDX_MASK   (1u << BUT1_PIO_IDX)

// BOTAO 2
#define BUT2_PIO			PIOC
#define BUT2_PIO_ID		    ID_PIOC
#define BUT2_PIO_IDX		31
#define BUT2_PIO_IDX_MASK   (1u << BUT2_PIO_IDX)

// BOTAO 3
#define BUT3_PIO			PIOA
#define BUT3_PIO_ID		    ID_PIOA
#define BUT3_PIO_IDX		19
#define BUT3_PIO_IDX_MASK   (1u << BUT3_PIO_IDX)

//Encontrar ID utilziar  SAME70-XLPD 4.16 +\- --> Encontra o PIO e o Index
// Para encontar o ID_PIO utilziar a tabela 13 -peripheral do SAME70


// Função de inicialização do uC
void init(void){
	// Initialize the board clock
	sysclk_init();

	// Disativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;

	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
	//pmc_enable_periph_clk(LED_PIO_ID);
	// Inicializa PIO do botao
	// pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);
	pmc_enable_periph_clk(ID_PIOD);


	//Configura o index 8 (LED_PIO_IDX) do PIOC como sendo saída inicializada com o valor '0',
	//sem multidrive e sem resistor de pull-up. --> Deixa o led ligado
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED_PIO_TESTE, LED_PIO_IDX_MASK_TESTE, 0, 0, 0);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);


	//Setar botao -> necessario input e pull_up em cada um
	pio_set_input(BUT_PIO36, BUT_PIO36_IDX_MASK, PIO_DEFAULT);
	pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK, PIO_DEFAULT);
	pio_pull_up(BUT_PIO36,BUT_PIO36_IDX_MASK,1);
	pio_pull_up(BUT1_PIO, BUT1_PIO_IDX_MASK, 1);
	pio_pull_up(BUT2_PIO, BUT2_PIO_IDX_MASK, 1);
	pio_pull_up(BUT3_PIO, BUT3_PIO_IDX_MASK, 1);
	
	pio_configure(LED_PIO_TESTE, PIO_OUTPUT_0, LED_PIO_IDX_MASK_TESTE, PIO_DEFAULT);
	pio_configure(BUT_PIO36, PIO_INPUT, BUT_PIO36_IDX_MASK, PIO_PULLUP);
}


/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// SUPER LOOP
// Funcao principal chamada na inicalizacao do uC.
int main(void) {
	init();
	
	//desliga led --> no init o pio_set_output não é necessario já que pretendiamos desligar a led de qualquer jeito
	pio_set(LED_PIO_TESTE, LED_PIO_IDX_MASK_TESTE);
	pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
	pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
	pio_set(LED3_PIO, LED3_PIO_IDX_MASK);


	// super loop
	// aplicacoes embarcadas não devem sair do while(1).
	while(1) {
		// Verifica valor do pino que o botão está conectado
		if (!pio_get(BUT_PIO36, PIO_INPUT, BUT_PIO36_IDX_MASK)){
			for(int i=0; i<5; i++){
				pio_set(LED_PIO_TESTE, LED_PIO_IDX_MASK_TESTE);      // Coloca 1 no pino LED -> desliga
				delay_ms(200);                        
				pio_clear(LED_PIO_TESTE, LED_PIO_IDX_MASK_TESTE);    // Coloca 0 no pino do LED -> liga
				delay_ms(200);                        
			}
		}
		
		if(!pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)){
			for(int i = 0; i<5; i++){
				pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
				delay_ms(200);
				pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
				delay_ms(200);
			}
		}
		
		if(!pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK)){
			for(int i = 0; i<5; i++){
				pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
				delay_ms(200);
				pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
				delay_ms(200);
			}
		}
		
		if(!pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK)){
			for(int i = 0; i<5; i++){
				pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
				delay_ms(200);
				pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
				delay_ms(200);
			}
		}
	else  {
		// Ativa o pino LED_IDX (par apagar)
		pio_set(LED_PIO_TESTE, LED_PIO_IDX_MASK_TESTE);
		pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
		pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
		pio_set(LED3_PIO, LED3_PIO_IDX_MASK);


	}
}

	// Nunca devemos chegar aqui !
	return 0;
}