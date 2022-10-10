#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* BOARD CONFIG                                                         */
/************************************************************************/

#define TRIGGER_PIO PIOD
#define TRIGGER_PIO_ID ID_PIOD
#define TRIGGER_PIO_IDX 30
#define TRIGGER_PIO_IDX_MASK (1 << TRIGGER_PIO_IDX)

#define ECHO_PIO PIOA
#define ECHO_PIO_ID ID_PIOA
#define ECHO_PIO_PIN 6
#define ECHO_PIO_PIN_MASK (1 << ECHO_PIO_PIN)

#define BUT_2_PIO PIOA
#define BUT_2_PIO_ID ID_PIOA
#define BUT_2_IDX 19
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

/************************************************************************/
/* RTOS                                                                */
/************************************************************************/


#define TASK_TRIGGER_STACK_SIZE               (1024*6/sizeof(portSTACK_TYPE))
#define TASK_TRIGGER_STACK_PRIORITY            (tskIDLE_PRIORITY)
#define TASK_ECHO_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_ECHO_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

/** Semaforo a ser usado pela task echo */
SemaphoreHandle_t xSemaphoreEcho;
QueueHandle_t xQueueModo;


/** Queue for msg log send data */

/************************************************************************/
/* prototypes local                                                     */
/************************************************************************/
void echo_callback();
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void trigger_init();
static void echo_init(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}
extern void vApplicationIdleHook(void) { pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); }
extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
    configASSERT((volatile void *)NULL);
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
/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void echo_callback() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphoreEcho, &xHigherPriorityTaskWoken);
}

void screen_show(int dist) {
    gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
			if(dist == -1){
				gfx_mono_draw_string("Aproxime  ", 10, 10, &sysfont);			
				}
			
			else {
				char dist_str[10];
				sprintf(dist_str, "%d cm   ", dist);
				gfx_mono_draw_string(dist_str, 5, 10, &sysfont);
			}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
static void task_echo(void *pvParameters) {
	gfx_mono_ssd1306_init();
    echo_init();
    
    for (;;) {
        /* aguarda por tempo inderteminado at� a liberacao do semaforo */
        if (xSemaphoreTake(xSemaphoreEcho, 100)) {
            if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK)) {
                RTT_init(10000, 0, 0);
            } else {
				int distancia;
                uint32_t time_read = rtt_read_timer_value(RTT);
                distancia = 1.0 / 10000 * time_read *170 * 100;
				screen_show(distancia);
            }
        } 
    }
}

static void task_trigger(void *pvParameters) {
    trigger_init();
    for (;;) {
        if (!pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK)) {
            vTaskDelay(60);
            pio_set(TRIGGER_PIO, TRIGGER_PIO_IDX_MASK);
            delay_us(10);
            pio_clear(TRIGGER_PIO, TRIGGER_PIO_IDX_MASK);
        }
    }
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

/**
 * \brief Configure the console UART.
 */



static void echo_init(void) {
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK, 0);

	// Configura interrup��o no pino referente ao botao e associa
	// fun��o de callback caso uma interrup��o for gerada
	// a fun��o de callback � a: but_callback()
	pio_handler_set(ECHO_PIO,
	ECHO_PIO_ID,
	ECHO_PIO_PIN_MASK,
	PIO_IT_EDGE,
	echo_callback);

	// Ativa interrup��o e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_PIN_MASK);
	pio_get_interrupt_status(ECHO_PIO);
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4); // Prioridade 4
}

void trigger_init() {
    pmc_enable_periph_clk(TRIGGER_PIO_ID);
    pio_set_output(TRIGGER_PIO, TRIGGER_PIO_IDX_MASK, 1, 0, 0);
};

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
    /* Initialize the SAM system */
    sysclk_init();
    xSemaphoreEcho = xSemaphoreCreateBinary();
    /* cria queue com 32 "espacos" */
    /* cada espa�o possui o tamanho de um inteiro*/
	xQueueModo = xQueueCreate(32, sizeof(int16_t));

    if (xTaskCreate(task_trigger, "trig", TASK_TRIGGER_STACK_SIZE, NULL,
    TASK_TRIGGER_STACK_PRIORITY, NULL) != pdPASS) {
	    printf("Failed to create trig task\r\n");
    }
    /* Create task to monitor processor activity */
    if (xTaskCreate(task_echo, "echo", TASK_ECHO_STACK_SIZE, NULL,
                    TASK_ECHO_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create echo task\r\n");
    } 


    /* Start the scheduler. */
    vTaskStartScheduler();

    /* RTOS n�o deve chegar aqui !! */
    while (1) {
    }

    /* Will only get here if there was insufficient memory to create the idle
     * task. */
    return 0;
}
