#include <asf.h>
#include <string.h>
#include "ili9341.h"
#include "lvgl.h"
#include "touch/touch.h"

LV_FONT_DECLARE(dseg70);
LV_FONT_DECLARE(dseg50);
LV_FONT_DECLARE(dseg20);
LV_FONT_DECLARE(clock);

/************************************************************************/
/* LCD / LVGL                                                           */
/************************************************************************/

#define LV_HOR_RES_MAX          (320)
#define LV_VER_RES_MAX          (240)
#define SIMPLE_CLOCK "\xEF\x80\x97"

#define AFEC_POT0 AFEC0
#define AFEC_POT0_ID ID_AFEC0
#define AFEC_POT0_CHANNEL 8 // Canal do pino PC31

// LED ON/Off
#define LEDOnOff_PIO PIOA
#define LEDOnOff_PIO_ID ID_PIOA
#define LEDOnOff_IDX 3
#define LEDOnOff_IDX_MASK (1 << LEDOnOff_IDX)

/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];
static lv_disp_drv_t disp_drv;          /*A variable to hold the drivers. Must be static or global.*/
static lv_indev_drv_t indev_drv;

static lv_obj_t * labelBtn1;

volatile int flag = 0;

SemaphoreHandle_t xSemaphoreTIME;
QueueHandle_t xQueueVAR;

lv_obj_t * labelFloor;
lv_obj_t * labelPONTO;
lv_obj_t * labelCELSIUS;
lv_obj_t * labelTEMP;
lv_obj_t * labelHORA;
lv_obj_t * labelDOIS_PONTOS;
lv_obj_t * labelMIN;
lv_obj_t * labelCASINHA;

typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_ADAFRUIT_STACK_PRIORITY            (tskIDLE_PRIORITY)
#define TASK_ADAFRUIT_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_TIME_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_TIME_STACK_PRIORITY            (tskIDLE_PRIORITY)
#define TASK_TEMP_VAR_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_TEMP_VAR_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationMallocFailedHook(void);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void xPortSysTickHandler(void);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
static void light_init_(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

void alarme (int time) {
	/* Leitura do valor atual do RTC */           
    uint32_t current_hour, current_min, current_sec;
    uint32_t current_year, current_month, current_day, current_week;
    rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
    rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
	
    /* configura alarme do RTC para daqui 20 segundos */                                                                   
    rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);                              
    rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + time);
}

/************************************************************************/
/* lvgl                                                                 */
/************************************************************************/

static void event_handler_ONOFF(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
	else if(code == LV_EVENT_CLICKED) {
		pio_clear(LEDOnOff_PIO, LEDOnOff_IDX_MASK);
	}
}

static void event_handler_MENU(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

static void event_handler_RELOGIO(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

static void event_handler_TEMP_UP(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	char *c;
    int temp;
    if(code == LV_EVENT_CLICKED) {
        c = lv_label_get_text(labelTEMP);
        temp = atoi(c);
        lv_label_set_text_fmt(labelTEMP, "%02d", temp + 1);
    }
}

static void event_handler_TEMP_DOWN(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	char *c;
    int temp;
    if(code == LV_EVENT_CLICKED) {
        c = lv_label_get_text(labelTEMP);
        temp = atoi(c);
        lv_label_set_text_fmt(labelTEMP, "%02d", temp - 1);
    }
}

void RTC_Handler(void) {	
	uint32_t ul_status = rtc_get_status(RTC);
	
    /* seccond tick */
    if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {	
	// o código para irq de segundo vem aqui
		xSemaphoreGiveFromISR(xSemaphoreTIME, 0);
	}
	
    /* Time or date alarm */
    if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
    	// o código para irq de alame vem aqui
    }

    rtc_clear_status(RTC, RTC_SCCR_SECCLR);
    rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
    rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
    rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
    rtc_clear_status(RTC, RTC_SCCR_CALCLR);
    rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void task_time(void) {
	while (1) {
		if( xSemaphoreTake(xSemaphoreTIME, 0)){
			uint32_t current_hour, current_min, current_sec;
			uint32_t current_year, current_month, current_day, current_week;
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
			
			lv_label_set_text_fmt(labelHORA, "%02d", current_hour);
			lv_label_set_text_fmt(labelMIN, "%02d", current_min);
			if (current_sec % 2 == 0) {
				lv_label_set_text(labelDOIS_PONTOS, ":");
			} else {
				lv_label_set_text(labelDOIS_PONTOS, " ");
			}
      	}
	}
	
}

static void AFEC_pot0_callback(void) {
    int value = afec_channel_get_value(AFEC_POT0, AFEC_POT0_CHANNEL);
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    xQueueSendFromISR(xQueueVAR, &value, &xHigherPriorityTaskWoken);
}

void RTT_Handler(void) {
    uint32_t ul_status;
    ul_status = rtt_get_status(RTT);

    /* IRQ due to Alarm */
    if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
        afec_channel_enable(AFEC_POT0, AFEC_POT0_CHANNEL);
  		afec_start_software_conversion(AFEC_POT0);
    }
}

static void task_temp_var(void *pvParameters) {
  // configura ADC e TC para controlar a leitura
  config_AFEC_pot(AFEC_POT0, AFEC_POT0_ID, AFEC_POT0_CHANNEL, AFEC_pot0_callback);

  // variável para recever dados da fila
  int data;
  while (1) {
  	RTT_init(1000, 100, RTT_MR_ALMIEN);
    if (xQueueReceive(xQueueVAR, &(data), 1000)) {
		int temp = (data * 100) / 4096;
		lv_label_set_text_fmt(labelTEMP, "%02d", temp);
	}
  }
}

void lv_termostato(void) {
	static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_bg_color(&style, lv_palette_main(0xFFFFFF));
    lv_style_set_border_color(&style, lv_palette_main(0x000000));
    lv_style_set_border_width(&style, 0);
	
	// Butão de Ligar/Desligar
    lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn1, event_handler_ONOFF, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_BOTTOM_LEFT, 5, -20);
	lv_obj_add_style(btn1, &style, 0);

	labelBtn1 = lv_label_create(btn1);
	lv_label_set_text(labelBtn1, "[  " LV_SYMBOL_POWER);
	lv_obj_center(labelBtn1);

	// Butão de menu
	lv_obj_t * btn2 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn2, event_handler_MENU, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btn2, btn1, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
	lv_obj_add_style(btn2, &style, 0);

	lv_obj_t * labelbnt2 = lv_label_create(btn2);
	lv_label_set_text(labelbnt2, "| M |");
	lv_obj_center(labelbnt2);

	// Butão de Relógio
	lv_obj_t * btn3 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn3, event_handler_RELOGIO, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btn3, btn2, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
	lv_obj_add_style(btn3, &style, 0);

	lv_obj_t * labelbnt3 = lv_label_create(btn3);
	lv_obj_set_style_text_font(labelbnt3, &clock, LV_STATE_DEFAULT);
	lv_label_set_text(labelbnt3, SIMPLE_CLOCK "  ]");
	lv_obj_center(labelbnt3);

	// seta a temperatura cima
	lv_obj_t * bnt4 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(bnt4, event_handler_TEMP_UP, LV_EVENT_ALL, NULL);
	lv_obj_align_to(bnt4, btn3, LV_ALIGN_OUT_RIGHT_TOP, 20, 0);
	lv_obj_add_style(bnt4, &style, 0);

	lv_obj_t * labelbnt4 = lv_label_create(bnt4);
	lv_label_set_text(labelbnt4, "[  " LV_SYMBOL_UP);
	lv_obj_center(labelbnt4);

	// seta a temperatura baixo
	lv_obj_t * bnt5 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(bnt5, event_handler_TEMP_DOWN, LV_EVENT_ALL, NULL);
	lv_obj_align_to(bnt5, bnt4, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
	lv_obj_add_style(bnt5, &style, 0);
	
	lv_obj_t * labelbnt5 = lv_label_create(bnt5);
	lv_label_set_text(labelbnt5, LV_SYMBOL_DOWN);
	lv_obj_center(labelbnt5);
	
	// Label Floor
	labelFloor = lv_label_create(lv_scr_act());
    lv_obj_align(labelFloor, LV_ALIGN_LEFT_MID, 30, -30);
    lv_obj_set_style_text_font(labelFloor, &dseg70, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(labelFloor, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(labelFloor, "%.1f", 22.44);

	// Label TEMP
	labelTEMP = lv_label_create(lv_scr_act());
    lv_obj_align(labelTEMP, LV_ALIGN_TOP_RIGHT, 0, 45);
    lv_obj_set_style_text_font(labelTEMP, &dseg50, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(labelTEMP, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(labelTEMP, "%02d", 22);
	
	calendar rtc_initial = {2022, 3, 19, 12, 15, 45 ,1};                                            
    RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_SECEN);

	/* Leitura do valor atual do RTC */           
    uint32_t current_hour, current_min, current_sec;
    uint32_t current_year, current_month, current_day, current_week;
    rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
    rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
	
	// Label Horas
	labelHORA = lv_label_create(lv_scr_act());
	lv_obj_align(labelHORA, LV_ALIGN_TOP_RIGHT, -35, 5);
	lv_obj_set_style_text_font(labelHORA, &dseg20, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelHORA, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelHORA, "%02d", current_hour);

	// label dois pontos
	labelDOIS_PONTOS = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelDOIS_PONTOS, labelHORA, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
	lv_obj_set_style_text_font(labelDOIS_PONTOS, &dseg20, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelDOIS_PONTOS, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text(labelDOIS_PONTOS, ":");

	// Label Minutos
	labelMIN = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelMIN, labelDOIS_PONTOS, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
	lv_obj_set_style_text_font(labelMIN, &dseg20, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelMIN, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelMIN, "%02d", current_min);

	// Label casinha
	labelCASINHA = lv_label_create(lv_scr_act());
	lv_obj_align(labelCASINHA, LV_ALIGN_BOTTOM_LEFT, 100, -50);

	lv_obj_t * labelCASA = lv_label_create(labelCASINHA);
	lv_label_set_text(labelCASA, LV_SYMBOL_HOME);
	lv_obj_center(labelCASA);

}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_adafruit(void *pvParameters) {
	int px, py;

	// lv_ex_btn_1();
	lv_termostato();
	lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);

	for (;;)  {
		// alarme(1);
		lv_tick_inc(50);
		lv_task_handler();
		vTaskDelay(50);
	}
}

/************************************************************************/
/* configs                                                              */
/************************************************************************/

static void light_init_(void) {
    // Ativa PIOs
    pmc_enable_periph_clk(LEDOnOff_PIO_ID);

    // Configura Pinos
    pio_configure(LEDOnOff_PIO, PIO_OUTPUT_1, LEDOnOff_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
}

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
    /*************************************
     * Ativa e configura AFEC
     *************************************/
    /* Ativa AFEC - 0 */
    afec_enable(afec);

    /* struct de configuracao do AFEC */
    struct afec_config afec_cfg;

    /* Carrega parametros padrao */
    afec_get_config_defaults(&afec_cfg);

    /* Configura AFEC */
    afec_init(afec, &afec_cfg);

    /* Configura trigger por software */
    afec_set_trigger(afec, AFEC_TRIG_SW);

    /*** Configuracao específica do canal AFEC ***/
    struct afec_ch_config afec_ch_cfg;
    afec_ch_get_config_defaults(&afec_ch_cfg);
    afec_ch_cfg.gain = AFEC_GAINVALUE_0;
    afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

    /*
    * Calibracao:
    * Because the internal ADC offset is 0x200, it should cancel it and shift
    down to 0.
    */
    afec_channel_set_analog_offset(afec, afec_channel, 0x200);

    /***  Configura sensor de temperatura ***/
    struct afec_temp_sensor_config afec_temp_sensor_cfg;

    afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
    afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

    /* configura IRQ */
    afec_set_callback(afec, afec_channel, callback, 1);
    NVIC_SetPriority(afec_id, 4);
    NVIC_EnableIRQ(afec_id);
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

static void configure_lcd(void) {
	/**LCD pin configure on SPI*/
	pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS);  //
	pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
	pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
	pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
	pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
	pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);
	
	ili9341_init();
	ili9341_backlight_on();


static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT,
	};

	/* Configure console UART. */
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses,
uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);

	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT))
		;
		rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN)){
		rtt_enable_interrupt(RTT, rttIRQSource);
	}
	else{
		rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	}
}

/************************************************************************/
/* port lvgl                                                            */
/************************************************************************/

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
	ili9341_set_top_left_limit(area->x1, area->y1);   ili9341_set_bottom_right_limit(area->x2, area->y2);
	ili9341_copy_pixels_to_screen(color_p,  (area->x2 + 1 - area->x1) * (area->y2 + 1 - area->y1));
	
	/* IMPORTANT!!!
	* Inform the graphics library that you are ready with the flushing*/
	lv_disp_flush_ready(disp_drv);
}

void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
	int px, py, pressed;
	if (readPoint(&px, &py)){
		data->state = LV_INDEV_STATE_PRESSED;
		printf("FLAG: %d\n", flag);
		if (!pio_get(LEDOnOff_PIO, PIO_INPUT, LEDOnOff_IDX_MASK) && flag == 5){
			pio_set(LEDOnOff_PIO, LEDOnOff_IDX_MASK);
		}
		flag += 1;
		if (flag > 5) 
			flag = 5;
	} else {
		data->state = LV_INDEV_STATE_RELEASED;
		printf("FLAG: %d\n", flag);
		flag -= 1;
		if (flag < -5) 
			flag = -5;
	}
	
	data->point.x = px;
	data->point.y = py;
}

void configure_lvgl(void) {
	lv_init();
	lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);
	
	lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
	disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
	disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
	disp_drv.hor_res = LV_HOR_RES_MAX;      /*Set the horizontal resolution in pixels*/
	disp_drv.ver_res = LV_VER_RES_MAX;      /*Set the vertical resolution in pixels*/

	lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
	
	/* Init input on LVGL */
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_input_read;
	lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* board and sys init */
	board_init();
	sysclk_init();
	configure_console();
	light_init_();

	/* LCd, touch and lvgl init*/
	configure_lcd();
	configure_touch();
	configure_lvgl();

	xSemaphoreTIME = xSemaphoreCreateBinary();

	xQueueVAR = xQueueCreate(32, sizeof(int) );
	
	ili9341_backlight_off();
	
	// verifica se semáforo foi criado corretamente
	if (xSemaphoreTIME == NULL)
		printf("falha em criar o semaforo \n");

	if (xTaskCreate(task_time, "TIME", TASK_TIME_STACK_SIZE, NULL, TASK_TIME_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create time task\r\n");
	}

	/* Create task to control oled */
	if (xTaskCreate(task_adafruit, "adafruit", TASK_ADAFRUIT_STACK_SIZE, NULL, TASK_ADAFRUIT_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}

	if (xTaskCreate(task_temp_var, "TEMP_VAR", TASK_TEMP_VAR_STACK_SIZE, NULL, TASK_TEMP_VAR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create temp var task\r\n");
	}
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){ }
}
