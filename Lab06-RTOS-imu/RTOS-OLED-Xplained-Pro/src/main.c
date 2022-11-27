#include <asf.h>
#include "conf_board.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "mcu6050.h"
#include "math.h"
#include "Fusion/Fusion.h"

#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

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
#define TASK_IMU_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_IMU_STACK_PRIORITY            (tskIDLE_PRIORITY)


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

// Filas
QueueHandle_t xQueueLado;
SemaphoreHandle_t xSemaphoreHouseDown;

enum lado{ESQUERDA, FRENTE, DIREITA};


/** prototypes */
static void BUT_init(void);
int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
configASSERT( ( volatile void * ) NULL );}
int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int32_t ierror = 0x00;

	twihs_packet_t p_packet;
	p_packet.chip         = dev_addr;
	p_packet.addr[0]      = reg_addr;
	p_packet.addr_length  = 1;
	p_packet.buffer       = reg_data;
	p_packet.length       = cnt;

	ierror = twihs_master_write(TWIHS2, &p_packet);

	return (int8_t)ierror;
}
int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int32_t ierror = 0x00;

	twihs_packet_t p_packet;
	p_packet.chip         = dev_addr;
	p_packet.addr[0]      = reg_addr;
	p_packet.addr_length  = 1;
	p_packet.buffer       = reg_data;
	p_packet.length       = cnt;

	// TODO: Algum problema no SPI faz com que devemos ler duas vezes o registrador para
	//       conseguirmos pegar o valor correto.
	ierror = twihs_master_read(TWIHS2, &p_packet);

	return (int8_t)ierror;
}



void mcu6050_i2c_bus_init(void){
	twihs_options_t mcu6050_option;
	pmc_enable_periph_clk(ID_TWIHS2);

	/* Configure the options of TWI driver */
	mcu6050_option.master_clk = sysclk_get_cpu_hz();
	mcu6050_option.speed      = 40000;
	twihs_master_init(TWIHS2, &mcu6050_option);
	
	/** Enable TWIHS port to control PIO pins */
	pmc_enable_periph_clk(ID_PIOD);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 28);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 27);
}




void led_init(void){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);

	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 1, 0, 0);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 1, 0, 0);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 1, 0, 0);
	
}

void pisca_led(Pio * pio , const uint32_t idx_mask , int n){
	// Pisca LED
	int i = 0;
	for (int i = 0 ; i<n; i++){
		pio_clear(pio, idx_mask);  // Seta pino para 0
		delay_ms(100);
		pio_set(pio, idx_mask);
		delay_ms(100);
	}
}
void apaga_tela() {
	gfx_mono_draw_filled_rect(0, 0, 120, 30, GFX_PIXEL_CLR);
}

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
	gfx_mono_draw_string("oii", 0, 20, &sysfont);

	for (;;)  {
    

	}
}


/**********************************************************sssss**************/
/* TASKS                                                                */
/************************************************************************/


static void task_house_down(void *pvParameters) {
	led_init();
	// Piscar 10 vezes
	int n = 10;

	for (;;)  {
		if(xSemaphoreTake(xSemaphoreHouseDown, 1000)){
			pisca_led(LED_PIO, LED_IDX_MASK, n);
		}
		else{
			pio_set(LED_PIO, LED_IDX_MASK);
		}
	}
}

static void task_orientacao(void *pvParameters) {
	led_init();

	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("aaaaaaaaaa", 0, 0, &sysfont);
	printf("ashdsauuhuha");
	int lado;
	for (;;)  {
		if(xQueueReceive(xQueueLado, &(lado), 0)){
			if (lado == ESQUERDA){
				apaga_tela();
				gfx_mono_draw_string("esquerda", 0, 0, &sysfont);
				pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
				pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
				pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
			}
			if (lado == FRENTE){
				apaga_tela();
				
				gfx_mono_draw_string("frente", 0, 0, &sysfont);
				pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
				pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
				pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
			}
			if (lado == DIREITA){
				apaga_tela();


				printf("direita");
				gfx_mono_draw_string("direita", 0, 0, &sysfont);
				pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
				pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
				pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
			}
			if (lado == 4){
				apaga_tela();
				pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
				pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
				pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
			}
		}
	}
}


static void task_imu(void *pvParameters) {
	mcu6050_i2c_bus_init();


	/* Inicializa Fun  o de fus o */
	FusionAhrs ahrs;
	FusionAhrsInitialise(&ahrs);
	/* buffer para recebimento de dados */
	uint8_t bufferRX[10];
	uint8_t bufferTX[10];
	
	int16_t  raw_acc_x, raw_acc_y, raw_acc_z;
	volatile uint8_t  raw_acc_xHigh, raw_acc_yHigh, raw_acc_zHigh;
	volatile uint8_t  raw_acc_xLow,  raw_acc_yLow,  raw_acc_zLow;
	float proc_acc_x, proc_acc_y, proc_acc_z;

	int16_t  raw_gyr_x, raw_gyr_y, raw_gyr_z;
	volatile uint8_t  raw_gyr_xHigh, raw_gyr_yHigh, raw_gyr_zHigh;
	volatile uint8_t  raw_gyr_xLow,  raw_gyr_yLow,  raw_gyr_zLow;
	float proc_gyr_x, proc_gyr_y, proc_gyr_z;


	/* resultado da fun  o */
	uint8_t rtn;
	
	 rtn = twihs_probe(TWIHS2, MPU6050_DEFAULT_ADDRESS);
	 if(rtn != TWIHS_SUCCESS){
		 printf("[ERRO] [i2c] [probe] \n");
		 } else {
		 printf("[DADO] [i2c] probe OK\n" );
	 }
	
	// L  registrador WHO AM I
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, bufferRX, 1);
	if(rtn != TWIHS_SUCCESS){
		printf("[ERRO] [i2c] [read] \n");
		} else {
		printf("[DADO] [i2c] %x:%x", MPU6050_RA_WHO_AM_I, bufferRX[0]);
		printf("%x", bufferRX[0]);
	}
	
	if (bufferRX[0] == 104){
		printf("SUCESSO!");
	}
	else{
		printf("Incorreto :(");
	}
	
	// Set Clock source
	bufferTX[0] = MPU6050_CLOCK_PLL_XGYRO;
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, bufferTX, 1);
	if(rtn != TWIHS_SUCCESS)
	printf("[ERRO] [i2c] [write] \n");

	// Aceletromtro em 2G
	bufferTX[0] = MPU6050_ACCEL_FS_2 << MPU6050_ACONFIG_AFS_SEL_BIT;
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, bufferTX, 1);
	if(rtn != TWIHS_SUCCESS)
	printf("[ERRO] [i2c] [write] \n");

	// Configura range giroscopio para operar com 250  /s
	bufferTX[0] = 0x00; // 250  /s
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, bufferTX, 1);
	if(rtn != TWIHS_SUCCESS)
	printf("[ERRO] [i2c] [write] \n");
	float valor_anterior_yaw = 0.0;
	float roll, pitch, yaw;
	int d;
	while(1) {
		// Le valor do acc X High e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &raw_acc_xHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, &raw_acc_xLow,  1);

		// Le valor do acc y High e  Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, &raw_acc_yHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_yLow,  1);

		// Le valor do acc z HIGH e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, &raw_acc_zHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_zLow,  1);

		// Dados s o do tipo complemento de dois
		raw_acc_x = (raw_acc_xHigh << 8) | (raw_acc_xLow << 0);
		raw_acc_y = (raw_acc_yHigh << 8) | (raw_acc_yLow << 0);
		raw_acc_z = (raw_acc_zHigh << 8) | (raw_acc_zLow << 0);

		// Le valor do gyr X High e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_H, &raw_gyr_xHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_L, &raw_gyr_xLow,  1);

		// Le valor do gyr y High e  Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_YOUT_H, &raw_gyr_yHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_yLow,  1);

		// Le valor do gyr z HIGH e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, &raw_gyr_zHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_zLow,  1);

		// Dados s o do tipo complemento de dois
		raw_gyr_x = (raw_gyr_xHigh << 8) | (raw_gyr_xLow << 0);
		raw_gyr_y = (raw_gyr_yHigh << 8) | (raw_gyr_yLow << 0);
		raw_gyr_z = (raw_gyr_zHigh << 8) | (raw_gyr_zLow << 0);

		// Dados em escala real
		proc_acc_x = (float)raw_acc_x/16384;
		proc_acc_y = (float)raw_acc_y/16384;
		proc_acc_z = (float)raw_acc_z/16384;

		proc_gyr_x = (float)raw_gyr_x/131;
		proc_gyr_y = (float)raw_gyr_y/131;
		proc_gyr_z = (float)raw_gyr_z/131;
		
		//printf("proc_acc_x: %f \n", proc_acc_x);
		//printf("proc_acc_y: %f \n", proc_acc_y);
		//printf("proc_acc_z: %f \n", proc_acc_z);
		
		//printf("proc_gyr_x: %f \n", proc_gyr_x);
		//printf("proc_gyr_y: %f \n", proc_gyr_y);
		//printf("proc_gyr_z: %f \n", proc_gyr_z);
		
		if(sqrt(pow(proc_acc_x, 2) + pow(proc_acc_y, 2) + pow(proc_acc_z, 2)) < 0.1){
			printf("Em Queda!!");
			xSemaphoreGive(xSemaphoreHouseDown);
		}
		
		const FusionVector gyroscope = {proc_gyr_x, proc_gyr_y, proc_gyr_z};
		const FusionVector accelerometer = {proc_acc_x, proc_acc_y, proc_acc_z};
			
		// Tempo entre amostras
		float dT = 0.1;

		// aplica o algoritmo
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dT);

		// dados em pitch roll e yaw
		const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

		printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
		roll = euler.angle.roll;
		pitch = euler.angle.pitch;
		yaw = euler.angle.yaw;
		int d;
		if ((yaw - sqrt(pow(valor_anterior_yaw, 2)) > 30) && pitch < 15){
			d = ESQUERDA;
			xQueueSend(xQueueLado, &d, 0);
		}
		else if (yaw < -30 && pitch < 15){
			d = DIREITA;
			xQueueSend(xQueueLado, &d, 0);
		}
		else if (roll < -20 && pitch > 30){
			d = FRENTE;
			xQueueSend(xQueueLado, &d, 0);
		}
		else {
			d = 4;
			xQueueSend(xQueueLado, &d, 0);
		}
		// uma amostra a cada 1ms
		vTaskDelay(10);
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_IMU_STACK_SIZE, NULL, TASK_IMU_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	if (xTaskCreate(task_imu, "imu", TASK_IMU_STACK_SIZE, NULL, TASK_IMU_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled imu\r\n");
	}

	if (xTaskCreate(task_house_down, "house_down", TASK_IMU_STACK_SIZE, NULL, TASK_IMU_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create `HOUSE DOWN\r\n");
	}

	if (xTaskCreate(task_orientacao, "orientacao", TASK_IMU_STACK_SIZE, NULL, TASK_IMU_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create ORIENTATION\r\n");
	}

	// Sem?foro
	xSemaphoreHouseDown = xSemaphoreCreateBinary();
	if (xSemaphoreHouseDown == NULL){
		printf("falha em criar o semaforo \n");
	}
	
	xQueueLado = xQueueCreate(32, sizeof(int));
	
	if (xQueueLado == NULL)
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS n?o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
