#include <errno.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/alt_timestamp.h>
#include <sys/alt_irq.h>

#include "io.h"
#include "altera_avalon_pio_regs.h"
#include "fifoed_avalon_uart_regs.h"
#include "fifoed_avalon_uart.h"

#include "system.h"

#define SLEEP_DELAY_US (1000 * 1000)

int uart_getRXFifoLevel()
{

	return IORD_FIFOED_AVALON_UART_RX_FIFO_USED(FIFOED_AVALON_UART_1_BASE);

}

void tx_enable_UART1(bool activate) {

	if(activate)
		IOWR_ALTERA_AVALON_PIO_DATA(FIFOED_AVALON_UART_1_TXENABLE_BASE, 0x1);
	else
		IOWR_ALTERA_AVALON_PIO_DATA(FIFOED_AVALON_UART_1_TXENABLE_BASE, 0x0);
}

void enable_led(bool onoff) {
	uint32_t led = IORD_ALTERA_AVALON_PIO_DATA(NIOS_ACTIVITY_LED_1_BASE);

	if (onoff)
		led |= 0x01;
	else
		led &= 0x00;

	IOWR_ALTERA_AVALON_PIO_DATA(NIOS_ACTIVITY_LED_1_BASE, led);
}


#define RAMDEST_UART1_RX_START  (void*)0x24000000
#define RAMDEST_UART1_RX_END    (void*)0x25FFFFFF
#define RAMDEST_UART1_TX_START  (void*)0x26000000
#define RAMDEST_UART1_TX_END    (void*)0x27FFFFFF

struct {
	char* RXBaseAddress[6];
	char* RXTopAddress[6];
	uint32_t RXMaxFrameSize[6];
	uint32_t RXTimeStampSize[6];
	uint32_t RXFrameLengthSize[6];
	char* RXNextFrameAddress[6];
} UART_RXinfo;

typedef struct {
	alt_u64 timestamp;
	alt_u16 framelength;
} UART_RXFrameInfo;

// timestamp based on the 64-bit TIMER_0 = 8 bytes
#define TIMESTAMP_SIZE sizeof(alt_u64)

// 2 bytes to store the frame length actually received
#define FRAME_LENGTH_SIZE sizeof(alt_u16)

// Address in DDR3 where to push the RXINFO data
#define RAMDEST_UART_RXINFO     (void*)0x38000000

unsigned int channelRXBaudRate[6];
unsigned int channelRXGapDetectionChars[6];

int main() {


	alt_timestamp_start();

	alt_u32 divisor = alt_timestamp_freq()/1000000; // to get time in µsec
	alt_u64 time;
	alt_u64 lastactivitytime[6];
	alt_u64 time_start, time_stop;

	int uart_fd[6];
	uart_fd[1] = open(FIFOED_AVALON_UART_1_NAME, O_RDWR | O_NONBLOCK);



	bool msg_received;


	UART_RXinfo.RXBaseAddress[1] = RAMDEST_UART1_RX_START;
	UART_RXinfo.RXTopAddress[1] = RAMDEST_UART1_RX_END;
	UART_RXinfo.RXMaxFrameSize[1] = FIFOED_AVALON_UART_1_RX_FIFO_SIZE;
	UART_RXinfo.RXTimeStampSize[1] = TIMESTAMP_SIZE;
	UART_RXinfo.RXNextFrameAddress[1] = RAMDEST_UART1_RX_START;
	UART_RXinfo.RXFrameLengthSize[1] = FRAME_LENGTH_SIZE;
	channelRXBaudRate[1] = FIFOED_AVALON_UART_1_BAUD;
	channelRXGapDetectionChars[1] = FIFOED_AVALON_UART_1_GAP_VALUE;


	// Just a little LED animation to indicate that NIOS processor is alive and running
	for(int k=0;k<5;k++)
	{
		for (int i=0; i<6;i++)
		{
			enable_led(true);
			usleep(25000);
			enable_led(false);
		}

		for (int i=4; i>=0;i--)
		{
			enable_led(true);
			usleep(25000);
			enable_led(false);
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////
	// TODO : program a scatter/gather transfer to do a single DMA for all frames & their info ?
	///////////////////////////////////////////////////////////////////////////////////////////

	while (true) {

		usleep(25000);
	}


	return 0;
}


