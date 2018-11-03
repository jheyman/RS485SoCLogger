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
#include "altera_msgdma_descriptor_regs.h"
#include "altera_msgdma_prefetcher_regs.h"
#include "altera_msgdma_csr_regs.h"
#include "altera_msgdma.h"

#include "system.h"

#define SLEEP_DELAY_US (1000 * 1000)

int uart_getRXFifoLevel(int channel)
{
	int level;

	switch(channel) {
	case 0:
		level = IORD_FIFOED_AVALON_UART_RX_FIFO_USED(FIFOED_AVALON_UART_0_BASE);
		break;
	case 1:
		level = IORD_FIFOED_AVALON_UART_RX_FIFO_USED(FIFOED_AVALON_UART_1_BASE);
		break;
	case 2:
		level = IORD_FIFOED_AVALON_UART_RX_FIFO_USED(FIFOED_AVALON_UART_2_BASE);
		break;
	case 3:
		level = IORD_FIFOED_AVALON_UART_RX_FIFO_USED(FIFOED_AVALON_UART_3_BASE);
		break;
	case 4:
		level = IORD_FIFOED_AVALON_UART_RX_FIFO_USED(FIFOED_AVALON_UART_4_BASE);
		break;
	case 5:
		level = IORD_FIFOED_AVALON_UART_RX_FIFO_USED(FIFOED_AVALON_UART_5_BASE);
		break;
	default:
			break;
	}


	return level;
}

void tx_enable_UART0(bool activate) {

    if(activate)
    	IOWR_ALTERA_AVALON_PIO_DATA(FIFOED_AVALON_UART_0_TXENABLE_BASE, 0x1);
    else
    	IOWR_ALTERA_AVALON_PIO_DATA(FIFOED_AVALON_UART_0_TXENABLE_BASE, 0x0);
}

void enable_led(int channel, bool onoff) {
    uint32_t leds = IORD_ALTERA_AVALON_PIO_DATA(NIOS_ACTIVITY_LEDS_BASE);

    if (onoff)
    	leds |= (0x01 << channel);
    else
    	leds &= ~(0x01 << channel);

    IOWR_ALTERA_AVALON_PIO_DATA(NIOS_ACTIVITY_LEDS_BASE, leds);
}

#define RAMDEST_UART0_RX_START  (void*)0x20000000
#define RAMDEST_UART0_RX_END    (void*)0x21FFFFFF
#define RAMDEST_UART0_TX_START  (void*)0x22000000
#define RAMDEST_UART0_TX_END    (void*)0x23FFFFFF

#define RAMDEST_UART1_RX_START  (void*)0x24000000
#define RAMDEST_UART1_RX_END    (void*)0x25FFFFFF
#define RAMDEST_UART1_TX_START  (void*)0x26000000
#define RAMDEST_UART1_TX_END    (void*)0x27FFFFFF

#define RAMDEST_UART2_RX_START  (void*)0x28000000
#define RAMDEST_UART2_RX_END    (void*)0x29FFFFFF
#define RAMDEST_UART2_TX_START  (void*)0x2A000000
#define RAMDEST_UART2_TX_END    (void*)0x2BFFFFFF

#define RAMDEST_UART3_RX_START  (void*)0x2C000000
#define RAMDEST_UART3_RX_END    (void*)0x2DFFFFFF
#define RAMDEST_UART3_TX_START  (void*)0x2E000000
#define RAMDEST_UART3_TX_END    (void*)0x2FFFFFFF

#define RAMDEST_UART4_RX_START  (void*)0x30000000
#define RAMDEST_UART4_RX_END    (void*)0x31FFFFFF
#define RAMDEST_UART4_TX_START  (void*)0x32000000
#define RAMDEST_UART4_TX_END    (void*)0x33FFFFFF

#define RAMDEST_UART5_RX_START  (void*)0x34000000
#define RAMDEST_UART5_RX_END    (void*)0x35FFFFFF
#define RAMDEST_UART5_TX_START  (void*)0x36000000
#define RAMDEST_UART5_TX_END    (void*)0x37FFFFFF

struct {
	char* RXBaseAddress[6];
	char* RXTopAddress[6];
	uint32_t RXFrameSize[6];
	char* RXNextFrameAddress[6];
} UART_RXinfo;

#define RAMDEST_UART_RXINFO     (void*)0x38000000

int main() {
	unsigned long loop_index=0;

	alt_msgdma_dev *MSGDMADev;
	alt_msgdma_standard_descriptor MSGDMA_DESC;

    printf("DE0-Nano-SoC nios demo %s %s\n", __DATE__, __TIME__);

    alt_timestamp_start();
    alt_u32 divisor = alt_timestamp_freq()/1000000; // to get time in µsec
    alt_u32 time;
    alt_u32 lastactivitytime[6];
    alt_u32 time_start, time_stop;

    int uart_fd[6];
    uart_fd[0] = open(FIFOED_AVALON_UART_0_NAME, O_RDWR | O_NONBLOCK);
    uart_fd[1] = open(FIFOED_AVALON_UART_1_NAME, O_RDWR | O_NONBLOCK);
    uart_fd[2] = open(FIFOED_AVALON_UART_2_NAME, O_RDWR | O_NONBLOCK);
    uart_fd[3] = open(FIFOED_AVALON_UART_3_NAME, O_RDWR | O_NONBLOCK);
    uart_fd[4] = open(FIFOED_AVALON_UART_4_NAME, O_RDWR | O_NONBLOCK);
    uart_fd[5] = open(FIFOED_AVALON_UART_5_NAME, O_RDWR | O_NONBLOCK);

    MSGDMADev = alt_msgdma_open(MSGDMA_0_CSR_NAME);
	if (MSGDMADev == NULL) printf("Could not open mSGDMA\n");

    char RX[4096] __attribute__ ((aligned (32)));
	//char* RX = malloc(4096);
	if (RX==NULL) printf("RX malloc error \n");

    bool msg_received;

    UART_RXinfo.RXBaseAddress[0] = RAMDEST_UART0_RX_START;
    UART_RXinfo.RXTopAddress[0] = RAMDEST_UART0_RX_END;
    UART_RXinfo.RXFrameSize[0] = 4096;
    UART_RXinfo.RXNextFrameAddress[0] = RAMDEST_UART0_RX_START;

    UART_RXinfo.RXBaseAddress[1] = RAMDEST_UART1_RX_START;
    UART_RXinfo.RXTopAddress[1] = RAMDEST_UART1_RX_END;
    UART_RXinfo.RXFrameSize[1] = 4096;
    UART_RXinfo.RXNextFrameAddress[1] = RAMDEST_UART1_RX_START;

    UART_RXinfo.RXBaseAddress[2] = RAMDEST_UART2_RX_START;
    UART_RXinfo.RXTopAddress[2] = RAMDEST_UART2_RX_END;
    UART_RXinfo.RXFrameSize[2] = 4096;
    UART_RXinfo.RXNextFrameAddress[2] = RAMDEST_UART2_RX_START;

    UART_RXinfo.RXBaseAddress[3] = RAMDEST_UART3_RX_START;
    UART_RXinfo.RXTopAddress[3] = RAMDEST_UART3_RX_END;
    UART_RXinfo.RXFrameSize[3] = 4096;
    UART_RXinfo.RXNextFrameAddress[3] = RAMDEST_UART3_RX_START;

    UART_RXinfo.RXBaseAddress[4] = RAMDEST_UART4_RX_START;
    UART_RXinfo.RXTopAddress[4] = RAMDEST_UART4_RX_END;
    UART_RXinfo.RXFrameSize[4] = 4096;
    UART_RXinfo.RXNextFrameAddress[4] = RAMDEST_UART4_RX_START;

    UART_RXinfo.RXBaseAddress[5] = RAMDEST_UART5_RX_START;
    UART_RXinfo.RXTopAddress[5] = RAMDEST_UART5_RX_END;
    UART_RXinfo.RXFrameSize[5] = 4096;
    UART_RXinfo.RXNextFrameAddress[5] = RAMDEST_UART5_RX_START;

    // Just a little LED animation to indicate that NIOS processor is alive and running
    for(int k=0;k<5;k++)
    {
    	for (int i=0; i<6;i++)
    	{
    		enable_led(i, true);
    		usleep(25000);
    		enable_led(i, false);
    	}

    	for (int i=4; i>=0;i--)
    	{
    		enable_led(i, true);
    		usleep(25000);
    		enable_led(i, false);
    	}
    }

    while (true) {

    	msg_received = false;

    	loop_index++;

    	/*
    	if (loop_index % 3000 == 0)
    	{
    		//printf("loop:%ld\n", loop_index);
    		//printf("*\n");
    	}
    	*/

    	// Scan all UART FIFOs for data
    	for (int uart_index=0; uart_index<6; uart_index++)
    	{
    		if (uart_fd[uart_index])
    		{
    			time = alt_timestamp() / divisor;

    			// reset activity led
    			if (time - lastactivitytime[uart_index] > 100000)
    			{
    				enable_led(0, 0);
    			}

    			int nb_read=read(uart_fd[uart_index],RX,4096);

    			if(nb_read==-1)
    			{
    				printf("ERRNO: %d\n", errno);
    			}
    			else if (nb_read != 0)
    			{
    				msg_received = true;
    				enable_led(0, 1);
    				lastactivitytime[uart_index] = time;

    				printf("[%ld] NB read UART%d: %d\n", loop_index, uart_index, nb_read);
    				for (int i=0; i<4; i++)
    				{
    					printf("Char %d: %u\n", i, RX[i]);
    				}

    				printf("Timestamp: %ld\n", time);

    				int level;
    				level = uart_getRXFifoLevel(uart_index);
    				printf("FIFO level: %d\n", level);

    				time_start = alt_timestamp() / divisor;

    				if (MSGDMADev != NULL)
    				{
    					int err;
    					err= alt_msgdma_construct_standard_mm_to_mm_descriptor(
    							MSGDMADev,
								&MSGDMA_DESC,
								(alt_u32*)RX, /* read address */
								(alt_u32*)UART_RXinfo.RXNextFrameAddress[uart_index], /* write address */
								4096, /* size in bytes */
								0);

    					err = alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);

    					if (err != 0)
    					{
    						printf("Async DMA err %d\n", err);
    					}

    					// Move destination pointer for next transfer.
    					UART_RXinfo.RXNextFrameAddress[uart_index] += 4096;

    					// Rollover if needed
    					if (UART_RXinfo.RXNextFrameAddress[uart_index] > UART_RXinfo.RXTopAddress[uart_index])
    						UART_RXinfo.RXNextFrameAddress[uart_index] = UART_RXinfo.RXBaseAddress[uart_index];
    				}
    				else
    					printf("ERROR: MSGDMADev is NULL!\n");

    				time_stop = alt_timestamp() / divisor;

    				printf("DONE Writing.\n");
    				printf("Timestamp delta: %ld\n", time_stop-time_start);

    				/*
				printf("Sending data back over UART0...\n");
				tx_enable_UART0(true);
				int nb_written=write(fp_0,RX,32);
				tx_enable_UART0(false);
				printf("DONE, wrote %d chars\n", nb_written);
    				 */
    			}
    		}
    	}

        // If anything was received on any channel, refresh UART info struct in DDR for HPS to catch up
        if (msg_received && MSGDMADev != NULL)
        {
        	printf("Refreshing UART INFO in DDR\n");

			int err;
			err= alt_msgdma_construct_standard_mm_to_mm_descriptor(
					MSGDMADev,
					&MSGDMA_DESC,
					(alt_u32*)(&UART_RXinfo), /* read address  */
					RAMDEST_UART_RXINFO,      /* write address */
					sizeof(UART_RXinfo),      /* size in bytes */
					0);

			err = alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);

			if (err != 0)
			{
				printf("Async DMA err %d\n", err);
			}
        }
    }


    for (int i=0; i<6; i++)
    {
    	close(uart_fd[i]);
    }

    return 0;
}
