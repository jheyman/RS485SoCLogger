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

int uart_getRXFifoLevel(void* base)
{
	int level;

	level = IORD_FIFOED_AVALON_UART_RX_FIFO_USED(base);

	return level;
}
/*
void setup_leds() {
    // Switch on first LED only
    IOWR_ALTERA_AVALON_PIO_DATA(NIOS_ACTIVITY_LEDS_BASE, 0x1);
}

void handle_leds() {
    uint32_t leds_mask = IORD_ALTERA_AVALON_PIO_DATA(NIOS_ACTIVITY_LEDS_BASE);

    if (leds_mask != (0x01 << (NIOS_ACTIVITY_LEDS_DATA_WIDTH - 1))) {
        // rotate leds
        leds_mask <<= 1;
    } else {
        // reset leds
        leds_mask = 0x1;
    }

    IOWR_ALTERA_AVALON_PIO_DATA(NIOS_ACTIVITY_LEDS_BASE, leds_mask);
}
*/
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
	uint32_t* RXBaseAddress[6];
	uint32_t RXFrameSize[6];
	uint32_t* RXNextFrameAddress[6];
} UART_RXinfo;

#define RAMDEST_UART_RXINFO       (void*)0x38000000


int main() {

	unsigned long loop_index=0;

	alt_msgdma_dev *MSGDMADev;
	alt_msgdma_standard_descriptor MSGDMA_DESC;

    printf("DE0-Nano-SoC nios demo %s %s\n", __DATE__, __TIME__);

    /*setup_leds();*/

    alt_timestamp_start();
    alt_u32 divisor = alt_timestamp_freq()/1000000; // to get time in µsec
    alt_u32 time;
    alt_u32 lastactivitytime[6];
    alt_u32 time_start, time_stop;

    int fp_0, fp_1,fp_2, fp_3, fp_4, fp_5;
    fp_0 = open(FIFOED_AVALON_UART_0_NAME, O_RDWR | O_NONBLOCK);
    fp_1 = open(FIFOED_AVALON_UART_1_NAME, O_RDWR | O_NONBLOCK);
    fp_2 = open(FIFOED_AVALON_UART_2_NAME, O_RDWR | O_NONBLOCK);
    fp_3 = open(FIFOED_AVALON_UART_3_NAME, O_RDWR | O_NONBLOCK);
    fp_4 = open(FIFOED_AVALON_UART_4_NAME, O_RDWR | O_NONBLOCK);
    fp_5 = open(FIFOED_AVALON_UART_5_NAME, O_RDWR | O_NONBLOCK);

    MSGDMADev = alt_msgdma_open(MSGDMA_0_CSR_NAME);
	if (MSGDMADev == NULL) printf("Could not open mSGDMA\n");

    char RX[4096] __attribute__ ((aligned (32)));
	//char* RX = malloc(4096);
	if (RX==NULL) printf("RX malloc error \n");

    printf("RX address: 0x%X\n", (int)RX);

    void* RAMDest_UART0 = RAMDEST_UART0_RX_START;
    void* RAMDest_UART1 = RAMDEST_UART1_RX_START;
    void* RAMDest_UART2 = RAMDEST_UART2_RX_START;
    void* RAMDest_UART3 = RAMDEST_UART3_RX_START;
    void* RAMDest_UART4 = RAMDEST_UART4_RX_START;
    void* RAMDest_UART5 = RAMDEST_UART5_RX_START;

    bool msg_received;

    UART_RXinfo.RXBaseAddress[0] = RAMDEST_UART0_RX_START;
    UART_RXinfo.RXFrameSize[0] = 4096;
    UART_RXinfo.RXNextFrameAddress[0] = RAMDEST_UART0_RX_START;

    UART_RXinfo.RXBaseAddress[1] = RAMDEST_UART1_RX_START;
    UART_RXinfo.RXFrameSize[1] = 4096;
    UART_RXinfo.RXNextFrameAddress[1] = RAMDEST_UART1_RX_START;

    UART_RXinfo.RXBaseAddress[2] = RAMDEST_UART2_RX_START;
    UART_RXinfo.RXFrameSize[2] = 4096;
    UART_RXinfo.RXNextFrameAddress[2] = RAMDEST_UART2_RX_START;

    UART_RXinfo.RXBaseAddress[3] = RAMDEST_UART3_RX_START;
    UART_RXinfo.RXFrameSize[3] = 4096;
    UART_RXinfo.RXNextFrameAddress[3] = RAMDEST_UART3_RX_START;

    UART_RXinfo.RXBaseAddress[4] = RAMDEST_UART4_RX_START;
    UART_RXinfo.RXFrameSize[4] = 4096;
    UART_RXinfo.RXNextFrameAddress[4] = RAMDEST_UART4_RX_START;

    UART_RXinfo.RXBaseAddress[5] = RAMDEST_UART5_RX_START;
    UART_RXinfo.RXFrameSize[5] = 4096;
    UART_RXinfo.RXNextFrameAddress[5] = RAMDEST_UART5_RX_START;

    while (true) {

    	msg_received = false;

    	loop_index++;

    	if (loop_index % 3000 == 0)
    	{
    		//printf("loop:%ld\n", loop_index);
    		//printf("*\n");
    		//handle_leds();
    	}

		if (fp_0)
		{
        	time = alt_timestamp() / divisor;

        	// reset activity led
			if (time - lastactivitytime[0] > 100000)
			{
				enable_led(0, 0);
			}

			int nb_read=read(fp_0,RX,4096);

			if(nb_read==-1)
			{
				printf("ERRNO: %d\n", errno);
			}
			else if (nb_read != 0)
			{
				msg_received = true;
        		enable_led(0, 1);
				lastactivitytime[0] = time;

				printf("[%ld] NB read UART0: %d\n", loop_index, nb_read);
				for (int i=0; i<4; i++)
				{
					printf("Char %d: %u\n", i, RX[i]);
				}

				printf("Timestamp: %ld\n", time);

				int level;
				level = uart_getRXFifoLevel((void*)FIFOED_AVALON_UART_0_BASE);
				printf("FIFO level: %d\n", level);

				/*printf("Launching DMA...\n");*/

				printf("TEST Writing ...\n");

				time_start = alt_timestamp() / divisor;

				if (MSGDMADev != NULL)
				{
						int err;
						err= alt_msgdma_construct_standard_mm_to_mm_descriptor(
								MSGDMADev,
								&MSGDMA_DESC,
								(alt_u32*)(RX), /* read address */
								RAMDest_UART0, /* write address */
								4096, /* size */
								0);

						err = alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);

						if (err != 0)
						{
							printf("Async DMA err %d\n", err);
						}

						// Move destination pointer for next transfer.
						RAMDest_UART0 += 4096;
						// Rollover if needed
						if (RAMDest_UART0 > RAMDEST_UART0_RX_END) RAMDest_UART0 = RAMDEST_UART0_RX_START;
						// Update value for HPS
						UART_RXinfo.RXNextFrameAddress[0] = RAMDest_UART0;
				}
				else
					printf("ERROR: MSGDMADev is NULL!\n");

	     		time_stop = alt_timestamp() / divisor;

				printf("DONE Writing.\n");
				printf("Timestamp delta: %ld\n", time_stop-time_start);
			}
		}

        if (fp_1)
        {
        	time = alt_timestamp() / divisor;

        	// reset activity led
			if (time - lastactivitytime[1] > 100000)
			{
				enable_led(1, 0);
			}

        	int nb_read=read(fp_1,RX,4096);

        	if(nb_read==-1)
        	{
        	    printf("ERRNO: %d\n", errno);
        	}
        	else if (nb_read != 0)
        	{
				msg_received = true;
        		enable_led(1, 1);
				lastactivitytime[1] = time;

				printf("[%ld] NB read UART1: %d\n", loop_index,nb_read );
				for (int i=0; i<4; i++)
				{
					printf("Char %d: %u\n", i, RX[i]);
				}

				printf("Timestamp: %ld\n", time);

				int level;
				level = uart_getRXFifoLevel((void*)FIFOED_AVALON_UART_1_BASE);
				printf("FIFO level: %d\n", level);

				if (MSGDMADev != NULL)
				{

					int err;
					err= alt_msgdma_construct_standard_mm_to_mm_descriptor(
							MSGDMADev,
							&MSGDMA_DESC,
							(alt_u32*)(RX), /* read address */
							RAMDest_UART1,
							4096, /* size */
							/*ALTERA_MSGDMA_DESCRIPTOR_CONTROL_PARK_WRITES_MASK*/0);

					/*printf("Async DMA DESC : err %d\n", err);*/

					err = alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);

					if (err != 0)
					{
						printf("Async DMA err %d\n", err);
					}

					// Move destination pointer for next transfer.
					RAMDest_UART1 += 4096;
					// Rollover if needed
					if (RAMDest_UART1 > RAMDEST_UART1_RX_END) RAMDest_UART1 = RAMDEST_UART1_RX_START;

					// Update value for HPS
					UART_RXinfo.RXNextFrameAddress[1] = RAMDest_UART1;

				}
        	}
        }

        if (fp_2)
        {
        	time = alt_timestamp() / divisor;

        	// reset activity led
			if (time - lastactivitytime[2] > 100000)
			{
				enable_led(2, 0);
			}

        	int nb_read=read(fp_2,RX,4096);

        	if(nb_read==-1)
        	{
        	    printf("ERRNO: %d\n", errno);
        	}
        	else if (nb_read != 0)
        	{
				msg_received = true;
        		enable_led(2, 1);
				lastactivitytime[2] = time;

				printf("[%ld] NB read UART2: %d\n", loop_index, nb_read);
				for (int i=0; i<4; i++)
				{
					printf("Char %d: %u\n", i, RX[i]);
				}

				printf("Timestamp: %ld\n", time);

				int level;
				level = uart_getRXFifoLevel((void*)FIFOED_AVALON_UART_2_BASE);
				printf("FIFO level: %d\n", level);

				if (MSGDMADev != NULL)
				{

					int err;
					err= alt_msgdma_construct_standard_mm_to_mm_descriptor(
							MSGDMADev,
							&MSGDMA_DESC,
							(alt_u32*)(RX), /* read address */
							RAMDest_UART2,
							4096, /* size */
							/*ALTERA_MSGDMA_DESCRIPTOR_CONTROL_PARK_WRITES_MASK*/0);

					/*printf("Async DMA DESC : err %d\n", err);*/

					err = alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);

					if (err != 0)
					{
						printf("Async DMA err %d\n", err);
					}

					// Move destination pointer for next transfer.
					RAMDest_UART2 += 4096;
					// Rollover if needed
					if (RAMDest_UART2 > RAMDEST_UART2_RX_END) RAMDest_UART2 = RAMDEST_UART2_RX_START;

					// Update value for HPS
					UART_RXinfo.RXNextFrameAddress[2] = RAMDest_UART2;
				}
        	}
        }

        if (fp_3)
        {
        	time = alt_timestamp() / divisor;

        	// reset activity led
			if (time - lastactivitytime[3] > 100000)
			{
				enable_led(3, 0);
			}

        	int nb_read=read(fp_3,RX,4096);

        	if(nb_read==-1)
        	{
        	    printf("ERRNO: %d\n", errno);
        	}
        	else if (nb_read != 0)
        	{
				msg_received = true;
        		enable_led(3, 1);
				lastactivitytime[3] = time;

				printf("[%ld] NB read UART3: %d\n", loop_index, nb_read);
				for (int i=0; i<4; i++)
				{
					printf("Char %d: %u\n", i, RX[i]);
				}

				printf("Timestamp: %ld\n", time);

				int level;
				level = uart_getRXFifoLevel((void*)FIFOED_AVALON_UART_3_BASE);
				printf("FIFO level: %d\n", level);

				if (MSGDMADev != NULL)
				{
					int err;
					err= alt_msgdma_construct_standard_mm_to_mm_descriptor(
							MSGDMADev,
							&MSGDMA_DESC,
							(alt_u32*)(RX), /* read address */
							RAMDest_UART3,
							4096, /* size */
							/*ALTERA_MSGDMA_DESCRIPTOR_CONTROL_PARK_WRITES_MASK*/0);

					err = alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);

					if (err != 0)
					{
						printf("Async DMA err %d\n", err);
					}

					// Move destination pointer for next transfer.
					RAMDest_UART3 += 4096;
					// Rollover if needed
					if (RAMDest_UART3 > RAMDEST_UART3_RX_END) RAMDest_UART3 = RAMDEST_UART3_RX_START;

					// Update value for HPS
					UART_RXinfo.RXNextFrameAddress[3] = RAMDest_UART3;
				}
        	}
        }

        if (fp_4)
        {
        	time = alt_timestamp() / divisor;

        	// reset activity led
			if (time - lastactivitytime[4] > 100000)
			{
				enable_led(4, 0);
			}

        	int nb_read=read(fp_4,RX,4096);

        	if(nb_read==-1)
        	{
        	    printf("ERRNO: %d\n", errno);
        	}
        	else if (nb_read != 0)
        	{
				msg_received = true;
        		enable_led(4, 1);
				lastactivitytime[4] = time;

        		printf("[%ld] NB read UART4: %d\n", loop_index, nb_read);
				for (int i=0; i<4; i++)
				{
					printf("Char %d: %u\n", i, RX[i]);
				}


				printf("Timestamp: %ld\n", time);

				int level;
				level = uart_getRXFifoLevel((void*)FIFOED_AVALON_UART_4_BASE);
				printf("FIFO level: %d\n", level);

				if (MSGDMADev != NULL)
				{

					int err;
					err= alt_msgdma_construct_standard_mm_to_mm_descriptor(
							MSGDMADev,
							&MSGDMA_DESC,
							(alt_u32*)(RX), /* read address */
							RAMDest_UART4,
							4096, /* size */
							/*ALTERA_MSGDMA_DESCRIPTOR_CONTROL_PARK_WRITES_MASK*/0);

					/*printf("Async DMA DESC : err %d\n", err);*/

					err = alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);

					if (err != 0)
					{
						printf("Async DMA err %d\n", err);
					}

					// Move destination pointer for next transfer.
					RAMDest_UART4 += 4096;
					// Rollover if needed
					if (RAMDest_UART4 > RAMDEST_UART4_RX_END) RAMDest_UART4 = RAMDEST_UART4_RX_START;

					// Update value for HPS
					UART_RXinfo.RXNextFrameAddress[4] = RAMDest_UART4;

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

        if (fp_5)
        {
        	time = alt_timestamp() / divisor;

        	// reset activity led
			if (time - lastactivitytime[5] > 100000)
			{
				enable_led(5, 0);
			}

        	int nb_read=read(fp_5,RX,4096);

        	if(nb_read==-1)
        	{
        	    printf("ERRNO: %d\n", errno);
        	}
        	else if (nb_read != 0)
        	{
				msg_received = true;
        		enable_led(5, 1);
				lastactivitytime[5] = time;

				printf("[%ld] NB read UART5: %d\n", loop_index, nb_read);
				for (int i=0; i<4; i++)
				{
					printf("Char %d: %u\n", i, RX[i]);
				}

				printf("Timestamp: %ld\n", time);

				int level;
				level = uart_getRXFifoLevel((void*)FIFOED_AVALON_UART_5_BASE);
				printf("FIFO level: %d\n", level);
				if (MSGDMADev != NULL)
				{

					int err;
					err= alt_msgdma_construct_standard_mm_to_mm_descriptor(
							MSGDMADev,
							&MSGDMA_DESC,
							(alt_u32*)(RX), /* read address */
							RAMDest_UART5,
							4096, /* size */
							/*ALTERA_MSGDMA_DESCRIPTOR_CONTROL_PARK_WRITES_MASK*/0);

					/*printf("Async DMA DESC : err %d\n", err);*/

					err = alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);

					if (err != 0)
					{
						printf("Async DMA err %d\n", err);
					}

					// Move destination pointer for next transfer.
					RAMDest_UART5 += 4096;
					// Rollover if needed
					if (RAMDest_UART5> RAMDEST_UART5_RX_END) RAMDest_UART5 = RAMDEST_UART5_RX_START;

					// Update value for HPS
					UART_RXinfo.RXNextFrameAddress[5] = RAMDest_UART5;
				}
        	}
        }


        // Refresh UART info struct in DDR
        if (msg_received && MSGDMADev != NULL)
        {
        	printf("Refreshing UART INFO in DDR\n");

			int err;
			err= alt_msgdma_construct_standard_mm_to_mm_descriptor(
					MSGDMADev,
					&MSGDMA_DESC,
					(alt_u32*)(&UART_RXinfo), /* read address */
					RAMDEST_UART_RXINFO,/* write address */
					sizeof(UART_RXinfo),
					0);

			err = alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);

			if (err != 0)
			{
				printf("Async DMA err %d\n", err);
			}
        }
    }

	//fclose (fp);
    close (fp_0);
    close (fp_1);
    close (fp_2);
    close (fp_3);
    close (fp_4);
    close (fp_5);

    return 0;
}
