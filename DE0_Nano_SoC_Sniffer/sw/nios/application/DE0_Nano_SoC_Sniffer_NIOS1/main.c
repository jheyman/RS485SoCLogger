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
#include "altera_msgdma_descriptor_regs.h"
#include "altera_msgdma_prefetcher_regs.h"
#include "altera_msgdma_csr_regs.h"
#include "altera_msgdma.h"

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
		led |= 0x01 ;
	else
		led &= 0x00;

	IOWR_ALTERA_AVALON_PIO_DATA(NIOS_ACTIVITY_LED_1_BASE, led);
}

#define RAMDEST_UART1_RX_START  (void*)0x24000000
#define RAMDEST_UART1_RX_END    (void*)0x25FFFFFF
#define RAMDEST_UART1_TX_START  (void*)0x26000000
#define RAMDEST_UART1_TX_END    (void*)0x27FFFFFF

struct {
	char* RXBaseAddress;
	char* RXTopAddress;
	uint32_t RXMaxFrameSize;
	uint32_t RXTimeStampSize;
	uint32_t RXFrameLengthSize;
	char* RXNextFrameAddress;
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
#define RAMDEST_UART_RXINFO_UART1     (void*)0x38001000

unsigned int channelRXBaudRate;
unsigned int channelRXGapDetectionChars;

int main() {
	alt_msgdma_dev *MSGDMADev;
	alt_msgdma_standard_descriptor MSGDMA_DESC;

	//printf("DE0-Nano-SoC Sniffer %s %s\n", __DATE__, __TIME__);

	alt_timestamp_start();

	alt_u32 divisor = alt_timestamp_freq()/1000000; // to get time in µsec
	alt_u64 time;
	alt_u64 lastactivitytime;
	alt_u64 time_start, time_stop;

	int uart_fd;
	uart_fd = open(FIFOED_AVALON_UART_1_NAME, O_RDWR | O_NONBLOCK);

	MSGDMADev = alt_msgdma_open(MSGDMA_1_CSR_NAME);
	if (MSGDMADev == NULL) printf("Could not open mSGDMA\n");

	bool msg_received;

	UART_RXinfo.RXBaseAddress = RAMDEST_UART1_RX_START;
	UART_RXinfo.RXTopAddress = RAMDEST_UART1_RX_END;
	UART_RXinfo.RXMaxFrameSize = FIFOED_AVALON_UART_1_RX_FIFO_SIZE;
	UART_RXinfo.RXTimeStampSize = TIMESTAMP_SIZE;
	UART_RXinfo.RXFrameLengthSize = FRAME_LENGTH_SIZE;
	UART_RXinfo.RXNextFrameAddress = RAMDEST_UART1_RX_START;
	channelRXBaudRate = FIFOED_AVALON_UART_1_BAUD;
	channelRXGapDetectionChars = FIFOED_AVALON_UART_1_GAP_VALUE;

	//printf("Initializing UART INFO in HPS memory\n");

	int err;
	alt_msgdma_construct_standard_mm_to_mm_descriptor(
			MSGDMADev,
			&MSGDMA_DESC,
			(alt_u32*)(&UART_RXinfo), /* read address  */
			RAMDEST_UART_RXINFO_UART1,      /* write address */
			sizeof(UART_RXinfo),      /* size in bytes */
			0);

	err = alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);

	if (err != 0)
	{
		//printf("Initial UART INFO xfer DMA err %d\n", err);
	}

	// Just a little LED animation to indicate that NIOS processor is alive and running
	for(int k=0;k<10;k++)
	{
		enable_led(true);
		usleep(100000);
		enable_led(false);
		usleep(100000);
	}

	///////////////////////////////////////////////////////////////////////////////////////////
	// TODO : program a scatter/gather transfer to do a single DMA for all frames & their info ?
	///////////////////////////////////////////////////////////////////////////////////////////

	unsigned int nb_read;

	while (true) {

		msg_received = false;

			if (uart_fd)
			{
				time = alt_timestamp() / divisor;

				// turn off activity led 100ms after last activity on this channel
				if (time - lastactivitytime > 100000)
				{
					enable_led(0);
				}

				fifoed_avalon_uart_snaphot tmp;

				// Proceed to read the channel. The call is NOT-blocking, so as to scan all channels continuously,
				// and only grab data when it is present in the FIFO.
				nb_read=read(uart_fd,&tmp,0);

				if (nb_read != 0)
				{
					time_start = alt_timestamp();

					// keep track that a message was received (on any channel), to remember to push the updated RXinfo data to HPS
					msg_received = true;

					// (re)activate the led corresponding to the channel, and keep track of the activation time.
					enable_led(1);
					lastactivitytime = time;

					// DEBUG
					//printf("NB read UART: %d frames read\n", nb_read);

					fifoed_avalon_uart_state* state = tmp.ptr;
					unsigned short frameIndex = tmp.rx_framestart_index;

					UART_RXFrameInfo frameInfo[MAX_NB_FRAMES_BUFFERED];

					for (unsigned int i=0; i < tmp.rx_framecount; i++)
					{
						//DEBUG
						//printf("Dealing with frameIndex=%u\n", frameIndex);

						alt_u32 frameLength = state->rx_frame_size[frameIndex];

						// DEBUG
						//printf ("Frame %u: length=%lu\n", i, frameLength);

						/*
						for (int kk=0; kk<16;kk++)
						{
							printf("Ox%X ", state->rx_buf[state->rx_framestart_offset[frameIndex]+kk]);
						}
						printf("\n");
						*/

						// Push the received data to HPS DDR3 memory using DMA
						if (MSGDMADev != NULL)
						{
							///////////////////////////////////////////////
							// Prepare & execute DMA transfer of frame data
							///////////////////////////////////////////////

							/////////////
							// two cases:
							// - either the frame bytes are contiguous i.e. no buffer rollover occurred during storing => a single DMA is necessary
							// - or there was a buffer rollover during frame storage => in this case, perform two transfers:
							// 		- first transfer the part from start of frame to end of buffer
							//      - then transfer the part from start of buffer to end of frame

							alt_u16 startIndex = state->rx_framestart_offset[frameIndex];
							alt_u32* frameStartAddress_1stPart;
							alt_u32 frameLength_1stPart ;
							alt_u32* frameStartAddress_2ndPart;
							alt_u32 frameLength_2ndPart;

							frameStartAddress_1stPart = (alt_u32*)(&state->rx_buf[startIndex]);

							if (startIndex + frameLength <= FIFOED_AVALON_UART_BUF_LEN)
							{
								frameLength_1stPart = frameLength;
								frameStartAddress_2ndPart = 0;
								frameLength_2ndPart = 0;
							}
							else
							{
								frameLength_1stPart= (FIFOED_AVALON_UART_BUF_LEN - startIndex);
								frameStartAddress_2ndPart= (alt_u32*)(state->rx_buf);
								frameLength_2ndPart=frameLength -frameLength_1stPart;
							}

							//deBUG
							//printf("Frame %u: frameStartAddress_1stPart=%p\n", i,frameStartAddress_1stPart);
							//printf("Frame %u: frameLength_1stPart=%lu\n", i,frameLength_1stPart);
							//printf("Frame %u: frameStartAddress_2ndPart=%p\n", i,frameStartAddress_2ndPart);
							//printf("Frame %u: frameLength_2ndPart=%lu\n", i,frameLength_2ndPart);

							// Transfer 1st part
							int err;
							alt_msgdma_construct_standard_mm_to_mm_descriptor(
									MSGDMADev,
									&MSGDMA_DESC,
									frameStartAddress_1stPart, /* read address */
									(alt_u32*)(UART_RXinfo.RXNextFrameAddress), /* write address */
									frameLength_1stPart , /* size in bytes of data*/
									0);
							//time_start = alt_timestamp();
							err = alt_msgdma_standard_descriptor_sync_transfer(MSGDMADev, &MSGDMA_DESC);
							//time_stop = alt_timestamp();

							if (err != 0)
							{
								// DEBUG
								//printf("Frame data 1st part DMA err %d\n", err);
							}

							// Transfer 2nd part (if applicable)
							if (frameLength_2ndPart !=0)
							{
								alt_msgdma_construct_standard_mm_to_mm_descriptor(
										MSGDMADev,
										&MSGDMA_DESC,
										frameStartAddress_2ndPart, /* read address */
										(alt_u32*)(UART_RXinfo.RXNextFrameAddress+frameLength_1stPart), /* write address */
										frameLength_2ndPart , /* size in bytes of data*/
										0);

								err = alt_msgdma_standard_descriptor_sync_transfer(MSGDMADev, &MSGDMA_DESC);

								if (err != 0)
								{
									// DEBUG
									//printf("Frame data 2nd part DMA err %d\n", err);
								}
							}

							//////////////////////////////////////////////////
							// Prepare and transfer DMA transfer of Frame Info
							//////////////////////////////////////////////////
							alt_u32 divisor = alt_timestamp_freq()/1000000;
							frameInfo[i].timestamp = state->rx_timestamp[frameIndex]/divisor;

							// DEBUG
							//printf("Frame %u: Timestamp %llu\n", i , frameInfo[i].timestamp);
							//printf("Frame %u: DEBUGTimestamp %llu\n", i , state->rx_DEBUGtimestamp[frameIndex]/divisor - frameInfo[i].timestamp);


							frameInfo[i].framelength = frameLength;

							alt_msgdma_construct_standard_mm_to_mm_descriptor(
									MSGDMADev,
									&MSGDMA_DESC,
									(alt_u32*)&frameInfo[i], /* read address */
									(alt_u32*)(UART_RXinfo.RXNextFrameAddress+UART_RXinfo.RXMaxFrameSize), /* write address */
									TIMESTAMP_SIZE+FRAME_LENGTH_SIZE, /* size in bytes of info data*/
									0);

							err = alt_msgdma_standard_descriptor_sync_transfer(MSGDMADev, &MSGDMA_DESC);

							if (err != 0)
							{
								// DEBUG
								//printf("Frame info DMA err %d\n", err);
							}

							//printf("DONE Writing at address 0x%p\n", UART_RXinfo.RXNextFrameAddress[uart_index]);

							/////////////////////////////////////////////
							// Move destination pointer for next transfer.
							/////////////////////////////////////////////
							UART_RXinfo.RXNextFrameAddress += UART_RXinfo.RXMaxFrameSize + TIMESTAMP_SIZE + FRAME_LENGTH_SIZE;

							// Rollover if needed (when there is not enough space left to store one frame plus its timestamp and length
							if (UART_RXinfo.RXNextFrameAddress >= UART_RXinfo.RXTopAddress - UART_RXinfo.RXMaxFrameSize - TIMESTAMP_SIZE - FRAME_LENGTH_SIZE)
								UART_RXinfo.RXNextFrameAddress = UART_RXinfo.RXBaseAddress;

							//printf("Next frame will go at address 0x%p\n", UART_RXinfo.RXNextFrameAddress[uart_index]);
						}

						// move/rollback to next frame index
						frameIndex = (frameIndex+1) & MAX_NB_FRAMES_BUFFERED_MASK;
					}

					//time_stop = alt_timestamp();
				}

				/*
				printf("Sending data back over UART0...\n");
				tx_enable_UART0(true);
				int nb_written=write(fp_0,RX,32);
				tx_enable_UART0(false);
				printf("DONE, wrote %d chars\n", nb_written);
				 */
			}



		// If anything was received on any channel, refresh UART info struct in DDR for HPS to catch up
		if (msg_received && MSGDMADev != NULL)
		{

			int err;
			err= alt_msgdma_construct_standard_mm_to_mm_descriptor(
					MSGDMADev,
					&MSGDMA_DESC,
					(alt_u32*)(&UART_RXinfo), /* read address  */
					RAMDEST_UART_RXINFO_UART1,      /* write address */
					sizeof(UART_RXinfo),      /* size in bytes */
					0);

			if (err != 0)
			{
				//printf("DMA prep UARTRXInfo err %d\n", err);
			}

			err = alt_msgdma_standard_descriptor_sync_transfer(MSGDMADev, &MSGDMA_DESC);

			if (err != 0)
			{
				//printf("DMA run  UARTRXInfo err %d\n", err);
			}
			time_stop = alt_timestamp();

			// DEBUG
			//printf("Refreshing UART INFO in DDR; %d frames pushed\n", nb_read);
			//printf("UART_RXinfo.RXNextFrameAddress = %p\n", UART_RXinfo.RXNextFrameAddress);
			//printf("Timestamp delta: %lld\n", (time_stop-time_start)/divisor);
		}

		// DEBUG
		//usleep(1000000);

	}


	close(uart_fd);


	return 0;
}
