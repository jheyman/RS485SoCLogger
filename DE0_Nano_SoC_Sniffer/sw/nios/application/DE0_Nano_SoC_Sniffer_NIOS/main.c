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

// Assign 32MB for RX and 32MB for TX for each channel
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
	alt_msgdma_dev *MSGDMADev;
	alt_msgdma_standard_descriptor MSGDMA_DESC;

	printf("DE0-Nano-SoC Sniffer %s %s\n", __DATE__, __TIME__);

	alt_timestamp_start();

	alt_u32 divisor = alt_timestamp_freq()/1000000; // to get time in µsec
	alt_u64 time;
	alt_u64 lastactivitytime[6];
	alt_u64 time_start, time_stop;

	int uart_fd[6];
	uart_fd[0] = open(FIFOED_AVALON_UART_0_NAME, O_RDWR | O_NONBLOCK);
	uart_fd[1] = open(FIFOED_AVALON_UART_1_NAME, O_RDWR | O_NONBLOCK);
	uart_fd[2] = open(FIFOED_AVALON_UART_2_NAME, O_RDWR | O_NONBLOCK);
	uart_fd[3] = open(FIFOED_AVALON_UART_3_NAME, O_RDWR | O_NONBLOCK);
	uart_fd[4] = open(FIFOED_AVALON_UART_4_NAME, O_RDWR | O_NONBLOCK);
	uart_fd[5] = open(FIFOED_AVALON_UART_5_NAME, O_RDWR | O_NONBLOCK);

	MSGDMADev = alt_msgdma_open(MSGDMA_0_CSR_NAME);
	if (MSGDMADev == NULL) printf("Could not open mSGDMA\n");

	bool msg_received;

	UART_RXinfo.RXBaseAddress[0] = RAMDEST_UART0_RX_START;
	UART_RXinfo.RXTopAddress[0] = RAMDEST_UART0_RX_END;
	UART_RXinfo.RXMaxFrameSize[0] = FIFOED_AVALON_UART_0_RX_FIFO_SIZE;
	UART_RXinfo.RXTimeStampSize[0] = TIMESTAMP_SIZE;
	UART_RXinfo.RXFrameLengthSize[0] = FRAME_LENGTH_SIZE;
	UART_RXinfo.RXNextFrameAddress[0] = RAMDEST_UART0_RX_START;
	channelRXBaudRate[0] = FIFOED_AVALON_UART_0_BAUD;
	channelRXGapDetectionChars[0] = FIFOED_AVALON_UART_0_GAP_VALUE;

	UART_RXinfo.RXBaseAddress[1] = RAMDEST_UART1_RX_START;
	UART_RXinfo.RXTopAddress[1] = RAMDEST_UART1_RX_END;
	UART_RXinfo.RXMaxFrameSize[1] = FIFOED_AVALON_UART_1_RX_FIFO_SIZE;
	UART_RXinfo.RXTimeStampSize[1] = TIMESTAMP_SIZE;
	UART_RXinfo.RXNextFrameAddress[1] = RAMDEST_UART1_RX_START;
	UART_RXinfo.RXFrameLengthSize[1] = FRAME_LENGTH_SIZE;
	channelRXBaudRate[1] = FIFOED_AVALON_UART_1_BAUD;
	channelRXGapDetectionChars[1] = FIFOED_AVALON_UART_1_GAP_VALUE;

	UART_RXinfo.RXBaseAddress[2] = RAMDEST_UART2_RX_START;
	UART_RXinfo.RXTopAddress[2] = RAMDEST_UART2_RX_END;
	UART_RXinfo.RXMaxFrameSize[2] = FIFOED_AVALON_UART_2_RX_FIFO_SIZE;
	UART_RXinfo.RXTimeStampSize[2] = TIMESTAMP_SIZE;
	UART_RXinfo.RXNextFrameAddress[2] = RAMDEST_UART2_RX_START;
	UART_RXinfo.RXFrameLengthSize[2] = FRAME_LENGTH_SIZE;
	channelRXBaudRate[2] = FIFOED_AVALON_UART_2_BAUD;
	channelRXGapDetectionChars[2] = FIFOED_AVALON_UART_2_GAP_VALUE;

	UART_RXinfo.RXBaseAddress[3] = RAMDEST_UART3_RX_START;
	UART_RXinfo.RXTopAddress[3] = RAMDEST_UART3_RX_END;
	UART_RXinfo.RXMaxFrameSize[3] = FIFOED_AVALON_UART_3_RX_FIFO_SIZE;
	UART_RXinfo.RXTimeStampSize[3] = TIMESTAMP_SIZE;
	UART_RXinfo.RXNextFrameAddress[3] = RAMDEST_UART3_RX_START;
	UART_RXinfo.RXFrameLengthSize[3] = FRAME_LENGTH_SIZE;
	channelRXBaudRate[3] = FIFOED_AVALON_UART_3_BAUD;
	channelRXGapDetectionChars[3] = FIFOED_AVALON_UART_3_GAP_VALUE;

	UART_RXinfo.RXBaseAddress[4] = RAMDEST_UART4_RX_START;
	UART_RXinfo.RXTopAddress[4] = RAMDEST_UART4_RX_END;
	UART_RXinfo.RXMaxFrameSize[4] = FIFOED_AVALON_UART_4_RX_FIFO_SIZE;
	UART_RXinfo.RXTimeStampSize[4] = TIMESTAMP_SIZE;
	UART_RXinfo.RXNextFrameAddress[4] = RAMDEST_UART4_RX_START;
	UART_RXinfo.RXFrameLengthSize[4] = FRAME_LENGTH_SIZE;
	channelRXBaudRate[4] = FIFOED_AVALON_UART_4_BAUD;
	channelRXGapDetectionChars[4] = FIFOED_AVALON_UART_4_GAP_VALUE;

	UART_RXinfo.RXBaseAddress[5] = RAMDEST_UART5_RX_START;
	UART_RXinfo.RXTopAddress[5] = RAMDEST_UART5_RX_END;
	UART_RXinfo.RXMaxFrameSize[5] = FIFOED_AVALON_UART_5_RX_FIFO_SIZE;
	UART_RXinfo.RXTimeStampSize[5] = TIMESTAMP_SIZE;
	UART_RXinfo.RXNextFrameAddress[5] = RAMDEST_UART5_RX_START;
	UART_RXinfo.RXFrameLengthSize[5] = FRAME_LENGTH_SIZE;
	channelRXBaudRate[5] = FIFOED_AVALON_UART_5_BAUD;
	channelRXGapDetectionChars[5] = FIFOED_AVALON_UART_5_GAP_VALUE;

	printf("Initializing UART INFO in HPS memory\n");

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
		//printf("RXINFO xfer: Async DMA err %d\n", err);
	}

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

	///////////////////////////////////////////////////////////////////////////////////////////
	// TODO : program a scatter/gather transfer to do a single DMA for all frames & their info ?
	///////////////////////////////////////////////////////////////////////////////////////////

	while (true) {

		msg_received = false;

		time_start = alt_timestamp() / divisor;

		// Scan all UART FIFOs for data
		for (int uart_index=0; uart_index<6; uart_index++)
		{
			if (uart_fd[uart_index])
			{
				time = alt_timestamp() / divisor;

				// turn off activity led 100ms after last activity on this channel
				if (time - lastactivitytime[uart_index] > 100000)
				{
					enable_led(uart_index, 0);
				}

				fifoed_avalon_uart_snaphot tmp;

				// Proceed to read the channel. The call is NOT-blocking, so as to scan all channels continuously,
				// and only grab data when it is present in the FIFO.
				unsigned int nb_read=read(uart_fd[uart_index],&tmp,0);

				if (nb_read != 0)
				{
					// keep track that a message was received (on any channel), to remember to push the updated RXinfo data to HPS
					msg_received = true;

					// (re)activate the led corresponding to the channel, and keep track of the activation time.
					enable_led(uart_index, 1);
					lastactivitytime[uart_index] = time;

					// DEBUG
					printf("NB read UART%d: %d frames read\n", uart_index, nb_read);

					fifoed_avalon_uart_state* state = tmp.ptr;
					unsigned short frameIndex = tmp.rx_framestart_index;

					UART_RXFrameInfo frameInfo[MAX_NB_FRAMES_BUFFERED];


					int nbFrames = tmp.rx_framecount < 2 ? tmp.rx_framecount: 2;

					for (unsigned int i=0; i < /*tmp.rx_framecount*/nbFrames; i++)
					{
						//DEBUG
						printf("Dealing with frameIndex=%u\n", frameIndex);

						alt_u32 frameLength = state->rx_frame_size[frameIndex];

						// DEBUG
						printf ("Frame %u: length=%lu\n", i, frameLength);

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
							printf("Frame %u: frameStartAddress_1stPart=%p\n", i,frameStartAddress_1stPart);
							printf("Frame %u: frameLength_1stPart=%lu\n", i,frameLength_1stPart);
							printf("Frame %u: frameStartAddress_2ndPart=%p\n", i,frameStartAddress_2ndPart);
							printf("Frame %u: frameLength_2ndPart=%lu\n", i,frameLength_2ndPart);

							// Transfer 1st part
							int err;
							alt_msgdma_construct_standard_mm_to_mm_descriptor(
									MSGDMADev,
									&MSGDMA_DESC,
									frameStartAddress_1stPart, /* read address */
									(alt_u32*)(UART_RXinfo.RXNextFrameAddress[uart_index]), /* write address */
									frameLength_1stPart , /* size in bytes of data*/
									0);

							alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);

							// Transfer 2nd part (if applicable)
							if (frameLength_2ndPart !=0)
							{
								alt_msgdma_construct_standard_mm_to_mm_descriptor(
										MSGDMADev,
										&MSGDMA_DESC,
										frameStartAddress_2ndPart, /* read address */
										(alt_u32*)(UART_RXinfo.RXNextFrameAddress[uart_index]+frameLength_1stPart), /* write address */
										frameLength_2ndPart , /* size in bytes of data*/
										0);

								alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);
							}

							//////////////////////////////////////////////////
							// Prepare and transfer DMA transfer of Frame Info
							//////////////////////////////////////////////////
							frameInfo[i].timestamp = state->rx_timestamp[frameIndex];

							// DEBUG
							printf("Frame %u: Timestamp %llu\n", i , frameInfo[i].timestamp);

							frameInfo[i].framelength = frameLength;

							err= alt_msgdma_construct_standard_mm_to_mm_descriptor(
									MSGDMADev,
									&MSGDMA_DESC,
									(alt_u32*)&frameInfo[i], /* read address */
									(alt_u32*)(UART_RXinfo.RXNextFrameAddress[uart_index]+UART_RXinfo.RXMaxFrameSize[uart_index]), /* write address */
									TIMESTAMP_SIZE+FRAME_LENGTH_SIZE, /* size in bytes of info data*/
									0);

							err = alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);

							if (err != 0)
							{
								// DEBUG
								//printf("Async DMA err %d\n", err);
							}

							//printf("DONE Writing at address 0x%p\n", UART_RXinfo.RXNextFrameAddress[uart_index]);

							/////////////////////////////////////////////
							// Move destination pointer for next transfer.
							/////////////////////////////////////////////
							UART_RXinfo.RXNextFrameAddress[uart_index] += UART_RXinfo.RXMaxFrameSize[uart_index] + TIMESTAMP_SIZE + FRAME_LENGTH_SIZE;

							// Rollover if needed (when there is not enough space left to store one frame plus its timestamp and length
							if (UART_RXinfo.RXNextFrameAddress[uart_index] >= UART_RXinfo.RXTopAddress[uart_index] - UART_RXinfo.RXMaxFrameSize[uart_index] - TIMESTAMP_SIZE - FRAME_LENGTH_SIZE)
								UART_RXinfo.RXNextFrameAddress[uart_index] = UART_RXinfo.RXBaseAddress[uart_index];

							//printf("Next frame will go at address 0x%p\n", UART_RXinfo.RXNextFrameAddress[uart_index]);
						}

						// move/roolback to next frame index
						frameIndex = (frameIndex+1) & MAX_NB_FRAMES_BUFFERED_MASK;
					}
				}

				/*
				printf("Sending data back over UART0...\n");
				tx_enable_UART0(true);
				int nb_written=write(fp_0,RX,32);
				tx_enable_UART0(false);
				printf("DONE, wrote %d chars\n", nb_written);
				 */
			}
		}

		time_stop = alt_timestamp() / divisor;

		// If anything was received on any channel, refresh UART info struct in DDR for HPS to catch up
		if (msg_received && MSGDMADev != NULL)
		{
			// DEBUG
			printf("Timestamp delta: %lld\n", time_stop-time_start);
			printf("Refreshing UART INFO in DDR\n");

			printf("UART_RXinfo.RXNextFrameAddress[0] = %p\n", UART_RXinfo.RXNextFrameAddress[0]);


			int err;
			err= alt_msgdma_construct_standard_mm_to_mm_descriptor(
					MSGDMADev,
					&MSGDMA_DESC,
					(alt_u32*)(&UART_RXinfo), /* read address  */
					RAMDEST_UART_RXINFO,      /* write address */
					sizeof(UART_RXinfo),      /* size in bytes */
					0);

			if (err != 0)
			{
				printf("DMA prep UARTRXInfo err %d\n", err);
			}

			err = alt_msgdma_standard_descriptor_async_transfer(MSGDMADev, &MSGDMA_DESC);

			if (err != 0)
			{
				printf("DMA run  UARTRXInfo err %d\n", err);
			}
		}

		// DEBUG
		usleep(1000000);

	}

	for (int uart_to_close=0; uart_to_close<6; uart_to_close++)
	{
		close(uart_fd[uart_to_close]);
	}

	return 0;
}
