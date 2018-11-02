/*
 * hps_linux.h
 *
 *  Created on: Sep 1, 2018
 *      Author: etabli
 */

#ifndef HPS_LINUX_H_
#define HPS_LINUX_H_

#include <stdbool.h>
#include <stdint.h>

#include "socal/hps.h"
//#include "hps_soc_system.h"

// |=============|==========|==============|==========|
// | Signal Name | HPS GPIO | Register/bit | Function |
// |=============|==========|==============|==========|
// |   HPS_LED   |  GPIO53  |   GPIO1[24]  |    I/O   |
// |=============|==========|==============|==========|
#define HPS_LED_IDX      (ALT_GPIO_1BIT_53)                      // GPIO53
#define HPS_LED_PORT     (alt_gpio_bit_to_pid(HPS_LED_IDX))      // ALT_GPIO_PORTB
#define HPS_LED_PORT_BIT (alt_gpio_bit_to_port_pin(HPS_LED_IDX)) // 24 (from GPIO1[24])
#define HPS_LED_MASK     (1 << HPS_LED_PORT_BIT)

// |=============|==========|==============|==========|
// | Signal Name | HPS GPIO | Register/bit | Function |
// |=============|==========|==============|==========|
// |  HPS_KEY_N  |  GPIO54  |   GPIO1[25]  |    I/O   |
// |=============|==========|==============|==========|
#define HPS_KEY_N_IDX      (ALT_GPIO_1BIT_54)                        // GPIO54
#define HPS_KEY_N_PORT     (alt_gpio_bit_to_pid(HPS_KEY_N_IDX))      // ALT_GPIO_PORTB
#define HPS_KEY_N_PORT_BIT (alt_gpio_bit_to_port_pin(HPS_KEY_N_IDX)) // 25 (from GPIO1[25])
#define HPS_KEY_N_MASK     (1 << HPS_KEY_N_PORT_BIT)

// physical memory file descriptor
int fd_dev_mem = 0;

// memory-mapped peripherals
void   *hps_gpio     = NULL;
size_t hps_gpio_span = ALT_GPIO1_UB_ADDR - ALT_GPIO1_LB_ADDR + 1;
size_t hps_gpio_ofst = ALT_GPIO1_OFST;

void   *h2f_lw_axi_master     = NULL;
size_t h2f_lw_axi_master_span = ALT_LWFPGASLVS_UB_ADDR - ALT_LWFPGASLVS_LB_ADDR + 1;
size_t h2f_lw_axi_master_ofst = ALT_LWFPGASLVS_OFST;

void *fpga_leds = NULL;


#define RAMDEST_UART0_RX_START  0x20000000
#define RAMDEST_UART0_RX_END    0x21FFFFFF
#define RAMDEST_UART0_TX_START  0x22000000
#define RAMDEST_UART0_TX_END    0x23FFFFFF

#define RAMDEST_UART1_RX_START  0x24000000
#define RAMDEST_UART1_RX_END    0x25FFFFFF
#define RAMDEST_UART1_TX_START  0x26000000
#define RAMDEST_UART1_TX_END    0x27FFFFFF

#define RAMDEST_UART2_RX_START  0x28000000
#define RAMDEST_UART2_RX_END    0x29FFFFFF
#define RAMDEST_UART2_TX_START  0x2A000000
#define RAMDEST_UART2_TX_END    0x2BFFFFFF

#define RAMDEST_UART3_RX_START  0x2C000000
#define RAMDEST_UART3_RX_END    0x2DFFFFFF
#define RAMDEST_UART3_TX_START  0x2E000000
#define RAMDEST_UART3_TX_END    0x2FFFFFFF

#define RAMDEST_UART4_RX_START  0x30000000
#define RAMDEST_UART4_RX_END    0x31FFFFFF
#define RAMDEST_UART4_TX_START  0x32000000
#define RAMDEST_UART4_TX_END    0x33FFFFFF

#define RAMDEST_UART5_RX_START  0x34000000
#define RAMDEST_UART5_RX_END    0x35FFFFFF
#define RAMDEST_UART5_TX_START  0x36000000
#define RAMDEST_UART5_TX_END    0x37FFFFFF

#define RAMDEST_UART_INFO       0x38000000

void* RAMDest_UART_RX[6];
void* RAMDest_UART_Info;


void open_physical_memory_device();
void close_physical_memory_device();
void mmap_hps_peripherals();
void munmap_hps_peripherals();
void mmap_fpga_peripherals();
void munmap_fpga_peripherals();
void mmap_peripherals();
void munmap_peripherals();
void setup_hps_gpio();
void setup_fpga_leds();
void handle_hps_led();
void handle_fpga_leds();

#endif
