/*
 * system.h - SOPC Builder system and BSP software package information
 *
 * Machine generated for CPU 'nios2_gen2_0' in SOPC Builder design 'soc_system'
 * SOPC Builder design path: /home/etabli/DE0_Nano_SoC_Sniffer/hw/quartus/soc_system.sopcinfo
 *
 * Generated: Fri Nov 02 09:24:44 CET 2018
 */

/*
 * DO NOT MODIFY THIS FILE
 *
 * Changing this file will have subtle consequences
 * which will almost certainly lead to a nonfunctioning
 * system. If you do modify this file, be aware that your
 * changes will be overwritten and lost when this file
 * is generated again.
 *
 * DO NOT MODIFY THIS FILE
 */

/*
 * License Agreement
 *
 * Copyright (c) 2008
 * Altera Corporation, San Jose, California, USA.
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * This agreement shall be governed in all respects by the laws of the State
 * of California and by the laws of the United States of America.
 */

#ifndef __SYSTEM_H_
#define __SYSTEM_H_

/* Include definitions from linker script generator */
#include "linker.h"


/*
 * CPU configuration
 *
 */

#define ALT_CPU_ARCHITECTURE "altera_nios2_gen2"
#define ALT_CPU_BIG_ENDIAN 0
#define ALT_CPU_BREAK_ADDR 0x00040820
#define ALT_CPU_CPU_ARCH_NIOS2_R1
#define ALT_CPU_CPU_FREQ 50000000u
#define ALT_CPU_CPU_ID_SIZE 1
#define ALT_CPU_CPU_ID_VALUE 0x00000000
#define ALT_CPU_CPU_IMPLEMENTATION "tiny"
#define ALT_CPU_DATA_ADDR_WIDTH 0x13
#define ALT_CPU_DCACHE_LINE_SIZE 0
#define ALT_CPU_DCACHE_LINE_SIZE_LOG2 0
#define ALT_CPU_DCACHE_SIZE 0
#define ALT_CPU_EXCEPTION_ADDR 0x00000020
#define ALT_CPU_FLASH_ACCELERATOR_LINES 0
#define ALT_CPU_FLASH_ACCELERATOR_LINE_SIZE 0
#define ALT_CPU_FLUSHDA_SUPPORTED
#define ALT_CPU_FREQ 50000000
#define ALT_CPU_HARDWARE_DIVIDE_PRESENT 0
#define ALT_CPU_HARDWARE_MULTIPLY_PRESENT 0
#define ALT_CPU_HARDWARE_MULX_PRESENT 0
#define ALT_CPU_HAS_DEBUG_CORE 1
#define ALT_CPU_HAS_DEBUG_STUB
#define ALT_CPU_HAS_ILLEGAL_INSTRUCTION_EXCEPTION
#define ALT_CPU_HAS_JMPI_INSTRUCTION
#define ALT_CPU_ICACHE_LINE_SIZE 0
#define ALT_CPU_ICACHE_LINE_SIZE_LOG2 0
#define ALT_CPU_ICACHE_SIZE 0
#define ALT_CPU_INST_ADDR_WIDTH 0x13
#define ALT_CPU_NAME "nios2_gen2_0"
#define ALT_CPU_OCI_VERSION 1
#define ALT_CPU_RESET_ADDR 0x00000000


/*
 * CPU configuration (with legacy prefix - don't use these anymore)
 *
 */

#define NIOS2_BIG_ENDIAN 0
#define NIOS2_BREAK_ADDR 0x00040820
#define NIOS2_CPU_ARCH_NIOS2_R1
#define NIOS2_CPU_FREQ 50000000u
#define NIOS2_CPU_ID_SIZE 1
#define NIOS2_CPU_ID_VALUE 0x00000000
#define NIOS2_CPU_IMPLEMENTATION "tiny"
#define NIOS2_DATA_ADDR_WIDTH 0x13
#define NIOS2_DCACHE_LINE_SIZE 0
#define NIOS2_DCACHE_LINE_SIZE_LOG2 0
#define NIOS2_DCACHE_SIZE 0
#define NIOS2_EXCEPTION_ADDR 0x00000020
#define NIOS2_FLASH_ACCELERATOR_LINES 0
#define NIOS2_FLASH_ACCELERATOR_LINE_SIZE 0
#define NIOS2_FLUSHDA_SUPPORTED
#define NIOS2_HARDWARE_DIVIDE_PRESENT 0
#define NIOS2_HARDWARE_MULTIPLY_PRESENT 0
#define NIOS2_HARDWARE_MULX_PRESENT 0
#define NIOS2_HAS_DEBUG_CORE 1
#define NIOS2_HAS_DEBUG_STUB
#define NIOS2_HAS_ILLEGAL_INSTRUCTION_EXCEPTION
#define NIOS2_HAS_JMPI_INSTRUCTION
#define NIOS2_ICACHE_LINE_SIZE 0
#define NIOS2_ICACHE_LINE_SIZE_LOG2 0
#define NIOS2_ICACHE_SIZE 0
#define NIOS2_INST_ADDR_WIDTH 0x13
#define NIOS2_OCI_VERSION 1
#define NIOS2_RESET_ADDR 0x00000000


/*
 * Define for each module class mastered by the CPU
 *
 */

#define __ALTERA_AVALON_JTAG_UART
#define __ALTERA_AVALON_ONCHIP_MEMORY2
#define __ALTERA_AVALON_PIO
#define __ALTERA_AVALON_SYSID_QSYS
#define __ALTERA_AVALON_TIMER
#define __ALTERA_MSGDMA
#define __ALTERA_NIOS2_GEN2
#define __FIFOED_AVALON_UART


/*
 * System configuration
 *
 */

#define ALT_DEVICE_FAMILY "Cyclone V"
#define ALT_ENHANCED_INTERRUPT_API_PRESENT
#define ALT_IRQ_BASE NULL
#define ALT_LOG_PORT "/dev/null"
#define ALT_LOG_PORT_BASE 0x0
#define ALT_LOG_PORT_DEV null
#define ALT_LOG_PORT_TYPE ""
#define ALT_NUM_EXTERNAL_INTERRUPT_CONTROLLERS 0
#define ALT_NUM_INTERNAL_INTERRUPT_CONTROLLERS 1
#define ALT_NUM_INTERRUPT_CONTROLLERS 1
#define ALT_STDERR "/dev/jtag_uart_0"
#define ALT_STDERR_BASE 0x41268
#define ALT_STDERR_DEV jtag_uart_0
#define ALT_STDERR_IS_JTAG_UART
#define ALT_STDERR_PRESENT
#define ALT_STDERR_TYPE "altera_avalon_jtag_uart"
#define ALT_STDIN "/dev/jtag_uart_0"
#define ALT_STDIN_BASE 0x41268
#define ALT_STDIN_DEV jtag_uart_0
#define ALT_STDIN_IS_JTAG_UART
#define ALT_STDIN_PRESENT
#define ALT_STDIN_TYPE "altera_avalon_jtag_uart"
#define ALT_STDOUT "/dev/jtag_uart_0"
#define ALT_STDOUT_BASE 0x41268
#define ALT_STDOUT_DEV jtag_uart_0
#define ALT_STDOUT_IS_JTAG_UART
#define ALT_STDOUT_PRESENT
#define ALT_STDOUT_TYPE "altera_avalon_jtag_uart"
#define ALT_SYSTEM_NAME "soc_system"


/*
 * fifoed_avalon_uart_0 configuration
 *
 */

#define ALT_MODULE_CLASS_fifoed_avalon_uart_0 fifoed_avalon_uart
#define FIFOED_AVALON_UART_0_ADD_ERROR_BITS 0
#define FIFOED_AVALON_UART_0_BASE 0x41180
#define FIFOED_AVALON_UART_0_BAUD 1000000
#define FIFOED_AVALON_UART_0_DATA_BITS 8
#define FIFOED_AVALON_UART_0_FIFO_EXPORT_USED 0
#define FIFOED_AVALON_UART_0_FIXED_BAUD 1
#define FIFOED_AVALON_UART_0_FREQ 50000000
#define FIFOED_AVALON_UART_0_GAP_VALUE 2
#define FIFOED_AVALON_UART_0_IRQ 1
#define FIFOED_AVALON_UART_0_IRQ_INTERRUPT_CONTROLLER_ID 0
#define FIFOED_AVALON_UART_0_NAME "/dev/fifoed_avalon_uart_0"
#define FIFOED_AVALON_UART_0_PARITY 'N'
#define FIFOED_AVALON_UART_0_PASS_ERROR_BITS 0
#define FIFOED_AVALON_UART_0_RX_FIFO_LE 0
#define FIFOED_AVALON_UART_0_RX_FIFO_SIZE 4096
#define FIFOED_AVALON_UART_0_RX_IRQ_THRESHOLD 1
#define FIFOED_AVALON_UART_0_SIM_CHAR_STREAM ""
#define FIFOED_AVALON_UART_0_SIM_TRUE_BAUD 0
#define FIFOED_AVALON_UART_0_SPAN 64
#define FIFOED_AVALON_UART_0_STOP_BITS 1
#define FIFOED_AVALON_UART_0_SYNC_REG_DEPTH 2
#define FIFOED_AVALON_UART_0_TIMEOUT_VALUE 4
#define FIFOED_AVALON_UART_0_TIMESTAMP_WIDTH 8
#define FIFOED_AVALON_UART_0_TRANSMIT_PIN 0
#define FIFOED_AVALON_UART_0_TX_FIFO_LE 0
#define FIFOED_AVALON_UART_0_TX_FIFO_SIZE 8
#define FIFOED_AVALON_UART_0_TX_IRQ_THRESHOLD 1
#define FIFOED_AVALON_UART_0_TYPE "fifoed_avalon_uart"
#define FIFOED_AVALON_UART_0_UHW_CTS 0
#define FIFOED_AVALON_UART_0_USE_CTS_RTS 0
#define FIFOED_AVALON_UART_0_USE_EOP_REGISTER 0
#define FIFOED_AVALON_UART_0_USE_EXT_TIMESTAMP 0
#define FIFOED_AVALON_UART_0_USE_GAP_DETECTION 1
#define FIFOED_AVALON_UART_0_USE_RX_FIFO 1
#define FIFOED_AVALON_UART_0_USE_RX_TIMEOUT 0
#define FIFOED_AVALON_UART_0_USE_STATUS_BIT_CLEAR 0
#define FIFOED_AVALON_UART_0_USE_TIMESTAMP 0
#define FIFOED_AVALON_UART_0_USE_TX_FIFO 0


/*
 * fifoed_avalon_uart_0_txenable configuration
 *
 */

#define ALT_MODULE_CLASS_fifoed_avalon_uart_0_txenable altera_avalon_pio
#define FIFOED_AVALON_UART_0_TXENABLE_BASE 0x41230
#define FIFOED_AVALON_UART_0_TXENABLE_BIT_CLEARING_EDGE_REGISTER 0
#define FIFOED_AVALON_UART_0_TXENABLE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define FIFOED_AVALON_UART_0_TXENABLE_CAPTURE 0
#define FIFOED_AVALON_UART_0_TXENABLE_DATA_WIDTH 1
#define FIFOED_AVALON_UART_0_TXENABLE_DO_TEST_BENCH_WIRING 0
#define FIFOED_AVALON_UART_0_TXENABLE_DRIVEN_SIM_VALUE 0
#define FIFOED_AVALON_UART_0_TXENABLE_EDGE_TYPE "NONE"
#define FIFOED_AVALON_UART_0_TXENABLE_FREQ 50000000
#define FIFOED_AVALON_UART_0_TXENABLE_HAS_IN 0
#define FIFOED_AVALON_UART_0_TXENABLE_HAS_OUT 1
#define FIFOED_AVALON_UART_0_TXENABLE_HAS_TRI 0
#define FIFOED_AVALON_UART_0_TXENABLE_IRQ -1
#define FIFOED_AVALON_UART_0_TXENABLE_IRQ_INTERRUPT_CONTROLLER_ID -1
#define FIFOED_AVALON_UART_0_TXENABLE_IRQ_TYPE "NONE"
#define FIFOED_AVALON_UART_0_TXENABLE_NAME "/dev/fifoed_avalon_uart_0_txenable"
#define FIFOED_AVALON_UART_0_TXENABLE_RESET_VALUE 0
#define FIFOED_AVALON_UART_0_TXENABLE_SPAN 16
#define FIFOED_AVALON_UART_0_TXENABLE_TYPE "altera_avalon_pio"


/*
 * fifoed_avalon_uart_1 configuration
 *
 */

#define ALT_MODULE_CLASS_fifoed_avalon_uart_1 fifoed_avalon_uart
#define FIFOED_AVALON_UART_1_ADD_ERROR_BITS 0
#define FIFOED_AVALON_UART_1_BASE 0x41140
#define FIFOED_AVALON_UART_1_BAUD 1000000
#define FIFOED_AVALON_UART_1_DATA_BITS 8
#define FIFOED_AVALON_UART_1_FIFO_EXPORT_USED 0
#define FIFOED_AVALON_UART_1_FIXED_BAUD 1
#define FIFOED_AVALON_UART_1_FREQ 50000000
#define FIFOED_AVALON_UART_1_GAP_VALUE 2
#define FIFOED_AVALON_UART_1_IRQ 2
#define FIFOED_AVALON_UART_1_IRQ_INTERRUPT_CONTROLLER_ID 0
#define FIFOED_AVALON_UART_1_NAME "/dev/fifoed_avalon_uart_1"
#define FIFOED_AVALON_UART_1_PARITY 'N'
#define FIFOED_AVALON_UART_1_PASS_ERROR_BITS 0
#define FIFOED_AVALON_UART_1_RX_FIFO_LE 0
#define FIFOED_AVALON_UART_1_RX_FIFO_SIZE 4096
#define FIFOED_AVALON_UART_1_RX_IRQ_THRESHOLD 1
#define FIFOED_AVALON_UART_1_SIM_CHAR_STREAM ""
#define FIFOED_AVALON_UART_1_SIM_TRUE_BAUD 0
#define FIFOED_AVALON_UART_1_SPAN 64
#define FIFOED_AVALON_UART_1_STOP_BITS 1
#define FIFOED_AVALON_UART_1_SYNC_REG_DEPTH 2
#define FIFOED_AVALON_UART_1_TIMEOUT_VALUE 4
#define FIFOED_AVALON_UART_1_TIMESTAMP_WIDTH 8
#define FIFOED_AVALON_UART_1_TRANSMIT_PIN 0
#define FIFOED_AVALON_UART_1_TX_FIFO_LE 0
#define FIFOED_AVALON_UART_1_TX_FIFO_SIZE 8
#define FIFOED_AVALON_UART_1_TX_IRQ_THRESHOLD 1
#define FIFOED_AVALON_UART_1_TYPE "fifoed_avalon_uart"
#define FIFOED_AVALON_UART_1_UHW_CTS 0
#define FIFOED_AVALON_UART_1_USE_CTS_RTS 0
#define FIFOED_AVALON_UART_1_USE_EOP_REGISTER 0
#define FIFOED_AVALON_UART_1_USE_EXT_TIMESTAMP 0
#define FIFOED_AVALON_UART_1_USE_GAP_DETECTION 1
#define FIFOED_AVALON_UART_1_USE_RX_FIFO 1
#define FIFOED_AVALON_UART_1_USE_RX_TIMEOUT 0
#define FIFOED_AVALON_UART_1_USE_STATUS_BIT_CLEAR 0
#define FIFOED_AVALON_UART_1_USE_TIMESTAMP 0
#define FIFOED_AVALON_UART_1_USE_TX_FIFO 0


/*
 * fifoed_avalon_uart_1_txenable configuration
 *
 */

#define ALT_MODULE_CLASS_fifoed_avalon_uart_1_txenable altera_avalon_pio
#define FIFOED_AVALON_UART_1_TXENABLE_BASE 0x41220
#define FIFOED_AVALON_UART_1_TXENABLE_BIT_CLEARING_EDGE_REGISTER 0
#define FIFOED_AVALON_UART_1_TXENABLE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define FIFOED_AVALON_UART_1_TXENABLE_CAPTURE 0
#define FIFOED_AVALON_UART_1_TXENABLE_DATA_WIDTH 1
#define FIFOED_AVALON_UART_1_TXENABLE_DO_TEST_BENCH_WIRING 0
#define FIFOED_AVALON_UART_1_TXENABLE_DRIVEN_SIM_VALUE 0
#define FIFOED_AVALON_UART_1_TXENABLE_EDGE_TYPE "NONE"
#define FIFOED_AVALON_UART_1_TXENABLE_FREQ 50000000
#define FIFOED_AVALON_UART_1_TXENABLE_HAS_IN 0
#define FIFOED_AVALON_UART_1_TXENABLE_HAS_OUT 1
#define FIFOED_AVALON_UART_1_TXENABLE_HAS_TRI 0
#define FIFOED_AVALON_UART_1_TXENABLE_IRQ -1
#define FIFOED_AVALON_UART_1_TXENABLE_IRQ_INTERRUPT_CONTROLLER_ID -1
#define FIFOED_AVALON_UART_1_TXENABLE_IRQ_TYPE "NONE"
#define FIFOED_AVALON_UART_1_TXENABLE_NAME "/dev/fifoed_avalon_uart_1_txenable"
#define FIFOED_AVALON_UART_1_TXENABLE_RESET_VALUE 0
#define FIFOED_AVALON_UART_1_TXENABLE_SPAN 16
#define FIFOED_AVALON_UART_1_TXENABLE_TYPE "altera_avalon_pio"


/*
 * fifoed_avalon_uart_2 configuration
 *
 */

#define ALT_MODULE_CLASS_fifoed_avalon_uart_2 fifoed_avalon_uart
#define FIFOED_AVALON_UART_2_ADD_ERROR_BITS 0
#define FIFOED_AVALON_UART_2_BASE 0x41100
#define FIFOED_AVALON_UART_2_BAUD 1000000
#define FIFOED_AVALON_UART_2_DATA_BITS 8
#define FIFOED_AVALON_UART_2_FIFO_EXPORT_USED 0
#define FIFOED_AVALON_UART_2_FIXED_BAUD 1
#define FIFOED_AVALON_UART_2_FREQ 50000000
#define FIFOED_AVALON_UART_2_GAP_VALUE 2
#define FIFOED_AVALON_UART_2_IRQ 3
#define FIFOED_AVALON_UART_2_IRQ_INTERRUPT_CONTROLLER_ID 0
#define FIFOED_AVALON_UART_2_NAME "/dev/fifoed_avalon_uart_2"
#define FIFOED_AVALON_UART_2_PARITY 'N'
#define FIFOED_AVALON_UART_2_PASS_ERROR_BITS 0
#define FIFOED_AVALON_UART_2_RX_FIFO_LE 0
#define FIFOED_AVALON_UART_2_RX_FIFO_SIZE 4096
#define FIFOED_AVALON_UART_2_RX_IRQ_THRESHOLD 1
#define FIFOED_AVALON_UART_2_SIM_CHAR_STREAM ""
#define FIFOED_AVALON_UART_2_SIM_TRUE_BAUD 0
#define FIFOED_AVALON_UART_2_SPAN 64
#define FIFOED_AVALON_UART_2_STOP_BITS 1
#define FIFOED_AVALON_UART_2_SYNC_REG_DEPTH 2
#define FIFOED_AVALON_UART_2_TIMEOUT_VALUE 4
#define FIFOED_AVALON_UART_2_TIMESTAMP_WIDTH 8
#define FIFOED_AVALON_UART_2_TRANSMIT_PIN 0
#define FIFOED_AVALON_UART_2_TX_FIFO_LE 0
#define FIFOED_AVALON_UART_2_TX_FIFO_SIZE 8
#define FIFOED_AVALON_UART_2_TX_IRQ_THRESHOLD 1
#define FIFOED_AVALON_UART_2_TYPE "fifoed_avalon_uart"
#define FIFOED_AVALON_UART_2_UHW_CTS 0
#define FIFOED_AVALON_UART_2_USE_CTS_RTS 0
#define FIFOED_AVALON_UART_2_USE_EOP_REGISTER 0
#define FIFOED_AVALON_UART_2_USE_EXT_TIMESTAMP 0
#define FIFOED_AVALON_UART_2_USE_GAP_DETECTION 1
#define FIFOED_AVALON_UART_2_USE_RX_FIFO 1
#define FIFOED_AVALON_UART_2_USE_RX_TIMEOUT 0
#define FIFOED_AVALON_UART_2_USE_STATUS_BIT_CLEAR 0
#define FIFOED_AVALON_UART_2_USE_TIMESTAMP 0
#define FIFOED_AVALON_UART_2_USE_TX_FIFO 0


/*
 * fifoed_avalon_uart_2_txenable configuration
 *
 */

#define ALT_MODULE_CLASS_fifoed_avalon_uart_2_txenable altera_avalon_pio
#define FIFOED_AVALON_UART_2_TXENABLE_BASE 0x41210
#define FIFOED_AVALON_UART_2_TXENABLE_BIT_CLEARING_EDGE_REGISTER 0
#define FIFOED_AVALON_UART_2_TXENABLE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define FIFOED_AVALON_UART_2_TXENABLE_CAPTURE 0
#define FIFOED_AVALON_UART_2_TXENABLE_DATA_WIDTH 1
#define FIFOED_AVALON_UART_2_TXENABLE_DO_TEST_BENCH_WIRING 0
#define FIFOED_AVALON_UART_2_TXENABLE_DRIVEN_SIM_VALUE 0
#define FIFOED_AVALON_UART_2_TXENABLE_EDGE_TYPE "NONE"
#define FIFOED_AVALON_UART_2_TXENABLE_FREQ 50000000
#define FIFOED_AVALON_UART_2_TXENABLE_HAS_IN 0
#define FIFOED_AVALON_UART_2_TXENABLE_HAS_OUT 1
#define FIFOED_AVALON_UART_2_TXENABLE_HAS_TRI 0
#define FIFOED_AVALON_UART_2_TXENABLE_IRQ -1
#define FIFOED_AVALON_UART_2_TXENABLE_IRQ_INTERRUPT_CONTROLLER_ID -1
#define FIFOED_AVALON_UART_2_TXENABLE_IRQ_TYPE "NONE"
#define FIFOED_AVALON_UART_2_TXENABLE_NAME "/dev/fifoed_avalon_uart_2_txenable"
#define FIFOED_AVALON_UART_2_TXENABLE_RESET_VALUE 0
#define FIFOED_AVALON_UART_2_TXENABLE_SPAN 16
#define FIFOED_AVALON_UART_2_TXENABLE_TYPE "altera_avalon_pio"


/*
 * fifoed_avalon_uart_3 configuration
 *
 */

#define ALT_MODULE_CLASS_fifoed_avalon_uart_3 fifoed_avalon_uart
#define FIFOED_AVALON_UART_3_ADD_ERROR_BITS 0
#define FIFOED_AVALON_UART_3_BASE 0x410c0
#define FIFOED_AVALON_UART_3_BAUD 1000000
#define FIFOED_AVALON_UART_3_DATA_BITS 8
#define FIFOED_AVALON_UART_3_FIFO_EXPORT_USED 0
#define FIFOED_AVALON_UART_3_FIXED_BAUD 1
#define FIFOED_AVALON_UART_3_FREQ 50000000
#define FIFOED_AVALON_UART_3_GAP_VALUE 2
#define FIFOED_AVALON_UART_3_IRQ 4
#define FIFOED_AVALON_UART_3_IRQ_INTERRUPT_CONTROLLER_ID 0
#define FIFOED_AVALON_UART_3_NAME "/dev/fifoed_avalon_uart_3"
#define FIFOED_AVALON_UART_3_PARITY 'N'
#define FIFOED_AVALON_UART_3_PASS_ERROR_BITS 0
#define FIFOED_AVALON_UART_3_RX_FIFO_LE 0
#define FIFOED_AVALON_UART_3_RX_FIFO_SIZE 4096
#define FIFOED_AVALON_UART_3_RX_IRQ_THRESHOLD 1
#define FIFOED_AVALON_UART_3_SIM_CHAR_STREAM ""
#define FIFOED_AVALON_UART_3_SIM_TRUE_BAUD 0
#define FIFOED_AVALON_UART_3_SPAN 64
#define FIFOED_AVALON_UART_3_STOP_BITS 1
#define FIFOED_AVALON_UART_3_SYNC_REG_DEPTH 2
#define FIFOED_AVALON_UART_3_TIMEOUT_VALUE 4
#define FIFOED_AVALON_UART_3_TIMESTAMP_WIDTH 8
#define FIFOED_AVALON_UART_3_TRANSMIT_PIN 0
#define FIFOED_AVALON_UART_3_TX_FIFO_LE 0
#define FIFOED_AVALON_UART_3_TX_FIFO_SIZE 8
#define FIFOED_AVALON_UART_3_TX_IRQ_THRESHOLD 1
#define FIFOED_AVALON_UART_3_TYPE "fifoed_avalon_uart"
#define FIFOED_AVALON_UART_3_UHW_CTS 0
#define FIFOED_AVALON_UART_3_USE_CTS_RTS 0
#define FIFOED_AVALON_UART_3_USE_EOP_REGISTER 0
#define FIFOED_AVALON_UART_3_USE_EXT_TIMESTAMP 0
#define FIFOED_AVALON_UART_3_USE_GAP_DETECTION 1
#define FIFOED_AVALON_UART_3_USE_RX_FIFO 1
#define FIFOED_AVALON_UART_3_USE_RX_TIMEOUT 0
#define FIFOED_AVALON_UART_3_USE_STATUS_BIT_CLEAR 0
#define FIFOED_AVALON_UART_3_USE_TIMESTAMP 0
#define FIFOED_AVALON_UART_3_USE_TX_FIFO 0


/*
 * fifoed_avalon_uart_3_txenable configuration
 *
 */

#define ALT_MODULE_CLASS_fifoed_avalon_uart_3_txenable altera_avalon_pio
#define FIFOED_AVALON_UART_3_TXENABLE_BASE 0x41200
#define FIFOED_AVALON_UART_3_TXENABLE_BIT_CLEARING_EDGE_REGISTER 0
#define FIFOED_AVALON_UART_3_TXENABLE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define FIFOED_AVALON_UART_3_TXENABLE_CAPTURE 0
#define FIFOED_AVALON_UART_3_TXENABLE_DATA_WIDTH 1
#define FIFOED_AVALON_UART_3_TXENABLE_DO_TEST_BENCH_WIRING 0
#define FIFOED_AVALON_UART_3_TXENABLE_DRIVEN_SIM_VALUE 0
#define FIFOED_AVALON_UART_3_TXENABLE_EDGE_TYPE "NONE"
#define FIFOED_AVALON_UART_3_TXENABLE_FREQ 50000000
#define FIFOED_AVALON_UART_3_TXENABLE_HAS_IN 0
#define FIFOED_AVALON_UART_3_TXENABLE_HAS_OUT 1
#define FIFOED_AVALON_UART_3_TXENABLE_HAS_TRI 0
#define FIFOED_AVALON_UART_3_TXENABLE_IRQ -1
#define FIFOED_AVALON_UART_3_TXENABLE_IRQ_INTERRUPT_CONTROLLER_ID -1
#define FIFOED_AVALON_UART_3_TXENABLE_IRQ_TYPE "NONE"
#define FIFOED_AVALON_UART_3_TXENABLE_NAME "/dev/fifoed_avalon_uart_3_txenable"
#define FIFOED_AVALON_UART_3_TXENABLE_RESET_VALUE 0
#define FIFOED_AVALON_UART_3_TXENABLE_SPAN 16
#define FIFOED_AVALON_UART_3_TXENABLE_TYPE "altera_avalon_pio"


/*
 * fifoed_avalon_uart_4 configuration
 *
 */

#define ALT_MODULE_CLASS_fifoed_avalon_uart_4 fifoed_avalon_uart
#define FIFOED_AVALON_UART_4_ADD_ERROR_BITS 0
#define FIFOED_AVALON_UART_4_BASE 0x41080
#define FIFOED_AVALON_UART_4_BAUD 1000000
#define FIFOED_AVALON_UART_4_DATA_BITS 8
#define FIFOED_AVALON_UART_4_FIFO_EXPORT_USED 0
#define FIFOED_AVALON_UART_4_FIXED_BAUD 1
#define FIFOED_AVALON_UART_4_FREQ 50000000
#define FIFOED_AVALON_UART_4_GAP_VALUE 2
#define FIFOED_AVALON_UART_4_IRQ 5
#define FIFOED_AVALON_UART_4_IRQ_INTERRUPT_CONTROLLER_ID 0
#define FIFOED_AVALON_UART_4_NAME "/dev/fifoed_avalon_uart_4"
#define FIFOED_AVALON_UART_4_PARITY 'N'
#define FIFOED_AVALON_UART_4_PASS_ERROR_BITS 0
#define FIFOED_AVALON_UART_4_RX_FIFO_LE 0
#define FIFOED_AVALON_UART_4_RX_FIFO_SIZE 4096
#define FIFOED_AVALON_UART_4_RX_IRQ_THRESHOLD 1
#define FIFOED_AVALON_UART_4_SIM_CHAR_STREAM ""
#define FIFOED_AVALON_UART_4_SIM_TRUE_BAUD 0
#define FIFOED_AVALON_UART_4_SPAN 64
#define FIFOED_AVALON_UART_4_STOP_BITS 1
#define FIFOED_AVALON_UART_4_SYNC_REG_DEPTH 2
#define FIFOED_AVALON_UART_4_TIMEOUT_VALUE 4
#define FIFOED_AVALON_UART_4_TIMESTAMP_WIDTH 8
#define FIFOED_AVALON_UART_4_TRANSMIT_PIN 0
#define FIFOED_AVALON_UART_4_TX_FIFO_LE 0
#define FIFOED_AVALON_UART_4_TX_FIFO_SIZE 8
#define FIFOED_AVALON_UART_4_TX_IRQ_THRESHOLD 1
#define FIFOED_AVALON_UART_4_TYPE "fifoed_avalon_uart"
#define FIFOED_AVALON_UART_4_UHW_CTS 0
#define FIFOED_AVALON_UART_4_USE_CTS_RTS 0
#define FIFOED_AVALON_UART_4_USE_EOP_REGISTER 0
#define FIFOED_AVALON_UART_4_USE_EXT_TIMESTAMP 0
#define FIFOED_AVALON_UART_4_USE_GAP_DETECTION 1
#define FIFOED_AVALON_UART_4_USE_RX_FIFO 1
#define FIFOED_AVALON_UART_4_USE_RX_TIMEOUT 0
#define FIFOED_AVALON_UART_4_USE_STATUS_BIT_CLEAR 0
#define FIFOED_AVALON_UART_4_USE_TIMESTAMP 0
#define FIFOED_AVALON_UART_4_USE_TX_FIFO 0


/*
 * fifoed_avalon_uart_4_txenable configuration
 *
 */

#define ALT_MODULE_CLASS_fifoed_avalon_uart_4_txenable altera_avalon_pio
#define FIFOED_AVALON_UART_4_TXENABLE_BASE 0x411f0
#define FIFOED_AVALON_UART_4_TXENABLE_BIT_CLEARING_EDGE_REGISTER 0
#define FIFOED_AVALON_UART_4_TXENABLE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define FIFOED_AVALON_UART_4_TXENABLE_CAPTURE 0
#define FIFOED_AVALON_UART_4_TXENABLE_DATA_WIDTH 1
#define FIFOED_AVALON_UART_4_TXENABLE_DO_TEST_BENCH_WIRING 0
#define FIFOED_AVALON_UART_4_TXENABLE_DRIVEN_SIM_VALUE 0
#define FIFOED_AVALON_UART_4_TXENABLE_EDGE_TYPE "NONE"
#define FIFOED_AVALON_UART_4_TXENABLE_FREQ 50000000
#define FIFOED_AVALON_UART_4_TXENABLE_HAS_IN 0
#define FIFOED_AVALON_UART_4_TXENABLE_HAS_OUT 1
#define FIFOED_AVALON_UART_4_TXENABLE_HAS_TRI 0
#define FIFOED_AVALON_UART_4_TXENABLE_IRQ -1
#define FIFOED_AVALON_UART_4_TXENABLE_IRQ_INTERRUPT_CONTROLLER_ID -1
#define FIFOED_AVALON_UART_4_TXENABLE_IRQ_TYPE "NONE"
#define FIFOED_AVALON_UART_4_TXENABLE_NAME "/dev/fifoed_avalon_uart_4_txenable"
#define FIFOED_AVALON_UART_4_TXENABLE_RESET_VALUE 0
#define FIFOED_AVALON_UART_4_TXENABLE_SPAN 16
#define FIFOED_AVALON_UART_4_TXENABLE_TYPE "altera_avalon_pio"


/*
 * fifoed_avalon_uart_5 configuration
 *
 */

#define ALT_MODULE_CLASS_fifoed_avalon_uart_5 fifoed_avalon_uart
#define FIFOED_AVALON_UART_5_ADD_ERROR_BITS 0
#define FIFOED_AVALON_UART_5_BASE 0x41040
#define FIFOED_AVALON_UART_5_BAUD 1000000
#define FIFOED_AVALON_UART_5_DATA_BITS 8
#define FIFOED_AVALON_UART_5_FIFO_EXPORT_USED 0
#define FIFOED_AVALON_UART_5_FIXED_BAUD 1
#define FIFOED_AVALON_UART_5_FREQ 50000000
#define FIFOED_AVALON_UART_5_GAP_VALUE 2
#define FIFOED_AVALON_UART_5_IRQ 6
#define FIFOED_AVALON_UART_5_IRQ_INTERRUPT_CONTROLLER_ID 0
#define FIFOED_AVALON_UART_5_NAME "/dev/fifoed_avalon_uart_5"
#define FIFOED_AVALON_UART_5_PARITY 'N'
#define FIFOED_AVALON_UART_5_PASS_ERROR_BITS 0
#define FIFOED_AVALON_UART_5_RX_FIFO_LE 0
#define FIFOED_AVALON_UART_5_RX_FIFO_SIZE 4096
#define FIFOED_AVALON_UART_5_RX_IRQ_THRESHOLD 1
#define FIFOED_AVALON_UART_5_SIM_CHAR_STREAM ""
#define FIFOED_AVALON_UART_5_SIM_TRUE_BAUD 0
#define FIFOED_AVALON_UART_5_SPAN 64
#define FIFOED_AVALON_UART_5_STOP_BITS 1
#define FIFOED_AVALON_UART_5_SYNC_REG_DEPTH 2
#define FIFOED_AVALON_UART_5_TIMEOUT_VALUE 4
#define FIFOED_AVALON_UART_5_TIMESTAMP_WIDTH 8
#define FIFOED_AVALON_UART_5_TRANSMIT_PIN 0
#define FIFOED_AVALON_UART_5_TX_FIFO_LE 0
#define FIFOED_AVALON_UART_5_TX_FIFO_SIZE 8
#define FIFOED_AVALON_UART_5_TX_IRQ_THRESHOLD 1
#define FIFOED_AVALON_UART_5_TYPE "fifoed_avalon_uart"
#define FIFOED_AVALON_UART_5_UHW_CTS 0
#define FIFOED_AVALON_UART_5_USE_CTS_RTS 0
#define FIFOED_AVALON_UART_5_USE_EOP_REGISTER 0
#define FIFOED_AVALON_UART_5_USE_EXT_TIMESTAMP 0
#define FIFOED_AVALON_UART_5_USE_GAP_DETECTION 1
#define FIFOED_AVALON_UART_5_USE_RX_FIFO 1
#define FIFOED_AVALON_UART_5_USE_RX_TIMEOUT 0
#define FIFOED_AVALON_UART_5_USE_STATUS_BIT_CLEAR 0
#define FIFOED_AVALON_UART_5_USE_TIMESTAMP 0
#define FIFOED_AVALON_UART_5_USE_TX_FIFO 0


/*
 * fifoed_avalon_uart_5_txenable configuration
 *
 */

#define ALT_MODULE_CLASS_fifoed_avalon_uart_5_txenable altera_avalon_pio
#define FIFOED_AVALON_UART_5_TXENABLE_BASE 0x411e0
#define FIFOED_AVALON_UART_5_TXENABLE_BIT_CLEARING_EDGE_REGISTER 0
#define FIFOED_AVALON_UART_5_TXENABLE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define FIFOED_AVALON_UART_5_TXENABLE_CAPTURE 0
#define FIFOED_AVALON_UART_5_TXENABLE_DATA_WIDTH 1
#define FIFOED_AVALON_UART_5_TXENABLE_DO_TEST_BENCH_WIRING 0
#define FIFOED_AVALON_UART_5_TXENABLE_DRIVEN_SIM_VALUE 0
#define FIFOED_AVALON_UART_5_TXENABLE_EDGE_TYPE "NONE"
#define FIFOED_AVALON_UART_5_TXENABLE_FREQ 50000000
#define FIFOED_AVALON_UART_5_TXENABLE_HAS_IN 0
#define FIFOED_AVALON_UART_5_TXENABLE_HAS_OUT 1
#define FIFOED_AVALON_UART_5_TXENABLE_HAS_TRI 0
#define FIFOED_AVALON_UART_5_TXENABLE_IRQ -1
#define FIFOED_AVALON_UART_5_TXENABLE_IRQ_INTERRUPT_CONTROLLER_ID -1
#define FIFOED_AVALON_UART_5_TXENABLE_IRQ_TYPE "NONE"
#define FIFOED_AVALON_UART_5_TXENABLE_NAME "/dev/fifoed_avalon_uart_5_txenable"
#define FIFOED_AVALON_UART_5_TXENABLE_RESET_VALUE 0
#define FIFOED_AVALON_UART_5_TXENABLE_SPAN 16
#define FIFOED_AVALON_UART_5_TXENABLE_TYPE "altera_avalon_pio"


/*
 * hal configuration
 *
 */

#define ALT_INCLUDE_INSTRUCTION_RELATED_EXCEPTION_API
#define ALT_MAX_FD 32
#define ALT_SYS_CLK TIMER_0
#define ALT_TIMESTAMP_CLK TIMER_0


/*
 * jtag_uart_0 configuration
 *
 */

#define ALT_MODULE_CLASS_jtag_uart_0 altera_avalon_jtag_uart
#define JTAG_UART_0_BASE 0x41268
#define JTAG_UART_0_IRQ 0
#define JTAG_UART_0_IRQ_INTERRUPT_CONTROLLER_ID 0
#define JTAG_UART_0_NAME "/dev/jtag_uart_0"
#define JTAG_UART_0_READ_DEPTH 64
#define JTAG_UART_0_READ_THRESHOLD 8
#define JTAG_UART_0_SPAN 8
#define JTAG_UART_0_TYPE "altera_avalon_jtag_uart"
#define JTAG_UART_0_WRITE_DEPTH 64
#define JTAG_UART_0_WRITE_THRESHOLD 8


/*
 * msgdma_0_csr configuration
 *
 */

#define ALT_MODULE_CLASS_msgdma_0_csr altera_msgdma
#define MSGDMA_0_CSR_BASE 0x411c0
#define MSGDMA_0_CSR_BURST_ENABLE 0
#define MSGDMA_0_CSR_BURST_WRAPPING_SUPPORT 0
#define MSGDMA_0_CSR_CHANNEL_ENABLE 0
#define MSGDMA_0_CSR_CHANNEL_ENABLE_DERIVED 0
#define MSGDMA_0_CSR_CHANNEL_WIDTH 8
#define MSGDMA_0_CSR_DATA_FIFO_DEPTH 32
#define MSGDMA_0_CSR_DATA_WIDTH 32
#define MSGDMA_0_CSR_DESCRIPTOR_FIFO_DEPTH 128
#define MSGDMA_0_CSR_DMA_MODE 0
#define MSGDMA_0_CSR_ENHANCED_FEATURES 0
#define MSGDMA_0_CSR_ERROR_ENABLE 0
#define MSGDMA_0_CSR_ERROR_ENABLE_DERIVED 0
#define MSGDMA_0_CSR_ERROR_WIDTH 8
#define MSGDMA_0_CSR_IRQ 8
#define MSGDMA_0_CSR_IRQ_INTERRUPT_CONTROLLER_ID 0
#define MSGDMA_0_CSR_MAX_BURST_COUNT 2
#define MSGDMA_0_CSR_MAX_BYTE 8192
#define MSGDMA_0_CSR_MAX_STRIDE 1
#define MSGDMA_0_CSR_NAME "/dev/msgdma_0_csr"
#define MSGDMA_0_CSR_PACKET_ENABLE 0
#define MSGDMA_0_CSR_PACKET_ENABLE_DERIVED 0
#define MSGDMA_0_CSR_PREFETCHER_ENABLE 0
#define MSGDMA_0_CSR_PROGRAMMABLE_BURST_ENABLE 0
#define MSGDMA_0_CSR_RESPONSE_PORT 2
#define MSGDMA_0_CSR_SPAN 32
#define MSGDMA_0_CSR_STRIDE_ENABLE 0
#define MSGDMA_0_CSR_STRIDE_ENABLE_DERIVED 0
#define MSGDMA_0_CSR_TRANSFER_TYPE "Unaligned Accesses"
#define MSGDMA_0_CSR_TYPE "altera_msgdma"


/*
 * msgdma_0_descriptor_slave configuration
 *
 */

#define ALT_MODULE_CLASS_msgdma_0_descriptor_slave altera_msgdma
#define MSGDMA_0_DESCRIPTOR_SLAVE_BASE 0x41250
#define MSGDMA_0_DESCRIPTOR_SLAVE_BURST_ENABLE 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_BURST_WRAPPING_SUPPORT 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_CHANNEL_ENABLE 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_CHANNEL_ENABLE_DERIVED 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_CHANNEL_WIDTH 8
#define MSGDMA_0_DESCRIPTOR_SLAVE_DATA_FIFO_DEPTH 32
#define MSGDMA_0_DESCRIPTOR_SLAVE_DATA_WIDTH 32
#define MSGDMA_0_DESCRIPTOR_SLAVE_DESCRIPTOR_FIFO_DEPTH 128
#define MSGDMA_0_DESCRIPTOR_SLAVE_DMA_MODE 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_ENHANCED_FEATURES 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_ERROR_ENABLE 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_ERROR_ENABLE_DERIVED 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_ERROR_WIDTH 8
#define MSGDMA_0_DESCRIPTOR_SLAVE_IRQ -1
#define MSGDMA_0_DESCRIPTOR_SLAVE_IRQ_INTERRUPT_CONTROLLER_ID -1
#define MSGDMA_0_DESCRIPTOR_SLAVE_MAX_BURST_COUNT 2
#define MSGDMA_0_DESCRIPTOR_SLAVE_MAX_BYTE 8192
#define MSGDMA_0_DESCRIPTOR_SLAVE_MAX_STRIDE 1
#define MSGDMA_0_DESCRIPTOR_SLAVE_NAME "/dev/msgdma_0_descriptor_slave"
#define MSGDMA_0_DESCRIPTOR_SLAVE_PACKET_ENABLE 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_PACKET_ENABLE_DERIVED 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_PREFETCHER_ENABLE 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_PROGRAMMABLE_BURST_ENABLE 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_RESPONSE_PORT 2
#define MSGDMA_0_DESCRIPTOR_SLAVE_SPAN 16
#define MSGDMA_0_DESCRIPTOR_SLAVE_STRIDE_ENABLE 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_STRIDE_ENABLE_DERIVED 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_TRANSFER_TYPE "Unaligned Accesses"
#define MSGDMA_0_DESCRIPTOR_SLAVE_TYPE "altera_msgdma"


/*
 * nios_activity_leds configuration
 *
 */

#define ALT_MODULE_CLASS_nios_activity_leds altera_avalon_pio
#define NIOS_ACTIVITY_LEDS_BASE 0x41240
#define NIOS_ACTIVITY_LEDS_BIT_CLEARING_EDGE_REGISTER 0
#define NIOS_ACTIVITY_LEDS_BIT_MODIFYING_OUTPUT_REGISTER 0
#define NIOS_ACTIVITY_LEDS_CAPTURE 0
#define NIOS_ACTIVITY_LEDS_DATA_WIDTH 6
#define NIOS_ACTIVITY_LEDS_DO_TEST_BENCH_WIRING 0
#define NIOS_ACTIVITY_LEDS_DRIVEN_SIM_VALUE 0
#define NIOS_ACTIVITY_LEDS_EDGE_TYPE "NONE"
#define NIOS_ACTIVITY_LEDS_FREQ 50000000
#define NIOS_ACTIVITY_LEDS_HAS_IN 0
#define NIOS_ACTIVITY_LEDS_HAS_OUT 1
#define NIOS_ACTIVITY_LEDS_HAS_TRI 0
#define NIOS_ACTIVITY_LEDS_IRQ -1
#define NIOS_ACTIVITY_LEDS_IRQ_INTERRUPT_CONTROLLER_ID -1
#define NIOS_ACTIVITY_LEDS_IRQ_TYPE "NONE"
#define NIOS_ACTIVITY_LEDS_NAME "/dev/nios_activity_leds"
#define NIOS_ACTIVITY_LEDS_RESET_VALUE 0
#define NIOS_ACTIVITY_LEDS_SPAN 16
#define NIOS_ACTIVITY_LEDS_TYPE "altera_avalon_pio"


/*
 * onchip_memory2_0 configuration
 *
 */

#define ALT_MODULE_CLASS_onchip_memory2_0 altera_avalon_onchip_memory2
#define ONCHIP_MEMORY2_0_ALLOW_IN_SYSTEM_MEMORY_CONTENT_EDITOR 0
#define ONCHIP_MEMORY2_0_ALLOW_MRAM_SIM_CONTENTS_ONLY_FILE 0
#define ONCHIP_MEMORY2_0_BASE 0x0
#define ONCHIP_MEMORY2_0_CONTENTS_INFO ""
#define ONCHIP_MEMORY2_0_DUAL_PORT 0
#define ONCHIP_MEMORY2_0_GUI_RAM_BLOCK_TYPE "AUTO"
#define ONCHIP_MEMORY2_0_INIT_CONTENTS_FILE "soc_system_onchip_memory2_0"
#define ONCHIP_MEMORY2_0_INIT_MEM_CONTENT 1
#define ONCHIP_MEMORY2_0_INSTANCE_ID "NONE"
#define ONCHIP_MEMORY2_0_IRQ -1
#define ONCHIP_MEMORY2_0_IRQ_INTERRUPT_CONTROLLER_ID -1
#define ONCHIP_MEMORY2_0_NAME "/dev/onchip_memory2_0"
#define ONCHIP_MEMORY2_0_NON_DEFAULT_INIT_FILE_ENABLED 1
#define ONCHIP_MEMORY2_0_RAM_BLOCK_TYPE "AUTO"
#define ONCHIP_MEMORY2_0_READ_DURING_WRITE_MODE "DONT_CARE"
#define ONCHIP_MEMORY2_0_SINGLE_CLOCK_OP 0
#define ONCHIP_MEMORY2_0_SIZE_MULTIPLE 1
#define ONCHIP_MEMORY2_0_SIZE_VALUE 229376
#define ONCHIP_MEMORY2_0_SPAN 229376
#define ONCHIP_MEMORY2_0_TYPE "altera_avalon_onchip_memory2"
#define ONCHIP_MEMORY2_0_WRITABLE 1


/*
 * sysid configuration
 *
 */

#define ALT_MODULE_CLASS_sysid altera_avalon_sysid_qsys
#define SYSID_BASE 0x41260
#define SYSID_ID 0
#define SYSID_IRQ -1
#define SYSID_IRQ_INTERRUPT_CONTROLLER_ID -1
#define SYSID_NAME "/dev/sysid"
#define SYSID_SPAN 8
#define SYSID_TIMESTAMP 1541104560
#define SYSID_TYPE "altera_avalon_sysid_qsys"


/*
 * timer_0 configuration
 *
 */

#define ALT_MODULE_CLASS_timer_0 altera_avalon_timer
#define TIMER_0_ALWAYS_RUN 1
#define TIMER_0_BASE 0x41000
#define TIMER_0_COUNTER_SIZE 64
#define TIMER_0_FIXED_PERIOD 0
#define TIMER_0_FREQ 50000000
#define TIMER_0_IRQ 7
#define TIMER_0_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER_0_LOAD_VALUE 49999
#define TIMER_0_MULT 0.001
#define TIMER_0_NAME "/dev/timer_0"
#define TIMER_0_PERIOD 1
#define TIMER_0_PERIOD_UNITS "ms"
#define TIMER_0_RESET_OUTPUT 0
#define TIMER_0_SNAPSHOT 1
#define TIMER_0_SPAN 64
#define TIMER_0_TICKS_PER_SEC 1000
#define TIMER_0_TIMEOUT_PULSE_OUTPUT 0
#define TIMER_0_TYPE "altera_avalon_timer"

#endif /* __SYSTEM_H_ */
