#ifndef _ITECIRREG_H_
#define _ITECIRREG_H_

/* See IT8712F EC - LPC I/O Preliminary Specifiction V0.81
   for details.
*/

/* FIFO sizes */
#define ITE_TX_FIFO_LEN 32
#define ITE_RX_FIFO_LEN 32

/* Interrupt types */
#define ITE_IRQ_TX_FIFO         1
#define ITE_IRQ_RX_FIFO         2
#define ITE_IRQ_RX_FIFO_OVERRUN 4

#define ITE_BAUDRATE_DIVISOR 1
#define ITE_DEFAULT_BAUDRATE 115200

#define ITE_LCF_MIN_CARRIER_FREQ  27000
#define ITE_LCF_MAX_CARRIER_FREQ  58000
#define ITE_HCF_MIN_CARRIER_FREQ  400000
#define ITE_HCF_MAX_CARRIER_FREQ  500000
#define ITE_DEFAULT_CARRIER_FREQ  38000

#define ITE_IDLE_TIMEOUT 200000000UL

/* Convert bits to us */
#define ITE_BITS_TO_NS(bits, sample_period) \
((u_int32_t) ((bits) * ITE_BAUDRATE_DIVISOR * sample_period))

#define ITE_RXDCR_PER_10000_STEP 625

/* High speed carrier freq values */
#define ITE_CFQ_400		0x03
#define ITE_CFQ_450		0x08
#define ITE_CFQ_480		0x0b
#define ITE_CFQ_500		0x0d

/* Values for pulse widths */
#define ITE_TXMPW_A		0x02
#define ITE_TXMPW_B		0x03
#define ITE_TXMPW_C		0x04
#define ITE_TXMPW_D		0x05
#define ITE_TXMPW_E		0x06

#define IT87_CIR_RCR_RXDCR_DEFAULT 0x1
#define IT87_CIR_RCR_RXDCR_MAX 0x7

/* CIR register offsets */
#define IT87_CIR_DR   0
#define IT87_CIR_IER  1
#define IT87_CIR_RCR  2
#define IT87_CIR_TCR1 3
#define IT87_CIR_TCR2 4
#define IT87_CIR_TSR  5
#define IT87_CIR_RSR  6
#define IT87_CIR_BDLR 5
#define IT87_CIR_BDHR 6
#define IT87_CIR_IIR  7

#define IT87_CIR_IOREG_LEN 8

/* IER */
#define IT87_CIR_IER_RESET   0x20
#define IT87_CIR_IER_BR      0x10
#define IT87_CIR_IER_IEC     0x8
#define IT87_CIR_IER_RFOIE   0x4
#define IT87_CIR_IER_RDAIE   0x2
#define IT87_CIR_IER_TLDLIE  0x1

/* RCR */
#define IT87_CIR_RCR_RDWOS  0x80
#define IT87_CIR_RCR_HCFS   0x40
#define IT87_CIR_RCR_RXEN   0x20
#define IT87_CIR_RCR_RXEND  0x10
#define IT87_CIR_RCR_RXACT  0x8
#define IT87_CIR_RCR_RXDCR  0x7

/* TCR1 */
#define IT87_CIR_TCR1_FIFOCLR 0x80
#define IT87_CIR_TCR1_ILE     0x40
#define IT87_CIR_TCR1_FIFOTL  0x30
#define IT87_CIR_TCR1_FIFOTL_DEFAULT  0x20
#define IT87_CIR_TCR1_TXRLE   0x8
#define IT87_CIR_TCR1_TXENDF  0x4
#define IT87_CIR_TCR1_TXMPM   0x3
#define IT87_CIR_TCR1_TXMPM_DEFAULT   0x0

/* TCR2 */
#define IT87_CIR_TCR2_CFQ           0xf8
#define IT87_CIR_TCR2_TXMPW         0x7
#define IT87_CIR_TCR2_TXMPW_DEFAULT 0x4
#define IT87_CIR_TCR2_CFQ_SHIFT     3

/* TSR */
#define IT87_CIR_TSR_TXFBC    0x3f

/* RSR */
#define IT87_CIR_RSR_RXFTO    0x80
#define IT87_CIR_RSR_RXFBC    0x3f

/* IIR */
#define IT87_CIR_IIR_II       0x6
#define IT87_CIR_IIR_RXFO     0x6
#define IT87_CIR_IIR_RXDS     0x4
#define IT87_CIR_IIR_TXLDL    0x2
#define IT87_CIR_IIR_IP       0x1
#define IT87_CIR_IIR_NOINT    0x0

#endif // _ITECIRREG_H_
