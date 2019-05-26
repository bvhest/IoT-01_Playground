#ifndef RF24L01_H
#define RF24L01_H

#include <stm8s.h> //Required for the stdint typedefs

#define RF24L01_reg_CONFIG      0x00
#define RF24L01_reg_EN_AA       0x01
#define RF24L01_reg_EN_RXADDR   0x02
#define RF24L01_reg_SETUP_AW    0x03    // Setup of Address Widths
#define RF24L01_reg_SETUP_RETR  0x04
#define RF24L01_reg_RF_CH       0x05
#define RF24L01_reg_RF_SETUP    0x06
#define RF24L01_reg_STATUS      0x07
#define RF24L01_reg_OBSERVE_TX  0x08
#define RF24L01_reg_CD          0x09
#define RF24L01_reg_RX_ADDR_P0  0x0A
#define RF24L01_reg_RX_ADDR_P1  0x0B
#define RF24L01_reg_RX_ADDR_P2  0x0C
#define RF24L01_reg_RX_ADDR_P3  0x0D
#define RF24L01_reg_RX_ADDR_P4  0x0E
#define RF24L01_reg_RX_ADDR_P5  0x0F
#define RF24L01_reg_TX_ADDR     0x10
#define RF24L01_reg_RX_PW_P0    0x11
#define RF24L01_reg_RX_PW_P1    0x12
#define RF24L01_reg_RX_PW_P2    0x13
#define RF24L01_reg_RX_PW_P3    0x14
#define RF24L01_reg_RX_PW_P4    0x15
#define RF24L01_reg_RX_PW_P5    0x16
#define RF24L01_reg_FIFO_STATUS 0x17
#define RF24L01_reg_DYNPD       0x1C
#define RF24L01_reg_FEATURE     0x1D

#define RF24L01_command_R_REGISTER            0x00 // Plus 5 bit register address
#define RF24L01_command_W_REGISTER            0x20 // Plus 5 bit register address
#define RF24L01_command_R_RX_PAYLOAD          0x61
#define RF24L01_command_W_TX_PAYLOAD          0xA0
#define RF24L01_command_FLUSH_TX              0xE1
#define RF24L01_command_FLUSH_RX              0xE2
#define RF24L01_command_REUSE_TX_PL           0xE3
#define RF24L01_command_R_RX_PL_WID           0x60
#define RF24L01_command_W_ACK_PAYLOAD         0xA8
#define RF24L01_command_W_TX_PAYLOAD_NOACK    0x58
#define RF24L01_command_NOP                   0xFF


void RF24L01_init(void) ;
void RF24L01_setup ( uint8_t *nw_name, uint8_t channel ) ;
void RF24L01_set_mode_TX ( uint8_t node ) ;
void RF24L01_set_mode_RX ( uint8_t node ) ;
uint8_t RF24L01_get_status(void) ;
void RF24L01_read_payload ( uint8_t *data, uint8_t length ) ;
void RF24L01_write_payload ( uint8_t *data, uint8_t length ) ;
uint8_t RF24L01_clear_interrupts(void);
uint8_t RF24L01_read_register ( uint8_t register_addr ) ;
void RF24L01_write_1_reg ( uint8_t register_addr, uint8_t value ) ;
void RF24L01_auto_ack ( bool enable ) ;

#endif