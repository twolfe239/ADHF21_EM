/************************************************************
*
*	This is the general driver for the ENC28J60
*	I changed some things to make it work with uIP
*	Some files of uIP have changes too.
*
*								edi87 [at] fibertel.com.ar
*								Jonathan Granade
*
************************************************************/
#ifndef ENC28J60_H
#define ENC28J60_H
#include <spi.h>
#include <stdint.h>
#include "uip-conf.h"



#ifdef UIP_CONF_BUFFER_SIZE
#define ENC28J60_MAX_FRAMELEN	(UIP_CONF_BUFFER_SIZE+18)	// maximum ethernet frame length
#endif //UIP_CONF_BUFFER_SIZE









#define SPI_PORT		hspi1


//// ���� ��� ����������� � ENC28J60 (����� ��� SCLK, MOSI, MISO)
// CS
#define ENC28J60_CS_PORT	GPIOA
#define ENC28J60_CS_PIN	        (1 << 4)
// RESET
#define ENC28J60_RESET_PORT	GPIOA
#define ENC28J60_RESET_PIN	(1 << 3)
// INT
#define ENC28J60_INT_PORT	GPIOA
#define ENC28J60_INT_PIN	(1 << 2)


#define ENC28J60_ERRATA_B7
extern SPI_HandleTypeDef SPI_PORT;


uint8_t SPI_SendRecvByte(uint8_t TxByte);
//! perform software reset
void enc28j60SoftwareReset(void);
//! do a ENC28J60 read operation
uint8_t enc28j60ReadOp(uint8_t op, uint8_t address);
//! do a ENC28J60 write operation
void enc28j60WriteOp(uint8_t op, uint8_t address, uint8_t data);
//! read the packet buffer memory
void enc28j60ReadBuffer(uint16_t len, uint8_t* data);
//! write the packet buffer memory
void enc28j60WriteBuffer(uint16_t len, uint8_t* data);
//! set the register bank for register at address
void enc28j60SetBank(uint8_t address);
//! read ENC28J60 register 8bit
uint8_t enc28j60Read(uint8_t address);
//! write ENC28J60 register 8bit
void enc28j60Write(uint8_t address, uint8_t data);
//! read ENC28J60 register 8bit
uint16_t enc28j60Read16(uint8_t address);
//! write ENC28J60 register 8bit
void enc28j60Write16(uint8_t address, uint16_t data);
//! read a PHY register
uint16_t enc28j60PhyRead(uint8_t address);
//! write a PHY register
void enc28j60PhyWrite(uint8_t address, uint16_t data);

//! initialize the ethernet interface for transmit/receive
void enc28j60Init(uint8_t *eth_addr, uint8_t DuplexState);

//dummy ?
void enc28j60BeginPacketSend(uint16_t packetLength);
void enc28j60EndPacketSend(void);
uint8_t enc28j60PollPacketSending(void);
//

//! Packet transmit function.
/// Sends a packet on the network.  It is assumed that the packet is headed by a valid ethernet header.
/// \param len		Length of packet in bytes.
/// \param packet	Pointer to packet data.
void enc28j60PacketSend(uint8_t* packet, uint16_t len);
//void enc28j60PacketSend2(unsigned int len1, unsigned char* packet1, unsigned int len2, unsigned char* packet2);

//! Packet receive function.
/// Gets a packet from the network receive buffer, if one is available.
/// The packet will by headed by an ethernet header.
/// \param	maxlen	The maximum acceptable length of a retrieved packet.
/// \param	packet	Pointer where packet data should be stored.
/// \return Packet length in bytes if a packet was retrieved, zero otherwise.
uint16_t enc28j60BeginPacketReceive(void);
void enc28j60EndPacketReceive(void);
void enc28j60PacketReceive(unsigned char* packet, uint16_t maxlen);

//! execute procedure for recovering from a receive overflow
/// this should be done when the receive memory fills up with packets
void enc28j60ReceiveOverflowRecover(void);

uint8_t enc28j60getrev(void);
uint8_t enc28j60linkup(void);

// ��������� ���������� ������ �����������
void enc28j60ResetTxLogic(void);

//! formatted print of important ENC28J60 registers
void enc28j60RegDump(void);

#endif
