// modified for the ENC28J60 Rev. B7 Silicon Errata
//  Vladimir S. Fonov ~ vladimir.fonov <at> gmail.com
//
// File Name	: 'enc28j60.c'
// Title		: Microchip ENC28J60 Ethernet Interface Driver
// Author		: Pascal Stang (c)2005
// Created		: 9/22/2005
// Revised		: 9/22/2005
// Version		: 0.1
// Editor Tabs	: 4
//
// Description	: This driver provides initialization and transmit/receive
//	functions for the Microchip ENC28J60 10Mb Ethernet Controller and PHY.
// This chip is novel in that it is a full MAC+PHY interface all in a 28-pin
// chip, using an SPI interface to the host processor.
//

/************************************************************
*
*	This is the general driver for the ENC28J60
*	I changed some things to make it work with uIP
*	Some files of uIP have changes too.
*
*	edi87 [at] fibertel.com.ar
*	Jonathan Granade
*
************************************************************/

#include <gpio.h>
#include <spi.h>
#include "uip.h"
#include "enc28j60.h"
#include "enc28j60def.h"
#include "uip-conf.h"
#ifdef ENC28J60_ERRATA_B7
#endif //ENC28J60_ERRATA_B7


#define ENC28J60_CS_LOW()       HAL_GPIO_WritePin(ENC28J60_CS_PORT, ENC28J60_CS_PIN, RESET)
#define ENC28J60_CS_HIGH()      HAL_GPIO_WritePin(ENC28J60_CS_PORT, ENC28J60_CS_PIN, SET)
#define ENC28J60_RESET_LOW()    HAL_GPIO_WritePin(ENC28J60_RESET_PORT, ENC28J60_RESET_PIN, RESET)
#define ENC28J60_RESET_HIGH()   HAL_GPIO_WritePin(ENC28J60_RESET_PORT, ENC28J60_RESET_PIN, SET)

uint8_t Enc28j60Bank;
uint16_t NextPacketPtr;




//#define ENC28J60_SPIx hspi1



uint8_t SPI_SendRecvByte(uint8_t TxByte)
{
	uint8_t RxByte;
	HAL_SPI_TransmitReceive(&SPI_PORT, &TxByte, &RxByte,1,10000);
/*
	HAL_SPI_Transmit(&SPI_PORT, TxByte, 1, 10000);
	HAL_SPI_Receive(&SPI_PORT, RxByte, 1, 10000);*/
  return RxByte;
}
//==============================================================================




uint8_t enc28j60ReadOp(uint8_t op, uint8_t address)
{
  // assert CS
  ENC28J60_CS_LOW();
  // issue read command
  SPI_SendRecvByte(op | (address & ADDR_MASK));
  // read data
  uint8_t data = SPI_SendRecvByte(0x00);
  // do dummy read if needed
  if (address & 0x80)
    data = SPI_SendRecvByte(0x00);
  // release CS
  ENC28J60_CS_HIGH();

  return data;
}

void enc28j60WriteOp(uint8_t op, uint8_t address, uint8_t data)
{
  // assert CS
  ENC28J60_CS_LOW();





  // issue write command
  SPI_SendRecvByte(op | (address & ADDR_MASK));
  // write data
  SPI_SendRecvByte(data);
  // release CS
  ENC28J60_CS_HIGH();
}

void enc28j60ReadBuffer(uint16_t len, uint8_t* data)
{
  // assert CS
  ENC28J60_CS_LOW();

  /*
  	HAL_SPI_Transmit(&SPI_PORT, TxByte, 1, 10000);
  	HAL_SPI_Receive(&SPI_PORT, RxByte, 1, 10000);*/


  // issue read command
  SPI_SendRecvByte(ENC28J60_READ_BUF_MEM);
  // read data
  while(len--)
    *data++ = SPI_SendRecvByte(0x00);
  // release CS
  ENC28J60_CS_HIGH();
}

void enc28j60WriteBuffer(uint16_t len, uint8_t* data)
{
  // assert CS
  ENC28J60_CS_LOW();

  /*
  	HAL_SPI_Transmit(&SPI_PORT, TxByte, 1, 10000);
  	HAL_SPI_Receive(&SPI_PORT, RxByte, 1, 10000);*/

  // issue write command
  SPI_SendRecvByte(ENC28J60_WRITE_BUF_MEM);
  // write data
  while(len--)
    SPI_SendRecvByte(*data++);
  // release CS
  ENC28J60_CS_HIGH();
}










void enc28j60SetBank(uint8_t address)
{
  // set the bank (if needed)
  if ((address & BANK_MASK) != Enc28j60Bank)
  {
    // set the bank
    enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
    enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
    Enc28j60Bank = (address & BANK_MASK);
  }
}

uint8_t enc28j60Read(uint8_t address)
{
  // set the bank
  enc28j60SetBank(address);
  // do the read
  return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
}

uint16_t enc28j60Read16(uint8_t address)
{
  uint16_t data = enc28j60Read(address);
  data |= enc28j60Read(address + 1) << 8;
  return data;
}

void enc28j60Write(uint8_t address, uint8_t data)
{
  // set the bank
  enc28j60SetBank(address);
  // do the write
  enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

void enc28j60Write16(uint8_t address, uint16_t data)
{
  enc28j60Write(  // INT
		  //gpio_PortClockStart(ENC28J60_INT_PORT);
		  //gpio_SetGPIOmode_In(ENC28J60_INT_PORT, ENC28J60_INT_PIN, gpio_PullUp);
		  // ������������� SPI
address, data & 0xff);
  enc28j60Write(address + 1, data >> 8);
}

uint16_t enc28j60PhyRead(uint8_t address)
{
  uint16_t data;

  // Set the right address and start the register read operation
  enc28j60Write(MIREGADR, address);
  enc28j60Write(MICMD, MICMD_MIIRD);

  // wait until the PHY read completes
  while (enc28j60Read(MISTAT) & MISTAT_BUSY)
    ;

  // quit reading
  enc28j60Write(MICMD, 0x00);
	
  // get data value
  data  = enc28j60Read(MIRDL);
  data <<= 8;
  data |= enc28j60Read(MIRDH);

  return data;
}

void enc28j60PhyWrite(uint8_t address, uint16_t data)
{
  // set the PHY register address
  enc28j60Write(MIREGADR, address);
	
  // write the PHY data
  enc28j60Write(MIWRL, data);	
  enc28j60Write(MIWRH, data >> 8);
  
  // wait until the PHY write completes
  while (enc28j60Read(MISTAT) & MISTAT_BUSY)
    ;
}


void enc28j60SoftwareReset(void)
{
  // perform system reset
  enc28j60WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
  
#ifdef ENC28J60_ERRATA_B7
  HAL_Delay(1); //wait for 1 ms
#else
  // check CLKRDY bit to see if reset is complete
  HAL_Delay(1);
  while (!(enc28j60Read(ESTAT) & ESTAT_CLKRDY))
    ;
#endif //ENC28J60_ERRATA_B7
}


void enc28j60Init(uint8_t *eth_addr, uint8_t DuplexState)
{

  
  // CS
  ENC28J60_CS_HIGH();
  // RESET
  ENC28J60_RESET_HIGH();

  enc28j60SoftwareReset();
  
  // do bank 0 stuff
  // initialize receive buffer
  // 16-bit transfers, must write low byte first
  // set receive buffer start address
  NextPacketPtr = ENC28J60_RXSTART_INIT;
    
  enc28j60Write16(ERXSTL, ENC28J60_RXSTART_INIT);
  // set receive pointer address
  enc28j60Write16(ERXRDPTL, ENC28J60_RXSTART_INIT);
  // set receive buffer end
  enc28j60Write16(ERXNDL, ENC28J60_RXSTOP_INIT);
  // set transmit buffer start
  enc28j60Write16(ETXSTL, ENC28J60_TXSTART_INIT);
  
  // allow Unicast to US and Broad cast packets with correct CRC
  enc28j60Write(ERXFCON, ERXFCON_CRCEN | ERXFCON_UCEN | ERXFCON_BCEN);//ERXFCON_UCEN||ERXFCON_PMEN

  // do bank 2 stuff
  // enable MAC receive
  enc28j60Write(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);
  // bring MAC out of reset
  enc28j60Write(MACON2, 0x00);
  // enable automatic padding and CRC operations
  enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN);
  // enc28j60Write(MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
  // set inter-frame gap (non-back-to-back)
  enc28j60Write(MAIPGL, 0x12);
  enc28j60Write(MAIPGH, 0x0C);//?
  // set inter-frame gap (back-to-back)
  enc28j60Write(MABBIPG, 0x12);
  // Set the maximum packet size which the controller will accept
  enc28j60Write16(MAMXFLL, ENC28J60_MAX_FRAMELEN);	

  // do bank 3 stuff
  // write MAC address
  // NOTE: MAC address in ENC28J60 is byte-backward
  enc28j60Write(MAADR5, eth_addr[0]);
  enc28j60Write(MAADR4, eth_addr[1]);
  enc28j60Write(MAADR3, eth_addr[2]);
  enc28j60Write(MAADR2, eth_addr[3]);
  enc28j60Write(MAADR1, eth_addr[4]);
  enc28j60Write(MAADR0, eth_addr[5]);

  // no loopback of transmitted frames
  enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);
  
  // 0x476 is PHLCON LEDA=links status, LEDB=receive/transmit
  enc28j60PhyWrite(PHLCON, 0x476);

  // switch to bank 0
  // enable interrutps
  enc28j60Write(EIE, EIE_INTIE | EIE_PKTIE);
  // enable packet reception
  enc28j60Write(ECON1, ECON1_RXEN);

  //enc28j60PhyWrite(PHLCON, 0x0AA2);

  // setup duplex ----------------------

  // Disable receive logic and abort any packets currently being transmitted
  enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS | ECON1_RXEN);
  
  if (DuplexState)
  {
    uint16_t temp;
    // Set the PHY to the proper duplex mode
    temp = enc28j60PhyRead(PHCON1);
    temp &= ~PHCON1_PDPXMD;
    enc28j60PhyWrite(PHCON1, temp);
    // Set the MAC to the proper duplex mode
    temp = enc28j60Read(MACON3);
    temp &= ~MACON3_FULDPX;
    enc28j60Write(MACON3, temp);
  }

  // Set the back-to-back inter-packet gap time to IEEE specified 
  // requirements. The meaning of the MABBIPG value changes with the duplex
  // state, so it must be updated in this function.
  // In full duplex, 0x15 represents 9.6us; 0x12 is 9.6us in half duplex
  enc28j60Write(MABBIPG, DuplexState ? 0x15 : 0x12);	
	
  // Reenable receive logic
  enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);

  // setup duplex ----------------------
}

// read the revision of the chip:
uint8_t enc28j60getrev(void)
{
  return (enc28j60Read(EREVID));
}

// link status
uint8_t enc28j60linkup(void)
{
  // bit 10 (= bit 3 in upper reg)
  return (enc28j60PhyRead(PHSTAT2) && (1 << 10)) ? 1 : 0;
}

void enc28j60BeginPacketSend(uint16_t packetLength)
{
//
}

// ��������� ���������� ������ �����������
void enc28j60ResetTxLogic(void)
{
  enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
  enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
  enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF);
}

void enc28j60PacketSend(uint8_t * packet, uint16_t len)
{
#ifdef ENC28J60_ERRATA_B7
  // ������������ �� Errata B.7 - �������� ������ ����������� ����� �������� ��������
  enc28j60ResetTxLogic();
#endif //ENC28J60_ERRATA_B7
  
  // Set the write pointer to start of transmit buffer area
  enc28j60Write16(EWRPTL, ENC28J60_TXSTART_INIT);
  // Set the TXND pointer to correspond to the packet size given
  enc28j60Write16(ETXNDL, (ENC28J60_TXSTART_INIT + len));
	
  // write per-packet control byte
  enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);

  // copy the packet into the transmit buffer
  enc28j60WriteBuffer(len, packet);
  
  // send the contents of the transmit buffer onto the network
  enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}

uint8_t enc28j60PollPacketSending(void)
{
  return enc28j60Read(ECON1) & ECON1_TXRTS;
}

void enc28j60EndPacketSend(void)
{
//
}

uint16_t enc28j60BeginPacketReceive(void)
{
  uint16_t rxstat;
  uint16_t len;

  // check if a packet has been received and buffered
#ifdef ENC28J60_ERRATA_B7
  if (enc28j60Read(EPKTCNT) == 0)
    return 0;
#else
  if (!(enc28j60Read(EIR) & EIR_PKTIF))
    return 0;
#endif //ENC28J60_ERRATA_B7
  
  // Make absolutely certain that any previous packet was discarded	        
  //if( WasDiscarded == FALSE)
  //	MACDiscardRx();

  // Set the read pointer to the start of the received packet
  enc28j60Write16(ERDPTL, NextPacketPtr);
  
  // read the next packet pointer
  NextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
  NextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
  // read the packet length
  len  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
  len |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
  // read the receive status
  rxstat  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
  rxstat |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
  len-=4;
  
  //  limit retrieve length
  // (we reduce the MAC-reported length by 4 to remove the CRC)
  //	if(len>maxlen) len=maxlen;
  //	if(len<=0) return 0;
  //	else return len;
  if ((rxstat & 0x80) == 0)
  {
    // invalid
    len = 0;
  }

  return len;
}

void enc28j60PacketReceive(uint8_t * packet, uint16_t maxlen)
{
  // copy the packet from the receive buffer
  enc28j60ReadBuffer(maxlen, packet);
}

#define ENC28J60_RESET_PORT	GPIOA
void enc28j60EndPacketReceive(void)
{
#ifdef ENC28J60_ERRATA_B7
  if (((NextPacketPtr - 1) < ENC28J60_RXSTART_INIT) || ((NextPacketPtr - 1) > ENC28J60_RXSTOP_INIT))
    enc28j60Write16(ERXRDPTL, ENC28J60_RXSTOP_INIT);
  else
    enc28j60Write16(ERXRDPTL, NextPacketPtr - 1);
#endif //ENC28J60_ERRATA_B7
  
  // Move the RX read pointer to the start of the next received packet
  // This frees the memory we just read out
  // decrement the packet counter indicate we are done with this packet
  enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
}

void enc28j60ReceiveOverflowRecover(void)
{
  // receive buffer overflow handling procedure
  // recovery completed
}

void enc28j60RegDump(void)
{
#ifdef ENC28J60_DEBUG
  printf_P(PSTR("RevID: 0x%x\r\n"), enc28j60Read(EREVID));

  printf_P(PSTR("Cntrl: ECON1 ECON2 ESTAT  EIR  EIE\r\n"));
  printf_P(PSTR("       0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n"),
    enc28j60Read(ECON1),
    enc28j60Read(ECON2),
    enc28j60Read(ESTAT),
    enc28j60Read(EIR),
    enc28j60Read(EIE));

  printf_P(PSTR("PHY  : PHCON1  PHSTAT1 PHSTAT2 PHID1 PHID2 PHCON2 PHIE PHIR PHLCON\r\n"));
  printf_P(PSTR("       %04x    %04x    %04x    %04x  %04x   %04x  %04x %04x %04x\r\n"),
     enc28j60PhyRead(PHCON1),
     enc28j60PhyRead(PHSTAT1),
     enc28j60PhyRead(PHSTAT2),
     enc28j60PhyRead(PHHID1),
     enc28j60PhyRead(PHHID2),
     enc28j60PhyRead(PHCON2),
     enc28j60PhyRead(PHIE),
     enc28j60PhyRead(PHIR),
     enc28j60PhyRead(PHLCON));

  printf_P(PSTR("MAC  : MACON1  MACON3  MACON4  MAC-Address\r\n"));
  printf_P(PSTR("       0x%02x  0x%02x 0x%02x   %02x:%02x:%02x:%02x:%02x:%02x\r\n"),
     enc28j60Read(MACON1),
     enc28j60Read(MACON3),
     enc28j60Read(MACON4),
     enc28j60Read(MAADR5),
     enc28j60Read(MAADR4),
     enc28j60Read(MAADR3),
     enc28j60Read(MAADR2),
     enc28j60Read(MAADR1),
     enc28j60Read(MAADR0));

  printf_P(PSTR("Rx   : ERXST  ERXND  ERXWRPT ERXRDPT ERXFCON EPKTCNT MAMXFL\r\n"));
  printf_P(PSTR("       0x%04x 0x%04x 0x%04x   0x%04x 0x%02x 0x%02x 0x%04x\r\n"),
     enc28j60Read16(ERXSTL),
     enc28j60Read16(ERXNDL),
     enc28j60Read16(ERXWRPTL),
     (enc28j60Read16(ERXRDPTL),
     enc28j60Read(ERXFCON),
     enc28j60Read(EPKTCNT),
     enc28j60Read16(MAMXFLL));

  printf_P(PSTR("Tx   : ETXST  ETXND  MACLCON1 MACLCON2 MAPHSUP\r\n"));
  printf_P(PSTR("       0x%04x 0x%04x 0x%04x   0x%02x   0x%02x\r\n"),
     enc28j60Read16(ETXNDL), (enc28j60Read(MACLCON1)<<8)|enc28j60Read(ETXNDL),
#endif//ENC28J60_DEBUG     
}
