//NIC abstraction code, based on work by Louis Beaudoin 
#ifndef __NIC_H__
#define __NIC_H__

#include <main.h>
#include <uip.h>


void nic_init(SPI_TypeDef* SPIx, uint8_t *eth_addr);
void nic_send(void);
uint16_t nic_poll(void);
uint8_t nic_sending(void);

#endif /* __NIC_H__ */
