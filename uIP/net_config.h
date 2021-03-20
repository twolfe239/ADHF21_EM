#ifndef __NET_CONFIG_H__
#define __NET_CONFIG_H__


// ���������� ����� ������ ENC28J60
#define ENC28J60_ENABLE_DUPLEX  1
// uIP buffer size (maximum packat size, DHCP needs at least 400bytes)
#define UIP_CONF_BUFFER_SIZE    400
// MAC-�����
#define ETHADDR0        0x00 // The first octet of the Ethernet address 
#define ETHADDR1        0xbd // The second octet of the Ethernet address
#define ETHADDR2        0x3b // The third octet of the Ethernet address 
#define ETHADDR3        0x33 // The fourth octet of the Ethernet address
#define ETHADDR4        0x05 // The fifth octet of the Ethernet address 
#define ETHADDR5        0x71 // The sixth octet of the Ethernet address 

#define ENABLE_DHCP     1
// ��������� IP (���� DHCP ��������)
#define IPADDR0         192  // The first octet of the IP address of this uIP node
#define IPADDR1         168  // The second octet of the IP address of this uIP node
#define IPADDR2         0    // The third octet of the IP address of this uIP node
#define IPADDR3         13	 // The fourth octet of the IP address of this uIP node
#define NETMASK0        255  // The first octet of the netmask of this uIP node
#define NETMASK1        255  // The second octet of the netmask of this uIP node
#define NETMASK2        255  // The third octet of the netmask of this uIP node
#define NETMASK3        0    // The fourth octet of the netmask of this uIP node
#define DRIPADDR0       192  // The first octet of the IP address of the default router
#define DRIPADDR1       168  // The second octet of the IP address of the default router
#define DRIPADDR2       0    // The third octet of the IP address of the default router
#define DRIPADDR3       1    // The fourth octet of the IP address of the default router

// Serial Debug configuration
#define DEBUG_UMQTT     1
#define DEBUG_UIP       1

// enable uip split hack - to circumvent slow-down due to delayed ACK algorithm
// (will send each tcp packet in two halves)
#define UIP_SPLIT_HACK          0
// enable empty packet - to circumvent slow-down due to delayed ACK algorithm
// (will send each tcp packet in two halves) - alternative to UIP_SPLIT_HACK
#define UIP_EMPTY_PACKET_HACK   1

// which mechanism to use for protothreads
//#define LC_CONF_INCLUDE "lc-addrlabels.h" // using special GCC feature (uses sligtly less program memory)
#define LC_CONF_INCLUDE "lc-switch.h" // using switch statements (standard)

// UIP debug messages 
#define DEBUG_PRINTF(...) /*printf(__VA_ARGS__)*/

#endif //__NET_CONFIG_H__
