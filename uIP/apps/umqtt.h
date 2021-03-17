/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2014-2015 Josef Gajdusek
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * */

#ifndef __UMQTT_H__
#define __UMQTT_H__

#include <stdint.h>

enum umqtt_packet_type
{
  UMQTT_CONNECT		= 1,
  UMQTT_CONNACK		= 2,
  UMQTT_PUBLISH		= 3,
  UMQTT_SUBSCRIBE	= 8,
  UMQTT_SUBACK		= 9,
  UMQTT_UNSUBSCRIBE	= 10,
  UMQTT_UNSUBACK	= 11,
  UMQTT_PINGREQ		= 12,
  UMQTT_PINGRESP	= 13,
  UMQTT_DISCONNECT	= 14,
};

enum umqtt_client_state
{
  UMQTT_STATE_INIT,
  UMQTT_STATE_CONNECTING,
  UMQTT_STATE_CONNECTED,
  UMQTT_STATE_FAILED,
  UMQTT_STATE_DISCONNECTED
};

struct umqtt_circ_buffer
{
  uint8_t *start;
  uint16_t length;
  uint8_t *pointer;
  uint16_t datalen;
};

struct umqtt_connection
{
  struct umqtt_circ_buffer txbuff;
  struct umqtt_circ_buffer rxbuff;
  uint16_t kalive;
  char *clientid;

  void (*connected_callback)(struct umqtt_connection *);
  void (*message_callback)(struct umqtt_connection *, char *topic, char *data);
  void (*new_packet_callback)(struct umqtt_connection *);

  // ack counters - incremented on sending, decremented on ack
  uint16_t nack_publish;
  uint16_t nack_subscribe;
  uint16_t nack_ping;

  uint16_t message_id;

  uint8_t work_buf[5];
  uint16_t work_read;

  enum umqtt_client_state state;
};

struct nethandler_state
{
  struct umqtt_connection *conn;
  uint16_t slen; /* Length of data currently being sent (fox rxmit) */
};

typedef struct nethandler_state uip_tcp_appstate_t;



#define UMQTT_TOPIC_LEN       32
#define UMQTT_PAYLOAD_LEN     64
#define UMQTT_PAKET_LEN       128

#define umqtt_circ_datalen(buff)        ((buff)->datalen)
#define umqtt_circ_is_full(buff) 	((buff)->length == (buff)->datalen)
#define umqtt_circ_is_empty(buff)	(umqtt_circ_datalen(buff) == 0)

void umqtt_circ_init(struct umqtt_circ_buffer *buff);

/* Return the amount of bytes left */
int16_t umqtt_circ_push(struct umqtt_circ_buffer *buff, uint8_t *data, uint16_t len);

/* Returns amount of bytes popped/peeked */
int16_t umqtt_circ_pop(struct umqtt_circ_buffer *buff, uint8_t *data, uint16_t len);
int16_t umqtt_circ_peek(struct umqtt_circ_buffer *buff, uint8_t *data, uint16_t len);

void umqtt_init(struct umqtt_connection *conn);
void umqtt_connect(struct umqtt_connection *conn);
void umqtt_subscribe(struct umqtt_connection *conn, char *topic);
void umqtt_publish(struct umqtt_connection *conn, char *topic, uint8_t *data, uint16_t datalen);
void umqtt_ping(struct umqtt_connection *conn);
void umqtt_process(struct umqtt_connection *conn);
void umqtt_appcall();
void umqtt_disconnected(struct umqtt_connection *conn);
uint8_t umqtt_isTopicMatched(char *topicFilter, char *topicName);

#endif /* __UMQTT_H__ */
