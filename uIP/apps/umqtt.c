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

#include <string.h>
#include <uip.h>

#include "umqtt.h"
/*
//static
inline uint16_t htons(uint16_t x)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return ((x & 0xff) << 8) | (x >> 8);
#else
  return x;
#endif
}
*/
#define umqtt_insert_messageid(conn, ptr) \
	do { \
		ptr[0] = conn->message_id >> 8;		\
		ptr[1] = conn->message_id & 0xff;	\
		conn->message_id++;					\
	} while (0)

#define umqtt_build_header(type, dup, qos, retain) \
	(((type) << 4) | ((dup) << 3) | ((qos) << 1) | (retain))

#define umqtt_header_type(h) \
	((h) >> 4)

   
static uint16_t umqtt_decode_length(uint8_t *data)
{
  uint16_t mul = 1;
  uint16_t val = 0;

  for (uint16_t i = 0; i == 0 || (data[i - 1] & 0x80); i++)
  {
    val += (data[i] & 0x7f) * mul;
    mul <<= 7;          // * 128;
  }

  return val;
}

static uint16_t umqtt_encode_length(uint16_t len, uint8_t *data)
{
  uint16_t digit;
  uint16_t i = 0;

  do
  {
    digit = len % 128;
    len >>= 7;          // / 128;
		
    if (len > 0)
      digit |= 0x80;
	
    data[i++] = digit;
  } while (len);

  return i; /* Return the amount of bytes used */
}

void umqtt_circ_init(struct umqtt_circ_buffer *buff)
{
  buff->pointer = buff->start;
  buff->datalen = 0;
}

int16_t umqtt_circ_push(struct umqtt_circ_buffer *buff, uint8_t *data, uint16_t len)
{
  uint8_t *bend = buff->start + buff->length - 1;
  uint8_t *dend = (buff->pointer - buff->start + buff->datalen)
		% buff->length + buff->start; /* This points to new byte */

  for (; len > 0; len--)
  {
    if (dend > bend)
      dend = buff->start;
	
    if (buff->datalen != 0 && dend == buff->pointer)
      break;
	
    *dend = *data;
    dend++;
    data++;
    buff->datalen++;
  }

  return len; /* Return amount of bytes left */
}

int16_t umqtt_circ_peek(struct umqtt_circ_buffer *buff, uint8_t *data, uint16_t len)
{
  uint16_t i;
  uint8_t *ptr = buff->pointer;
  uint8_t *bend = buff->start + buff->length - 1;

  for (i = 0; i < len && i < buff->datalen; i++)
  {
    data[i] = ptr[i];
	
    if (ptr > bend)
      ptr = buff->start;
  }

  return i; /* Return the amount of bytes actually peeked */
}

int16_t umqtt_circ_pop(struct umqtt_circ_buffer *buff, uint8_t *data, uint16_t len)
{
  uint16_t i;
  uint8_t *bend = buff->start + buff->length - 1;

  for (i = 0; i < len && buff->datalen > 0; i++)
  {
    data[i] = *buff->pointer;
    buff->pointer++;
    buff->datalen--;

    if (buff->pointer > bend)
      buff->pointer = buff->start;
  }

  return i; /* Return the amount of bytes actually popped */
}

void umqtt_init(struct umqtt_connection *conn)
{
  conn->state = UMQTT_STATE_INIT;
  conn->nack_ping = 0;
  conn->nack_publish = 0;
  conn->nack_subscribe = 0;
  conn->message_id = 1; /* Id 0 is reserved */
}

void umqtt_connect(struct umqtt_connection *conn)
{
  uint16_t cidlen = strlen(conn->clientid);
  uint8_t remlen[4];
  uint8_t variable[12];

  umqtt_circ_init(&conn->rxbuff);
  umqtt_circ_init(&conn->txbuff);
  conn->state = UMQTT_STATE_INIT;
  conn->nack_ping = 0;
  conn->nack_publish = 0;
  conn->nack_subscribe = 0;
  conn->message_id = 1;

  uint8_t fixed = umqtt_build_header(UMQTT_CONNECT, 0, 0, 0);

  variable[0] = 0; /* UTF Protocol name */
  variable[1] = 6;
  variable[2] = 'M';
  variable[3] = 'Q';
  variable[4] = 'I';
  variable[5] = 's';
  variable[6] = 'd';
  variable[7] = 'p';
  variable[8] = 3; /* Protocol version */
  variable[9] = (1 << 1); /* Clean session flag */
  variable[10] = conn->kalive >> 8; /* Keep Alive timer */
  variable[11] = conn->kalive & 0xff;
  
  uint16_t paylen = htons(cidlen);
  umqtt_circ_push(&conn->txbuff, &fixed, 1);
  umqtt_circ_push(&conn->txbuff, remlen, umqtt_encode_length(sizeof(variable) + sizeof(paylen) + cidlen, remlen));
  umqtt_circ_push(&conn->txbuff, variable, sizeof(variable));
  umqtt_circ_push(&conn->txbuff, (uint8_t *) &paylen, sizeof(paylen));
  umqtt_circ_push(&conn->txbuff, (uint8_t *) conn->clientid, cidlen);

  conn->state = UMQTT_STATE_CONNECTING;
	
  if (conn->new_packet_callback)
    conn->new_packet_callback(conn);
}

void umqtt_subscribe(struct umqtt_connection *conn, char *topic)
{
  uint16_t topiclen = strlen(topic);
  uint8_t remlen[4];
  uint8_t messageid[2];
  uint8_t qos = 0;

  uint8_t fixed = umqtt_build_header(UMQTT_SUBSCRIBE, 0, 1, 0);

  umqtt_insert_messageid(conn, messageid);

  uint16_t paylen = htons(topiclen);

  umqtt_circ_push(&conn->txbuff, &fixed, 1);
  umqtt_circ_push(&conn->txbuff, remlen,
                  umqtt_encode_length(
                  sizeof(messageid) + sizeof(paylen) + topiclen + sizeof(qos), remlen));
  umqtt_circ_push(&conn->txbuff, messageid, sizeof(messageid));
  umqtt_circ_push(&conn->txbuff, (uint8_t *) &paylen, sizeof(paylen));
  umqtt_circ_push(&conn->txbuff, (uint8_t *) topic, topiclen);
  umqtt_circ_push(&conn->txbuff, &qos, sizeof(qos));

  conn->nack_subscribe++;

  if (conn->new_packet_callback)
    conn->new_packet_callback(conn);
}

void umqtt_publish(struct umqtt_connection *conn, char *topic, uint8_t *data, uint16_t datalen)
{
  uint16_t toplen = strlen(topic);
  uint8_t remlen[4];
  uint8_t len[2];

  uint8_t fixed = umqtt_build_header(UMQTT_PUBLISH, 0, 0, 0);

  len[0] = toplen >> 8;
  len[1] = toplen & 0xff;
  
  umqtt_circ_push(&conn->txbuff, &fixed, 1);
  umqtt_circ_push(&conn->txbuff, remlen, umqtt_encode_length(2 + toplen + datalen, remlen));
  umqtt_circ_push(&conn->txbuff, len, sizeof(len));
  umqtt_circ_push(&conn->txbuff, (uint8_t *) topic, toplen);
  umqtt_circ_push(&conn->txbuff, data, datalen);

  if (conn->new_packet_callback)
    conn->new_packet_callback(conn);
}

void umqtt_ping(struct umqtt_connection *conn)
{
  uint8_t packet[] = { umqtt_build_header(UMQTT_PINGREQ, 0, 0, 0), 0 };

  umqtt_circ_push(&conn->txbuff, packet, sizeof(packet));
  conn->nack_ping++;

  if (conn->new_packet_callback)
    conn->new_packet_callback(conn);
}

static void umqtt_handle_publish(struct umqtt_connection *conn, char *data, uint16_t len)
{
  uint16_t toplen = (data[0] << 8) | data[1];
  uint16_t payloadlen = len - 2 - toplen;

  if (toplen > UMQTT_TOPIC_LEN)
    return;
  if (payloadlen > UMQTT_PAYLOAD_LEN)
    return;
        
  char topic[UMQTT_TOPIC_LEN + 1];
  char payload[UMQTT_PAYLOAD_LEN + 1];

  memcpy(topic, data + 2, toplen);
  topic[toplen] = 0;              // ������� ����� ������
  memcpy(payload, data + 2 + toplen, payloadlen);
  payload[payloadlen] = 0;        // ������� ����� ������

  conn->message_callback(conn, topic, payload);
}

static void umqtt_packet_arrived(struct umqtt_connection *conn,	uint8_t header, uint16_t len)
{
  uint8_t data[UMQTT_PAKET_LEN];

  if (len > UMQTT_PAKET_LEN)
    return;

  umqtt_circ_pop(&conn->rxbuff, data, len);

  switch (umqtt_header_type(header)) 
  {
  case UMQTT_CONNACK:
    if (data[1] == 0x00)
    {
      conn->state = UMQTT_STATE_CONNECTED;
      
      if (conn->connected_callback)
        conn->connected_callback(conn);
    }
    else
      conn->state = UMQTT_STATE_FAILED;
    break;
  case UMQTT_DISCONNECT:
    conn->state = UMQTT_STATE_DISCONNECTED;
    break;
  case UMQTT_SUBACK:
    conn->nack_subscribe--;
    break;
  case UMQTT_PINGRESP:
    conn->nack_ping--;
    break;
  case UMQTT_PUBLISH:
    umqtt_handle_publish(conn, (char *) data, len);
    break;
  }
}

void umqtt_process(struct umqtt_connection *conn)
{
  uint8_t buf[5];
  uint16_t i = 2;

  while (conn->rxbuff.datalen >= 2)
  { /* We do have the fixed header */
    umqtt_circ_pop(&conn->rxbuff, buf, 2);
    
    for (i = 2; buf[i - 1] & 0x80 && i < sizeof(buf); i++)
      umqtt_circ_pop(&conn->rxbuff, &buf[i], 1);
  
    umqtt_packet_arrived(conn, buf[0],
    umqtt_decode_length(&buf[1]));
  }
}

void umqtt_appcall()
{
  struct umqtt_connection *conn = uip_conn->appstate.conn;

  uint16_t bufflen = uip_mss() > (uint16_t) conn->txbuff.datalen ? (uint16_t) conn->txbuff.datalen : uip_mss();
  uint8_t buff[UMQTT_PAKET_LEN];
  int16_t ret;

  if (uip_closed() || uip_aborted() || uip_timedout())
  {
    umqtt_disconnected(conn);
    uip_close();
    return;
  }

  if (uip_newdata()) 
  {
    umqtt_circ_push(&conn->rxbuff, uip_appdata, uip_datalen());
    umqtt_process(conn);
  }

  if (uip_poll() || uip_acked())
  {
    ret = umqtt_circ_pop(&conn->txbuff, buff, bufflen);
	
    if (!ret)
      return;
	
    uip_send(buff, ret);
  }
}

void umqtt_disconnected(struct umqtt_connection *conn)
{
  conn->state = UMQTT_STATE_DISCONNECTED;
}

// assume topic filter and name is in correct format
// # can only be at end
// + and # can only be next to separator
uint8_t umqtt_isTopicMatched(char *topicFilter, char *topicName)
{
  char* curf = topicFilter;
  char* curn = topicName;
  char* curn_end = curn + strlen(topicName);
    
  while (*curf && curn < curn_end)
  {
    if (*curn == '/' && *curf != '/')
      break;
  
    if (*curf != '+' && *curf != '#' && *curf != *curn)
      break;
  
    if (*curf == '+')
    {   // skip until we meet the next separator, or end of string
      char* nextpos = curn + 1;
  
      while (nextpos < curn_end && *nextpos != '/')
        nextpos = ++curn + 1;
    }
    else if (*curf == '#')
      curn = curn_end - 1;    // skip until end of string
  
    curf++;
    curn++;
  }
    
  return (curn == curn_end) && (*curf == '\0');
}
