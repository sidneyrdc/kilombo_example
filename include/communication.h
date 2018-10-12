/******************************************************************************
 * Communication Module for Kilombo Simulator <Header>
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2018 Ago 14 17:01:45
 * Info: This file contains the communication functions to be used in Kilombo
 * and Kilobots.
 *****************************************************************************/

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "util.h"

// ring buffer operations. Taken from kilolib's ringbuffer.h
// but adapted for use with mydata->
// ring buffer operations indexed with head, tail
// these waste one entry in the buffer, but are interrupt safe:
//   * head is changed only in popfront
//   * tail is changed only in pushback
//   * RB_popfront() is to be called AFTER the data in RB_front() has been used
//   * head and tail indices are uint8_t, which can be updated atomically
//     - still, the updates need to be atomic, especially in RB_popfront()

#define RB_init() { \
    mydata->rx_head = 0; \
    mydata->rx_tail = 0; \
}

#define RB_empty() (mydata->rx_head == mydata->rx_tail)

#define RB_full()  ((mydata->rx_head+1)%RB_SIZE == mydata->rx_tail)

#define RB_front() mydata->rx_buffer[mydata->rx_head]

#define RB_back() mydata->rx_buffer[mydata->rx_tail]

#define RB_popfront() mydata->rx_head = (mydata->rx_head+1)%RB_SIZE;

#define RB_pushback() { \
    mydata->rx_tail = (mydata->rx_tail+1)%RB_SIZE; \
    if(RB_empty()) { \
        mydata->rx_head = (mydata->rx_head+1)%RB_SIZE; \
        printf("Full.\n"); \
    } \
}

// message rx callback function. Pushes message to ring buffer.
void rxbuffer_push(message_t *msg, distance_measurement_t *dist);

// message tx callback function
message_t *message_tx();

// process a received message at the front of the ring buffer.
// go through the list of neighbors. If the message is from a bot
// already in the list, update the information, otherwise
// add a new entry in the list
void process_message();

// go through the list of neighbors, remove entries older than a threshold,
// currently 2 seconds.
void purge_neighbors();

// fill the message with the bot information
void setup_message();

// receive messages and storage them in the ring buffer
void receive_inputs();

#endif

