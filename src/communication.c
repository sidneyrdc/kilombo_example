/******************************************************************************
 * Communication Module for Kilombo Simulator <Implementation>
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2018 Ago 24 21:32:01
 * Info: This file contains the communication functions to be used in Kilombo
 * and Kilobots.
 *****************************************************************************/

#include "communication.h"

// message rx callback function. Pushes message to ring buffer.
void rxbuffer_push(message_t *msg, distance_measurement_t *dist) {
    received_message_t *rmsg = &RB_back();
    rmsg->msg = *msg;
    rmsg->dist = *dist;
    RB_pushback();
}

// message tx callback function
message_t *message_tx() {
    if (mydata->message_lock) return 0;

    return &mydata->transmit_msg;
}

// process a received message at the front of the ring buffer.
// go through the list of neighbors. If the message is from a bot
// already in the list, update the information, otherwise
// add a new entry in the list
void process_message() {
    uint8_t i;
    uint16_t id;

    uint8_t *data = RB_front().msg.data;
    id = data[0] | (data[1] << 8);
    uint8_t d = estimate_distance(&RB_front().dist);

    // search the neighbor list by id
    for(i = 0; i < mydata->n_neighbors; i++) {
        // found it
        if(mydata->neighbors[i].id == id) break;
    }

    // this neighbor is not in list
    if(i == mydata->n_neighbors) {
        // if we have too many neighbors, we overwrite the last entry
        // sloppy but better than overflow
        if(mydata->n_neighbors < MAXN-1) mydata->n_neighbors++;

        // set initial distances
        uint8_t k = 0;
        for(k = 0; k < HIST; k++) mydata->neighbors[i].dist[k] = d;
        mydata->neighbors[i].iter = HIST;
    }

    // i now points to where this message should be stored
    mydata->neighbors[i].iter = mydata->neighbors[i].iter < HIST-1 ? mydata->neighbors[i].iter+1 : 0;
    mydata->neighbors[i].id = id;
    mydata->neighbors[i].timestamp = kilo_ticks;
    mydata->neighbors[i].dist[mydata->neighbors[i].iter] = d;
    mydata->neighbors[i].n_neighbors = data[2];
    mydata->neighbors[i].bot_state = data[3];
    mydata->neighbors[i].gradient = data[4] | data[5]<<8;
}

// go through the list of neighbors, remove entries older than a threshold,
// currently 2 seconds.
void purge_neighbors() {
    int8_t i;

    for (i = mydata->n_neighbors-1; i >= 0; i--) {
        // 32 ticks = 1 s
        if (kilo_ticks - mydata->neighbors[i].timestamp > 2*32) {
            //this one is too old.
            mydata->neighbors[i] = mydata->neighbors[mydata->n_neighbors-1];

            //replace it by the last entry
            mydata->n_neighbors--;
        }
    }
}

// fill the message with the bot information
void setup_message() {
    // don't transmit while we are forming the message
    mydata->message_lock = 1;
    mydata->transmit_msg.type = NORMAL;
    mydata->transmit_msg.data[0] = kilo_uid & 0xff;            // 0 low  id
    mydata->transmit_msg.data[1] = kilo_uid >> 8;              // 1 high id
    mydata->transmit_msg.data[2] = mydata->n_neighbors;        // 2 number of neighbors
    mydata->transmit_msg.data[3] = mydata->bot_state[mydata->iter];          // 3 bot state
    mydata->transmit_msg.data[4] = mydata->gradient&0xFF;      // 4 low  byte of gradient value
    mydata->transmit_msg.data[5] = (mydata->gradient>>8)&0xFF; // 5 high byte of gradient value

    mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
    mydata->message_lock = 0;
}

// receive messages and storage them in the ring buffer
void receive_inputs() {
    while(!RB_empty()) {
        process_message();
        RB_popfront();
    }

    // remove older neighbours (that don't send more messages)
    purge_neighbors();
}

