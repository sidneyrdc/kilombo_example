/******************************************************************************
 * Personal Data Types for Kilombo Simulator <Header>
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2018 Sep 26 12:28:12
 * Info: This file contains personal datatypes needed to work with
 * the Kilobots and Kilombo simulator.
 *****************************************************************************/

#ifndef DATATYPE_H
#define DATATYPE_H

#include <kilombo.h>

#define MAXN 20     // maximum number of neighbours
#define HIST 30      // history size (used to compute integrals)
#define RB_SIZE 16  // Ring buffer size. Choose a power of two for faster code
                    // memory usage: 16*RB_SIZE
                    // 8 works too, but complains in the simulator
                    // when the bots are very dense

#define PI 3.14159265358979323846

// robot possible types
enum BOTTYPE {LAST, FOLLOWER, LEADER};

// robot possible states
enum BOTSTATES {WAIT, LISTEN, MOVE, ROTATE};

// robot possible movements
enum BOTMOVES {LEFT, RIGHT, STOP, STRAIGHT};

// received message type
typedef struct {
    message_t msg;
    distance_measurement_t dist;
} received_message_t;

// neighbour type
typedef struct {
    uint16_t id;
    uint8_t dist[HIST];
    uint8_t iter;

    uint16_t gradient;

    uint8_t bot_state;
    uint8_t move_type;

    uint8_t n_neighbors;
    uint32_t timestamp;
} neighbor_t;

// shared variables, access using a pointer to robot_data
typedef struct {
    neighbor_t neighbors[MAXN];
    uint8_t n_neighbors;
    uint8_t iter;               // allows the robots to know in which iteration from 0 to HIST they are

    uint8_t bot_type;
    uint8_t bot_state[HIST];
    uint8_t move_type[HIST];
    uint32_t move_timestamp;    // stores the time at which some movement is taken

    int32_t cycle_dt;

    uint16_t gradient;
    uint32_t timestamp;
    int16_t light_level[HIST];
    message_t transmit_msg;
    char message_lock;

    received_message_t rx_buffer[RB_SIZE];
    uint8_t rx_head, rx_tail;
} robot_data;

// borrow the declaration of mydata from 'kilombo.h' using the type 'robot_data'
// (after the call of 'REGISTER_USERDATA(type)', a struct of the passed type called
// mydata is created and used to store individual variables for each robot, like
// state, status, etc.)
extern robot_data *mydata;

#endif

