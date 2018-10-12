/******************************************************************************
 * Utility Module for Kilombo Simulator <Header>
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2018 Sep 07 22:03:42
 * Info: This file contains utilities and extra functions needed to work with
 * the Kilobots and Kilombo simulator.
 *****************************************************************************/

#ifndef UTIL_H
#define UTIL_H

// constants for light following.
#define THRESH_LO 300
#define THRESH_HI 980

// constant to the duration of each movement (0 means it will be executed only
// for each iteration of the simulator)
#define DT_MOVE 0.1

#include <kilombo.h>
#include <math.h>
#include <stdlib.h>
#include "datatype.h"

#ifdef SIMULATOR
//#pragma simulator
#include <stdio.h>     // for printf
#else
#include <avr/io.h>
#define printf(...)
#endif

// set the motors according to the move_type
void set_motion(uint8_t move_type);

// set current bot state
void set_state(uint8_t bot_state);

// get current bot state
uint8_t get_state();

// set current move type
void set_move_type(uint8_t move_type);

// get current move type
uint8_t get_move_type(int8_t iter);

// get neighbor distance
uint8_t get_neighbor_dist(uint8_t index, int8_t iter);

// get neighbor distance by its id
uint8_t get_dist_by_id(uint8_t id, int8_t iter);

// set current light level
void set_light_level(uint16_t light_level);

// get current light level
uint16_t get_light_level(int8_t iter);

// increase iteration number
void inc_iter();

// a helper function for setting motor state.
// automatic spin-up of left/right motor, when necessary.
// the standard way with spinup_motors() causes a small but
// noticeable jump when a motor which is already running is spun up again.
void smooth_set_motors(uint8_t ccw, uint8_t cw);

// edge following behaviour
void follow_edge();

// primitive behaviour circulate the leader
void circulate_leader();

// primitive behaviour for gradient formation
void compute_gradient();

// find the nearest neighbour
uint8_t find_nearest_neighbor(uint8_t *index);

// implements the l2-norm (euclidean norm) in 2D space
double norm(double dx, double dy);

// the ambient light sensor gives noisy readings. To mitigate this,
// we take the average of 300 samples in quick succession.
int16_t sample_light();

// primitive behaviour for light search
void goto_light();

// primitive behaviour for collision avoidance
void collision_avoidance();

// primitive behaviour follow the leader
void follow_leader();

// rainbow colors
extern uint8_t colors[];

#endif

