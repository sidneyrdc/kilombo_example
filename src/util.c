/******************************************************************************
 * Utility Module for Kilombo Simulator <Implementation>
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2018 Sep 26 12:41:15
 * Info: This file contains utilities and extra functions needed to work with
 * the Kilobots and Kilombo simulator.
 *****************************************************************************/

#include "util.h"
#include "communication.h"

// a helper function for setting motor state.
// automatic spin-up of left/right motor, when necessary.
// the standard way with spinup_motors() causes a small but
// noticeable jump when a motor which is already running is spun up again.
void smooth_set_motors(uint8_t ccw, uint8_t cw) {
    // OCR2A = ccw;  OCR2B = cw;
#ifdef KILOBOT
    uint8_t l = 0, r = 0;

    // we want left motor on, and it's off
    if(ccw && !OCR2A) l = 0xff;

    // we want right motor on, and it's off
    if(cw && !OCR2B) r = 0xff;

    // at least one motor needs spin-up
    if(l || r) {
        set_motors(l, r);
        delay(15);
    }
#endif

    // spin-up is done, now we set the real value
    set_motors(ccw, cw);
}

// set the motors according to the move_type
void set_motion(uint8_t move_type) {
    // get current time
    uint32_t current_time = kilo_ticks;

    // check if the minimum time defined to the execution of each movement is complete
    if(current_time - mydata->move_timestamp >= DT_MOVE*32 || mydata->timestamp == 0) {

        // store the current time as the time at which the movement was settled
        mydata->move_timestamp = current_time;

        switch(move_type) {
        case LEFT:
            smooth_set_motors(kilo_turn_left, 0);
            break;
        case RIGHT:
            smooth_set_motors(0, kilo_turn_right);
            break;
        case STRAIGHT:
            smooth_set_motors(kilo_turn_left, kilo_turn_right);
            break;
        case STOP:
        default:
            smooth_set_motors(0, 0);

            // do not wait DT_MOVE if the movement is stop
            /*mydata->move_timestamp = kilo_ticks + DT_MOVE*32;*/
            break;
        }

        // store the performed movement
        set_move_type(move_type);

    // if the minimum time is not reached, then set the current movement as the
    // last one performed (yet performing)
    } else set_move_type(get_move_type(HIST));
}

// set current bot state
void set_state(uint8_t bot_state) {
    uint8_t i;

    if(mydata->timestamp == 0) for(i = 0; i < HIST; i++) mydata->bot_state[i] = bot_state;
    else mydata->bot_state[mydata->iter] = bot_state;
}

// get current bot state
uint8_t get_state() {
    return mydata->bot_state[mydata->iter];
}

// set current move type
void set_move_type(uint8_t move_type) {
    uint8_t i;

    if(mydata->timestamp == 0) for(i = 0; i < HIST; i++) mydata->move_type[i] = move_type;
    else mydata->move_type[mydata->iter] = move_type;
}

// get current move type
uint8_t get_move_type(int8_t iter) {
    // if iter = HIST, gets the current move type
    if(iter == HIST) return mydata->move_type[mydata->iter];

    // if iter is negative, then get the value at index iter+HIST (to simulate
    // a circular buffer)
    return iter < 0 ? mydata->move_type[iter+HIST] : mydata->move_type[iter];
}

// get neighbor distance
uint8_t get_neighbor_dist(uint8_t index, int8_t iter) {
    if(iter >= HIST) return mydata->neighbors[index].dist[mydata->neighbors[index].iter];

    return iter < 0 ? mydata->neighbors[index].dist[iter+HIST] : mydata->neighbors[index].dist[iter];
}

// get neighbor distance by its id
uint8_t get_dist_by_id(uint8_t id, int8_t iter) {
    uint8_t i = 0;

    for(i = 0; i < mydata->n_neighbors; i++) {
        if(mydata->neighbors[i].id == id) {
            if(iter >= HIST) return mydata->neighbors[id].dist[mydata->neighbors[id].iter];
            else return iter < 0 ? mydata->neighbors[id].dist[iter+HIST] : mydata->neighbors[id].dist[iter];
        }
    }

    // if the robot id is in the current robot vicinity, return infinity distance
    return UINT8_MAX;
}

// set current light level
void set_light_level(uint16_t light_level) {
    uint8_t i;

    if(mydata->timestamp == 0) for(i = 0; i < HIST; i++) mydata->light_level[i] = light_level;
    else mydata->light_level[mydata->iter] = light_level;
}

// get current light level
uint16_t get_light_level(int8_t iter) {
    // if iter = HIST, gets the current move type
    if(iter == HIST) return mydata->light_level[mydata->iter];

    /*printf("iter=%d look@=%d\n", iter, iter < 0 ? iter+HIST : iter);*/

    // if iter is negative, then get the value at index iter+HIST (to simulate
    // a circular buffer)
    return iter < 0 ? mydata->light_level[iter+HIST] : mydata->light_level[iter];
}

// increase iteration number
void inc_iter() {
    // increase the robot current iteration
    mydata->iter = mydata->iter < HIST-1 ? mydata->iter + 1 : 0;
    /*printf("id=%d  iter=%d\n", kilo_uid, mydata->iter);*/

    // set current new values as the previous ones
    set_light_level(get_light_level(mydata->iter-1));
    set_move_type(get_move_type(mydata->iter-1));
    /*set_gradient(get_gradient(mydata->iter-1));*/

    // save the current time
    mydata->timestamp = kilo_ticks;
}

// primitive behaviour edge following
void follow_edge() {
    uint8_t desired_dist = 55;

    if(find_nearest_neighbor(NULL) > desired_dist) {
        set_motion(RIGHT);
    } else {
        set_motion(LEFT);
    }
}

// primitive behaviour circulate the leader
void circulate_leader() {
    uint8_t desired_dist = 55;
    /*uint8_t min_dist = 50;*/
    uint8_t i;
    uint8_t dist_leader = UINT8_MAX;

    // find the leader
    for(i = 0; i < mydata->n_neighbors; i++) {
        if(mydata->neighbors[i].gradient == 1) dist_leader = get_neighbor_dist(i, HIST);
    }

    /*printf("robot=%d dist=%d\n", kilo_uid, dist_leader);*/

    if(dist_leader > desired_dist && dist_leader < UINT8_MAX) {
        set_motion(RIGHT);
    /*} else if(dist_leader < min_dist) {*/
        /*set_motion(LEFT);*/
    } else if(dist_leader < UINT8_MAX) {
        /*set_motion(STRAIGHT);*/
        set_motion(LEFT);
    }
}

// primitive behaviour gradient formation
void compute_gradient() {
    uint8_t i;
    uint16_t min = UINT16_MAX-1;

    // search for the lower gradient value from all its current neighbours
    for(i = 0; i < mydata->n_neighbors; i++) {
        if(mydata->neighbors[i].gradient < min) min = mydata->neighbors[i].gradient;
    }

    // if the current robot is the leader (id=0), then set the minimum value as zero
    if(kilo_uid == 0) {
        min = 0;
        mydata->bot_type = LEADER;
    }

    // increase the robot's current gradient value according with the minimum
    // value of the gradient found
    if(mydata->gradient != min+1) {
        mydata->gradient = min+1;
        setup_message();
        mydata->bot_type = FOLLOWER;
    }
}

// find the nearest neighbour
uint8_t find_nearest_neighbor(uint8_t *index) {
    uint8_t i;
    uint8_t dist = 90;

    for(i = 0; i < mydata->n_neighbors; i++) {
        if(mydata->neighbors[i].dist[mydata->neighbors[i].iter] < dist) {
            dist = mydata->neighbors[i].dist[mydata->neighbors[i].iter];

            // get the index of the nearest neighbour
            *index = index != NULL ? i : NULL;
        }
    }

    return dist;
}

// implements the l2-norm (euclidean norm) on 2D space
double norm(double dx, double dy) {
    return sqrt(pow(dx, 2) + pow(dy, 2));
}

// the ambient light sensor gives noisy readings. To mitigate this,
// we take the average of 300 samples in quick succession.
int16_t sample_light() {
    int16_t number_of_samples = 0;
    int16_t sum = 0;

    while(number_of_samples < 100) {
        int16_t sample = get_ambientlight();

        // -1 indicates a failed sample, which should be discarded.
        if (sample != -1) {
            sum = sum + sample;
            number_of_samples = number_of_samples + 1;
        }
    }

    // Compute the average.
    return sum/number_of_samples;
}

// primitive behaviour for light search
void goto_light() {
    // how much of the robot history to look at
    int8_t look_after = HIST-1;

    // get current light level
    int16_t current_light_level = sample_light();

    // current error between the light level reading and the robot past light reading
    int16_t light_error = current_light_level - get_light_level(mydata->iter-look_after);                                // e[k] = l[k] - l[k-1]

    // debug
    /*printf("id=%d iter=%d current_light=%d my_light=%d\n", kilo_uid, mydata->iter, current_light_level, get_light_level(HIST));*/

    // if the light level error is greater than zero, then the robot is going
    // to the light source, so it keeps going forward
    if(light_error > 0) {
        printf("robot %d is going to the light...\n", kilo_uid);
        set_motion(STRAIGHT);

    // if the light level error is lower than zero, then the robot is going far
    // from the light source, and it should perform the opposite movement take at
    // the 'look_after' iterations
    } else if(light_error < 0) {
        printf("robot %d is going to the dark... light_error=%d last_move=%d\n", kilo_uid, light_error, get_move_type(mydata->iter-look_after));
        set_motion(!get_move_type(mydata->iter-look_after));

    // if the light level error is zero, but the current light level is lower
    // than a threshold, move to the right
    } else if(current_light_level < THRESH_HI) {
        printf("robot %d should move to the RIGHT!\n", kilo_uid);
        set_motion(RIGHT);

    // stop the robot if the current light level is greater than a threshold
    } else {
        printf("robot %d is stopped!\n", kilo_uid);
        set_motion(STOP);
    }

    // store the current light level
    set_light_level(current_light_level);
}

// rainbow colors
uint8_t colors[] = {
    RGB(0, 0, 0),  //0 - off
    RGB(2, 0, 0),  //1 - red
    RGB(2, 1, 0),  //2 - orange
    RGB(2, 2, 0),  //3 - yellow
    RGB(1, 2, 0),  //4 - yellowish green
    RGB(0, 2, 0),  //5 - green
    RGB(0, 1, 1),  //6 - cyan
    RGB(0, 0, 1),  //7 - blue
    RGB(1, 0, 1),  //8 - purple
    RGB(3, 3, 3)   //9  - bright white
};

// primitive behaviour for collision avoidance
void collision_avoidance() {
    // minimum safest distance (to avoid collision)
    uint8_t min_dist = 50;

    // how much of the robot history to look at
    int8_t look_after = 3;

    // index of the nearest neighbour
    uint8_t nearest_neighbor = MAXN-1;

    // approximated noise
    uint8_t dist_noise = 4;

    // current minimum distance
    uint16_t current_min_dist = find_nearest_neighbor(&nearest_neighbor);

    if(current_min_dist < min_dist) {
        // compares the current minimum distance with the past values
        int16_t dist_error = get_neighbor_dist(nearest_neighbor, mydata->neighbors[nearest_neighbor].iter-look_after) - current_min_dist;

        // compensates the distance error by process the noise difference
        dist_error = abs(dist_error) > dist_noise ? dist_error : 0;

        printf("1 - dist(%d)=%d dist(%d)=%d error=%d\n", mydata->neighbors[nearest_neighbor].iter, current_min_dist,
                                                     mydata->neighbors[nearest_neighbor].iter-look_after < 0 ? mydata->neighbors[nearest_neighbor].iter-look_after+HIST : mydata->neighbors[nearest_neighbor].iter-look_after,
                                                     get_neighbor_dist(nearest_neighbor, mydata->neighbors[nearest_neighbor].iter-look_after), dist_error);

        printf("2 - robot=%d neighbor=%d dist=[", kilo_uid, mydata->neighbors[nearest_neighbor].id);
        uint8_t k = 0;
        for(k = 0; k < HIST; k++) {
            if(k < HIST - 1) printf("%d ", mydata->neighbors[nearest_neighbor].dist[k]);
            else printf("%d]\n", mydata->neighbors[nearest_neighbor].dist[k]);
        }

        // if the distance error is greater than zero, then the robot is going
        // to the nearest neighbor
        if(dist_error > 0) {
            printf("3 - robot %d is going towards robot %d. last_move=%d\n", kilo_uid, mydata->neighbors[nearest_neighbor].id, get_move_type(mydata->iter-look_after));
            set_motion(!get_move_type(mydata->iter-look_after));

        // if the distance error is lower than zero, then the robot is going far
        // from the nearest neighbor
        } else if(dist_error < 0) {
            printf("3 - robot %d is moving away from robot %d. last move=%d\n", kilo_uid, mydata->neighbors[nearest_neighbor].id, get_move_type(mydata->iter-look_after));
            set_motion(STOP);

        // if the distance error is zero, the robots are not moving
        } else {
            /*printf("3 - robots %d and %d are stopped!\n", kilo_uid, mydata->neighbors[nearest_neighbor].id);*/
        }
        /*set_motion(STRAIGHT);*/

        printf("-----------------------------------------------\n");
    } else {
        /*set_motion(STOP);*/
    }

}

// primitive behaviour follow the leader
void follow_leader() {
    uint8_t i;
    uint8_t dist_leader = 255;
    uint8_t dist_follower = 255;
    uint8_t state_leader = WAIT, state_follower = WAIT;
    uint16_t i_follower = UINT16_MAX, i_leader = UINT16_MAX;

    // find the nearest follower and leader
    for(i = 0; i < mydata->n_neighbors; i++) {
        if(mydata->neighbors[i].gradient == mydata->gradient-1 && dist_leader > get_neighbor_dist(i, HIST)) {
            i_leader = i;
            dist_leader = get_neighbor_dist(i, HIST);

        } else if(mydata->neighbors[i].gradient == mydata->gradient+1 && dist_follower > get_neighbor_dist(i, HIST)) {
            i_follower = i;
            dist_follower = get_neighbor_dist(i, HIST);
        }
    }

    printf("robot=%d follower=%d leader=%d\n", kilo_uid,
           i_follower != UINT16_MAX ? mydata->neighbors[i_follower].id : -1,
           i_leader != UINT16_MAX ? mydata->neighbors[i_leader].id : -1);
}

