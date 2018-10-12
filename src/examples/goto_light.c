/* Test to Kilombo functions <Implementation>
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2018 Sep 26 12:41:24
 */

#include "communication.h"

// debug.h isn't a header file at all, it has function implementations, so be
// careful to not include it at header files to avoid multiple definition errors.
#ifndef SIMULATOR
#define DEBUG          // for printf to serial port
#include <debug.h>
#endif

// register individual variables: the simulator ensures that this pointer points to
// the data of the correct robot before calling any of the user program's functions.
REGISTER_USERDATA(robot_data);

// put your setup code here, to be run only once
void setup() {
    //seed the random number generator
    rand_seed(kilo_uid + 1);

    // set the current time at the robot
    mydata->timestamp = 0;

    // set initial iteration as zero
    mydata->iter = 0;

    // set robot type
    set_move_type(STOP);

    // set robot state
    set_state(LISTEN);

    // set message lock as 0
    mydata->message_lock = 0;

    // set the number of neighbours as zero
    mydata->n_neighbors = 0;

    // set the current light level
    set_light_level(0);

    // fill a message and send it
    setup_message();
}

// primitive behaviour for light search
void gotolight() {
    // maximum light level
    int16_t max_light = 155;

    // how much of the robot history to look at
    int8_t look_after = HIST-1;

    // get current light level
    int16_t current_light_level = sample_light();

    // current error between the light level reading and the robot past light reading
    int16_t light_error = current_light_level - get_light_level(mydata->iter-look_after);                                // e[k] = l[k] - l[k-1]

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
    } else if(current_light_level < max_light) {
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

// put your main code here, will be run repeatedly
void loop() {
    //receive messages
    receive_inputs();

    // select the leader
    if(kilo_uid == 0) {
        set_color(RGB(3, 0, 0));
    }

    // search for a light source
    gotolight();
    /*printf("light=%d\n", sample_light());*/

    // compute the gradient value
    compute_gradient();
    set_color(colors[mydata->gradient%10]);

    // fill and send a message
    setup_message();

    // increase iteration counter (used to monitors the history of each robot)
    inc_iter();
}

#ifdef SIMULATOR
// provide a text string for the status bar, about this bot
char *botinfo() {
    // string buffer
    static char botinfo_buffer[10000];

    // pointer to manipulate the showed string
    char *str_p = botinfo_buffer;

    // size of string returned from sprintf(number of char)
    size_t str_len;

    // concatenate the kilobot id and increase the pointer to insert the next string
    str_len = sprintf(str_p, "id=%d time=%.2f ", kilo_uid, kilo_ticks/32.0);
    str_p += str_len;

    str_len = sprintf(str_p, "N=[");
    str_p += str_len;

    // show the neighbours and their distance to the current robot
    for(size_t j = 0; j < mydata->n_neighbors; j++) {

        if(j + 1 < mydata->n_neighbors) str_len = sprintf(str_p, "%d:%d,", mydata->neighbors[j].id, get_neighbor_dist(j, HIST));
        else str_len = sprintf(str_p, "%d:%d", mydata->neighbors[j].id, get_neighbor_dist(j, HIST));

        str_p += str_len;
    }

    // get current pose of the robot
    pose_t pose = get_pose();

    str_len = sprintf(str_p, "] pose=[x:%.2f,y:%.2f,yaw:%.2f]\nlight=%d mylight=[", pose.x, pose.y, pose.yaw*180/PI, get_ambientlight());
    str_p += str_len;

    for(size_t i = 0; i < HIST; i++) {
        if(i + 1 < HIST) str_len = sprintf(str_p, "%lu:%d,", i, get_light_level(i));
        else str_len = sprintf(str_p, "%lu:%d]", i, get_light_level(i));

        str_p += str_len;
    }

    str_len = sprintf(str_p, " type=%d gradient=%d", mydata->bot_type, mydata->gradient);
    str_p += str_len;

    return botinfo_buffer;
}

// calculates light levels from x, y coordinates and calculate the light intensity
// in its neighbourhood according with the light radius and the robot's current
// distance to that
int16_t callback_lighting(double x, double y) {
    double light_x = 1;
    double light_y = 1;
    double light_radius = 1000;
    double light_dist = norm(x-light_x, y-light_y);

    if(light_dist <= light_radius) return (1 - light_dist/light_radius)*160;

    return 0;
}
#endif

// main function
int main(void) {
    // initialize the hardware of the kilobots
    kilo_init();

#ifdef DEBUG
    // setup debugging, i.e. printf to serial port, in real Kilobot
    debug_init();
    printf("debug ativado\n");
#endif

    // register a callback function to return a string describing the internal
    // state of the current bot, used for the simulator status bar
    SET_CALLBACK(botinfo, botinfo);

    // register a callback function to set the position of the light spot in
    // the map and calculate the light intensity in its neighbourhood according
    // with the light radius and the robot's current distance to that
    SET_CALLBACK(lighting, callback_lighting);

    // start kilobot event loop
    kilo_start(setup, loop);

    // initialize ring buffer
    RB_init();

    // register message callback
    kilo_message_rx = rxbuffer_push;

    // register message transmission callback
    kilo_message_tx = message_tx;

    return 0;
}

