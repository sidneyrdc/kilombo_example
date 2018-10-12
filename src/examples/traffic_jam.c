/* Test to Kilombo functions <Implementation>
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2018 Sep 07 22:08:19
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

    mydata->cycle_dt = 10;
}

// put your main code here, will be run repeatedly
void loop() {
    //receive messages
    receive_inputs();

    // select the leader
    if(kilo_uid == 0) {
        set_color(RGB(3, 0, 0));
    } else {
        /*uint32_t dt = kilo_uid == 1 ? 1 : 5;*/
        uint8_t min_dist = 45;
        uint8_t j = UINT8_MAX;
        int32_t noise = rand_soft()%3;
        /*uint8_t dist = get_dist_by_id(kilo_uid == 6 ? 1 : kilo_uid+1, HIST);*/

        // collision avoidance
        if(find_nearest_neighbor(&j) < min_dist && (j == kilo_uid+1 || (kilo_uid == 6 && j == 1))) {
        /*if(dist < min_dist) {*/
            set_motion(STOP);
            mydata->cycle_dt = 10;
            printf("robot=%d neighbor=%d\n", kilo_uid, j);
        } else {
            printf("robot=%d cycle_time=%d time_error=%d noise=%d\n", kilo_uid, mydata->cycle_dt, kilo_ticks - mydata->timestamp, noise);

            // change the average speed by implementing a stop-motion fashion
            if(kilo_ticks - mydata->timestamp < (mydata->cycle_dt+noise)*32) {
                circulate_leader();
            } else if(kilo_ticks - mydata->timestamp < (mydata->cycle_dt+noise+2)*32) {
                set_motion(STOP);
            } else {
                mydata->timestamp = kilo_ticks;
                mydata->cycle_dt = mydata->cycle_dt > 1 ? mydata->cycle_dt-1 : 1;
            }
        }
    }

    // search for a light source
    /*goto_light();*/

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

    if(light_dist <= light_radius) return (1 - light_dist/light_radius)*1023;

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

