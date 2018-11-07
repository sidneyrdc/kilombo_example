/* Main code to programming on Kilombo Simulator
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2018 Nov 07 09:45:05
 */

#include <kilombo.h>
#include <math.h>

// debug.h isn't a header file at all, it has function implementations, so be
// careful to not include it at header files to avoid multiple definition errors.
#ifndef SIMULATOR
#define DEBUG                          // for printf to serial port
#include <debug.h>
#endif

#define MAXN 5                        // maximum number of neighbours
#define MAXMSG 3                      // maximum number of stored messages
#define PI 3.14159265358979323846      // definition for π

// neighbor datatype
typedef struct {
    uint16_t id;
    uint8_t dist;
    uint8_t n_neighbors;
    uint32_t timestamp;
} neighbor_t;

// received message type
typedef struct {
    message_t msg;
    uint8_t dist;
} received_message_t;

// declare global variables
typedef struct {
    neighbor_t neighbors[MAXN];
    uint8_t n_neighbors;
    char new_message;
    char message_lock;
    message_t transmit_msg;
    received_message_t received_msg[MAXMSG];
    uint16_t n_messages;
} USERDATA;

// register individual variables: the simulator ensures that this pointer points to
// the data of the correct robot before calling any of the user program's functions.
REGISTER_USERDATA(USERDATA);

// fill the message with the bot information
void setup_message() {
    // don't transmit while we are forming the message
    mydata->message_lock = 1;
    mydata->transmit_msg.type = NORMAL;

    // ------------------------------------------------------------------------
    // put the data to be send below (limited to 9 bytes)
    // ------------------------------------------------------------------------
    mydata->transmit_msg.data[0] = kilo_uid & 0xff;            // 0 low  id
    mydata->transmit_msg.data[1] = kilo_uid >> 8;              // 1 high id
    mydata->transmit_msg.data[2] = mydata->n_neighbors;        // 2 number of neighbors
    // ------------------------------------------------------------------------

    mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
    mydata->message_lock = 0;
}

// process the received message
void process_message() {
    // auxiliary index
    int8_t i;

    // process all the received messages
    while(mydata->n_messages > 0) {
        // get the received data
        uint8_t *data = mydata->received_msg[mydata->n_messages-1].msg.data;

        // get sender id
        uint16_t id = data[0] | (data[1] << 8);

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
        }

        // --------------------------------------------------------------------
        // extract the received data below (limited to 9 bytes)
        // --------------------------------------------------------------------
        mydata->neighbors[i].id = id;
        mydata->neighbors[i].timestamp = kilo_ticks;
        mydata->neighbors[i].dist = mydata->received_msg[mydata->n_messages-1].dist;
        mydata->neighbors[i].n_neighbors = data[2];
        // --------------------------------------------------------------------

        // decrease the number of received messages
        mydata->n_messages--;
    }

    // go through the list of neighbors, remove entries older than a threshold,
    // currently 2 seconds.
    for(i = mydata->n_neighbors-1; i >= 0; i--) {
        // 32 ticks = 1 s
        if (kilo_ticks - mydata->neighbors[i].timestamp > 2*32) {
            // this one is too old.
            mydata->neighbors[i] = mydata->neighbors[mydata->n_neighbors-1];

            // replace it by the last entry
            mydata->n_neighbors--;
        }
    }

    // nullify the new message indicator
    mydata->new_message = 0;
}

// callback for message reception. This callback is triggered every time a message
// is successfully decoded
void message_rx(message_t *msg, distance_measurement_t *d) {
    // set new message indicator as true
    mydata->new_message = 1;

    // increase the number of received messages
    mydata->n_messages = mydata->n_messages < MAXMSG-1 ? mydata->n_messages + 1 : 1;

    // store received message
    mydata->received_msg[mydata->n_messages-1].msg = *msg;
    mydata->received_msg[mydata->n_messages-1].dist = estimate_distance(d);
}

// callback for message transmission. This callback is triggered every time a
// message is scheduled for transmission (roughly twice every second)
message_t *message_tx() {
    // if the message is being built, don't send it
    if(mydata->message_lock) return 0;

    return &mydata->transmit_msg;
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

    // show the neighbours and their distances to the current robot
    for(size_t j = 0; j < mydata->n_neighbors; j++) {
        if(j + 1 < mydata->n_neighbors) str_len = sprintf(str_p, "%d:%d,", mydata->neighbors[j].id, mydata->neighbors[j].dist);
        else str_len = sprintf(str_p, "%d:%d", mydata->neighbors[j].id, mydata->neighbors[j].dist);

        str_p += str_len;
    }

    // get current pose of the robot
    pose_t pose = get_pose();

    // show the robot coordinates on Cartesian space
    str_len = sprintf(str_p, "] pose=[x:%.2f,y:%.2f,yaw:%.2f] light=%d", pose.x, pose.y, pose.yaw*180/PI, get_ambientlight());
    str_p += str_len;

    return botinfo_buffer;
}
#endif

/******************************************************************************
 * Editable Part of the Code
 * Put your personal code in the functions bellow.
 *****************************************************************************/

// process the distance between the robots and set their colors according with
// that
void process_distance() {
    uint8_t i;
    uint8_t min_dist = UINT8_MAX;

    // search for the minimum distance among the neighbors
    for(i = 0; i < mydata->n_neighbors; i++) {
        if(mydata->neighbors[i].dist < min_dist) min_dist = mydata->neighbors[i].dist;
    }

    // define the led color according to the minimum received distance from its
    // neighbors
    if(min_dist < 40) {
        // if minimum distance is lower than 40 mm, set the led color as blue
        set_color(RGB(0, 0, 3));

    } else if (min_dist < 70) {
        // if the minimum distance is lower than 70 mm, set the led color as green
        set_color(RGB(0, 3, 0));

    } else {
        // if the minimum distance is greater than 70 mm, set the led color as red
        set_color(RGB(3, 0, 0));
    }
}

// put your setup code here, to be run only once
void setup() {
    // set the number of neighbors to zero
    mydata->n_neighbors = 0;

    // set the number of messages to zero
    mydata->n_messages = 0;
}

// put your main code here, will be run repeatedly
void loop() {
    // build message
    setup_message();

    // process received message
    process_message();

    // change the led colors according to the distance between the robots
    process_distance();
/*    if(mydata->n_neighbors == 1) set_color(RGB(0, 3, 0));*/
    /*else if(mydata->n_neighbors == 2) set_color(RGB(0, 0, 3));*/
    /*else if(mydata->n_neighbors == 0) set_color(RGB(0, 0, 0));*/
    /*else set_color(RGB(3, 0, 0));*/
}

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

    // register message callback
    kilo_message_rx = message_rx;

    // register message transmission callback
    kilo_message_tx = message_tx;

    // start kilobot event loop
    kilo_start(setup, loop);

    return 0;
}

