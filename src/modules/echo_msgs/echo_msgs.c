/*
@author Rowland O'Flaherty
@date 11/06/2014
*/

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/topics/vehicle_vicon_position.h>


//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------
static bool thread_should_exit = false;     /**< daemon exit flag */
static bool thread_running = false;     /**< daemon status flag */
static int daemon_task;             /**< Handle of daemon task / thread */

//------------------------------------------------------------------------------
// Function Definitions
//------------------------------------------------------------------------------
__EXPORT int echo_msgs_main(int argc, char *argv[]);
int echo_msgs_thread_main(int argc, char *argv[]);
static void usage(const char *reason);


//------------------------------------------------------------------------------
// Function Implementations
//------------------------------------------------------------------------------
static void usage(const char *reason) {
    if (reason)
        warnx("%s\n", reason);
    errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

int echo_msgs_main(int argc, char *argv[])
{
    if (argc < 1)
        usage("missing command");

    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            warnx("daemon already running\n");
            /* this is not an error */
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = task_spawn_cmd("daemon",
                     SCHED_DEFAULT,
                     SCHED_PRIORITY_DEFAULT,
                     2000,
                     echo_msgs_thread_main,
                     (argv) ? (const char **)&argv[2] : (const char **)NULL);
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("\trunning\n");
        } else {
            warnx("\tnot started\n");
        }
        exit(0);
    }

    usage("unrecognized command");
    exit(1);
}

int echo_msgs_thread_main(int argc, char *argv[])
{
    warnx("[echo_msgs] starting\n");
    thread_running = true;

    // subscribe to topics
    int vicon_position_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
    struct vehicle_vicon_position_s vicon_position;
    memset(&vicon_position, 0, sizeof(vicon_position));

    bool updated = false;
    while (!thread_should_exit) {

        /* check for vicon position estimate */
        orb_check(vicon_position_sub, &updated);
        if (updated) {
            /* position changed */
            orb_copy(ORB_ID(vehicle_vicon_position), vicon_position_sub, &vicon_position);
            printf("Vicon position:\nx: %8.4f\ny: %8.4f\nz: %8.4f\nroll: %8.4f\npitch: %8.4f\nyaw: %8.4f\n",
                   (double)vicon_position.x,
                   (double)vicon_position.y,
                   (double)vicon_position.z,
                   (double)vicon_position.roll,
                   (double)vicon_position.pitch,
                   (double)vicon_position.yaw);
        }
        usleep(50000);
    }

    warnx("[echo_msgs] exiting\n");
    thread_running = false;
    return 0;
}
