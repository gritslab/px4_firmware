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
#include <poll.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_local_position.h>


//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------
static bool thread_should_exit = false;     /**< daemon exit flag */
static bool thread_running = false;     /**< daemon status flag */
static int daemon_task;             /**< Handle of daemon task / thread */

//------------------------------------------------------------------------------
// Function Definitions
//------------------------------------------------------------------------------
__EXPORT int position_estimator_vicon_main(int argc, char *argv[]);
int position_estimator_vicon_thread_main(int argc, char *argv[]);
static void usage(const char *reason);


//------------------------------------------------------------------------------
// Function Implementations
//------------------------------------------------------------------------------
static void usage(const char *reason) {
    if (reason)
        warnx("%s\n", reason);
    errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

int position_estimator_vicon_main(int argc, char *argv[])
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
                     position_estimator_vicon_thread_main,
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

int position_estimator_vicon_thread_main(int argc, char *argv[])
{
    warnx("[position_estimator_vicon] starting\n");
    thread_running = true;

    // subscribe to topics
    struct vehicle_vicon_position_s vicon_position;
    memset(&vicon_position, 0, sizeof(vicon_position));
    int vicon_position_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
    orb_set_interval(vicon_position_sub, 50); // Limit topic to 20 Hz (1000/5)

    /* advertise */
    struct vehicle_local_position_s local_pos;
    memset(&local_pos, 0, sizeof(local_pos));
    orb_advert_t vehicle_local_position_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);

    struct pollfd fds[] = {
        { .fd = vicon_position_sub, .events = POLLIN }
    };

    bool updated = false;
    int error_counter = 0;
    while (!thread_should_exit) {
        int poll_ret = poll(fds, 1, 50); // wait 50 ms for update

        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                printf("[px4_simple_app] ERROR return value from poll(): %d\n"
                    , poll_ret);
            }
            error_counter++;
        } else {

            hrt_abstime t = hrt_absolute_time();

            /* check for vicon position estimate */
            orb_check(vicon_position_sub, &updated);
            if (updated) {
                /* position changed */
                orb_copy(ORB_ID(vehicle_vicon_position), vicon_position_sub, &vicon_position);

                local_pos.timestamp = t;
                local_pos.xy_valid = true;
                local_pos.z_valid = true;
                local_pos.v_xy_valid = false;
                local_pos.v_z_valid = false;
                local_pos.x = vicon_position.x;
                local_pos.y = vicon_position.y;
                local_pos.z = vicon_position.z;
                local_pos.vx = 0;
                local_pos.vy = 0;
                local_pos.vz = 0;
                local_pos.yaw = vicon_position.yaw;
                local_pos.xy_global = false;
                local_pos.z_global = false;
                local_pos.landed = true;
                local_pos.dist_bottom_valid = true;
                local_pos.dist_bottom = -local_pos.z;
                local_pos.dist_bottom_rate = -local_pos.vz;

                orb_publish(ORB_ID(vehicle_local_position), vehicle_local_position_pub, &local_pos);


                printf("Vicon position:\nt: %8.4f\nx: %8.4f\ny: %8.4f\nz: %8.4f\nroll: %8.4f\npitch: %8.4f\nyaw: %8.4f\n",
                       (double)t,
                       (double)vicon_position.x,
                       (double)vicon_position.y,
                       (double)vicon_position.z,
                       (double)vicon_position.roll,
                       (double)vicon_position.pitch,
                       (double)vicon_position.yaw);
            }
        }
    }

    warnx("[position_estimator_vicon] exiting\n");
    thread_running = false;
    return 0;
}
