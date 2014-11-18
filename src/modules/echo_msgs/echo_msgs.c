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
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_command.h>


//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------
static bool thread_should_exit = false;     /**< daemon exit flag */
static bool thread_running = false;     /**< daemon status flag */
static int daemon_task;             /**< Handle of daemon task / thread */

static bool vehicle_vicon_position_flag = false;
static bool vehicle_status_flag = false;
static bool safety_flag = false;
static bool rc_channels_flag = false;
static bool vehicle_command_flag = false;

//------------------------------------------------------------------------------
// Function Definitions
//------------------------------------------------------------------------------
__EXPORT int echo_msgs_main(int argc, char *argv[]);
int echo_msgs_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

void print_vehicle_vicon_position(struct vehicle_vicon_position_s vehicle_vicon_position_data);
void print_vehicle_status(struct vehicle_status_s vehicle_status_data);
void print_safety(struct safety_s safety_data);
void print_rc_channels(struct rc_channels_s rc_channels_data);
void print_vehicle_command(struct vehicle_command_s vehicle_command_data);


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

        if (vehicle_vicon_position_flag) {
            warnx("\tvehicle_vicon_position: on");
        } else {
            warnx("\tvehicle_vicon_position: off");
        }

        if (vehicle_status_flag) {
            warnx("\tvehicle_status: on");
        } else {
            warnx("\tvehicle_status: off");
        }

        if (safety_flag) {
            warnx("\tsafety: on");
        } else {
            warnx("\tsafety: off");
        }

        if (rc_channels_flag) {
            warnx("\trc_channels: on");
        } else {
            warnx("\trc_channels: off");
        }

        if (vehicle_command_flag) {
            warnx("\tvehicle_command: on");
        } else {
            warnx("\tvehicle_command: off");
        }

        exit(0);
    }

    if (!strcmp(argv[1], "vehicle_vicon_position")) {
        if (vehicle_vicon_position_flag) {
            vehicle_vicon_position_flag = false;
            warnx("\tvehicle_vicon_position: off");
        } else {
            vehicle_vicon_position_flag = true;
            warnx("\tvehicle_vicon_position: on");
        }
        exit(0);
    }

    if (!strcmp(argv[1], "vehicle_status")) {
        if (vehicle_status_flag) {
            vehicle_status_flag = false;
            warnx("\tvehicle_status: off");
        } else {
            vehicle_status_flag = true;
            warnx("\tvehicle_status: on");
        }
        exit(0);
    }

    if (!strcmp(argv[1], "safety")) {
        if (safety_flag) {
            safety_flag = false;
            warnx("\tsafety: off");
        } else {
            safety_flag = true;
            warnx("\tsafety: on");
        }
        exit(0);
    }

    if (!strcmp(argv[1], "rc_channels")) {
        if (rc_channels_flag) {
            rc_channels_flag = false;
            warnx("\trc_channels: off");
        } else {
            rc_channels_flag = true;
            warnx("\trc_channels: on");
        }
        exit(0);
    }

    if (!strcmp(argv[1], "vehicle_command")) {
        if (vehicle_command_flag) {
            vehicle_command_flag = false;
            warnx("\tvehicle_command: off");
        } else {
            vehicle_command_flag = true;
            warnx("\tvehicle_command: on");
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
    int vehicle_vicon_position_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
    struct vehicle_vicon_position_s vehicle_vicon_position_data;
    memset(&vehicle_vicon_position_data, 0, sizeof(vehicle_vicon_position_data));

    int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    struct vehicle_status_s vehicle_status_data;
    memset(&vehicle_status_data, 0, sizeof(vehicle_status_data));

    int safety_sub = orb_subscribe(ORB_ID(safety));
    struct safety_s safety_data;
    memset(&safety_data, 0, sizeof(safety_data));

    int rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
    struct rc_channels_s rc_channels_data;
    memset(&rc_channels_data, 0, sizeof(rc_channels_data));

    int vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
    struct vehicle_command_s vehicle_command_data;
    memset(&vehicle_command_data, 0, sizeof(vehicle_command_data));

    bool updated = false;
    while (!thread_should_exit) {

        orb_check(vehicle_vicon_position_sub, &updated);
        if (updated && vehicle_vicon_position_flag) {
            orb_copy(ORB_ID(vehicle_vicon_position), vehicle_vicon_position_sub, &vehicle_vicon_position_data);
            print_vehicle_vicon_position(vehicle_vicon_position_data);
        }

        orb_check(vehicle_status_sub, &updated);
        if (updated & vehicle_status_flag) {
            orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status_data);
            print_vehicle_status(vehicle_status_data);
        }

        orb_check(safety_sub, &updated);
        if (updated & safety_flag) {
            orb_copy(ORB_ID(safety), safety_sub, &safety_data);
            print_safety(safety_data);
        }

        orb_check(rc_channels_sub, &updated);
        if (updated & rc_channels_flag) {
            orb_copy(ORB_ID(rc_channels), rc_channels_sub, &rc_channels_data);
            print_rc_channels(rc_channels_data);
        }

        orb_check(vehicle_command_sub, &updated);
        if (updated & vehicle_command_flag) {
            orb_copy(ORB_ID(vehicle_command), vehicle_command_sub, &vehicle_command_data);
            print_vehicle_command(vehicle_command_data);
        }

        usleep(500000);
    }

    warnx("[echo_msgs] exiting\n");
    thread_running = false;
    return 0;
}

void print_vehicle_vicon_position(struct vehicle_vicon_position_s vehicle_vicon_position_data)
{
    printf("vehicle_vicon_position:\n\tx: %8.4f\n\ty: %8.4f\n\tz: %8.4f\n\troll: %8.4f\n\tpitch: %8.4f\n\tyaw: %8.4f\n",
                   (double)vehicle_vicon_position_data.x,
                   (double)vehicle_vicon_position_data.y,
                   (double)vehicle_vicon_position_data.z,
                   (double)vehicle_vicon_position_data.roll,
                   (double)vehicle_vicon_position_data.pitch,
                   (double)vehicle_vicon_position_data.yaw);
}

void print_vehicle_status(struct vehicle_status_s vehicle_status_data)
{
    printf("vehicle_status:\n\tsystem_type: %d\n\tsystem_id: %d\n\tcomponent_id: %d\n\tmain_state: %d\n\tarming_state: %d\n\t",
           vehicle_status_data.system_type,
           vehicle_status_data.system_id,
           vehicle_status_data.component_id,
           vehicle_status_data.main_state,
           vehicle_status_data.arming_state);
}

void print_safety(struct safety_s safety_data)
{
    printf("safety:\n\tsafety_switch_available: %d\n\tsafety_off: %d\n",
           safety_data.safety_switch_available,
           safety_data.safety_off);
}

void print_rc_channels(struct rc_channels_s rc_channels_data)
{
    printf("rc_channels:\n");
    for (int i=0; i<RC_CHANNELS_FUNCTION_MAX; i++) {
        printf("\tchannel %d: %8.4f\n", i, (double)rc_channels_data.channels[i]);
    }
    printf("\trssi: %d\n", rc_channels_data.rssi);
}

void print_vehicle_command(struct vehicle_command_s vehicle_command_data)
{
    printf("vehicle_command:\n");
    printf("\tcommand: %d\n", vehicle_command_data.command);
    printf("\tparam1: %8.4f\n", (double)vehicle_command_data.param1);
    printf("\tparam2: %8.4f\n", (double)vehicle_command_data.param2);
    printf("\tparam3: %8.4f\n", (double)vehicle_command_data.param3);
    printf("\tparam4: %8.4f\n", (double)vehicle_command_data.param4);
    printf("\tparam5: %8.4f\n", (double)vehicle_command_data.param5);
    printf("\tparam6: %8.4f\n", (double)vehicle_command_data.param6);
    printf("\tparam7: %8.4f\n", (double)vehicle_command_data.param7);
    printf("\ttarget_system: %d\n", vehicle_command_data.target_system);
    printf("\ttarget_component: %d\n", vehicle_command_data.target_component);
    printf("\tsource_system: %d\n", vehicle_command_data.source_system);
    printf("\tsource_component: %d\n", vehicle_command_data.source_component);
}
