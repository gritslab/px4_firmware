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
#include <string.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <drivers/drv_rc_input.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>


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
static bool actuator_armed_flag = false;
static bool manual_control_setpoint_flag = false;
static bool actuator_controls_flag = false;
static bool input_rc_flag = false;
static bool vehicle_control_mode_flag = false;
static bool vehicle_rates_setpoint_flag = false;

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
void print_actuator_armed(struct actuator_armed_s actuator_armed_data);
void print_manual_control_setpoint(struct manual_control_setpoint_s manual_control_setpoint_data);
void print_actuator_controls(struct actuator_controls_s actuator_controls_data);
void print_input_rc(struct rc_input_values input_rc_data);
void print_vehicle_control_mode(struct vehicle_control_mode_s vehicle_control_mode_data);
void print_vehicle_rates_setpoint(struct vehicle_rates_setpoint_s vehicle_rates_setpoint_data);

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
            return 0;
        }

        thread_should_exit = false;
        daemon_task = task_spawn_cmd("daemon",
                     SCHED_DEFAULT,
                     SCHED_PRIORITY_DEFAULT,
                     2000,
                     echo_msgs_thread_main,
                     (argv) ? (const char **)&argv[2] : (const char **)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("\t running\n");
        } else {
            warnx("\t not started\n");
        }

        if (vehicle_vicon_position_flag) {
            warnx("\t vehicle_vicon_position: on");
        } else {
            warnx("\t vehicle_vicon_position: off");
        }

        if (vehicle_status_flag) {
            warnx("\t vehicle_status: on");
        } else {
            warnx("\t vehicle_status: off");
        }

        if (safety_flag) {
            warnx("\t safety: on");
        } else {
            warnx("\t safety: off");
        }

        if (rc_channels_flag) {
            warnx("\t rc_channels: on");
        } else {
            warnx("\t rc_channels: off");
        }

        if (vehicle_command_flag) {
            warnx("\t vehicle_command: on");
        } else {
            warnx("\t vehicle_command: off");
        }

        if (actuator_armed_flag) {
            warnx("\t actuator_armed: on");
        } else {
            warnx("\t actuator_armed: off");
        }

        if (manual_control_setpoint_flag) {
            warnx("\t manual_control_setpoint: on");
        } else {
            warnx("\t manual_control_setpoint: off");
        }

        if (actuator_controls_flag) {
            warnx("\t actuator_controls: on");
        } else {
            warnx("\t actuator_controls: off");
        }

        if (input_rc_flag) {
            warnx("\t input_rc: on");
        } else {
            warnx("\t input_rc: off");
        }

        if (vehicle_control_mode_flag) {
            warnx("\t vehicle_control_mode: on");
        } else {
            warnx("\t vehicle_control_mode: off");
        }

        if (vehicle_rates_setpoint_flag) {
            warnx("\t vehicle_rates_setpoint: on");
        } else {
            warnx("\t vehicle_rates_setpoint: off");
        }

        return 0;
    }

    if (!strcmp(argv[1], "vehicle_vicon_position")) {
        if (vehicle_vicon_position_flag) {
            vehicle_vicon_position_flag = false;
            warnx("\t vehicle_vicon_position: off");
        } else {
            vehicle_vicon_position_flag = true;
            warnx("\t vehicle_vicon_position: on");
        }
        return 0;
    }

    if (!strcmp(argv[1], "vehicle_status")) {
        if (vehicle_status_flag) {
            vehicle_status_flag = false;
            warnx("\t vehicle_status: off");
        } else {
            vehicle_status_flag = true;
            warnx("\t vehicle_status: on");
        }
        return 0;
    }

    if (!strcmp(argv[1], "safety")) {
        if (safety_flag) {
            safety_flag = false;
            warnx("\t safety: off");
        } else {
            safety_flag = true;
            warnx("\t safety: on");
        }
        return 0;
    }

    if (!strcmp(argv[1], "rc_channels")) {
        if (rc_channels_flag) {
            rc_channels_flag = false;
            warnx("\t rc_channels: off");
        } else {
            rc_channels_flag = true;
            warnx("\t rc_channels: on");
        }
        return 0;
    }

    if (!strcmp(argv[1], "vehicle_command")) {
        if (vehicle_command_flag) {
            vehicle_command_flag = false;
            warnx("\t vehicle_command: off");
        } else {
            vehicle_command_flag = true;
            warnx("\t vehicle_command: on");
        }
        return 0;
    }

    if (!strcmp(argv[1], "actuator_armed")) {
        if (actuator_armed_flag) {
            actuator_armed_flag = false;
            warnx("\t actuator_armed: off");
        } else {
            actuator_armed_flag = true;
            warnx("\t actuator_armed: on");
        }
        return 0;
    }

    if (!strcmp(argv[1], "manual_control_setpoint")) {
        if (manual_control_setpoint_flag) {
            manual_control_setpoint_flag = false;
            warnx("\t manual_control_setpoint: off");
        } else {
            manual_control_setpoint_flag = true;
            warnx("\t manual_control_setpoint: on");
        }
        return 0;
    }

    if (!strcmp(argv[1], "actuator_controls")) {
        if (actuator_controls_flag) {
            actuator_controls_flag = false;
            warnx("\t actuator_controls: off");
        } else {
            actuator_controls_flag = true;
            warnx("\t actuator_controls: on");
        }
        return 0;
    }

    if (!strcmp(argv[1], "input_rc")) {
        if (input_rc_flag) {
            input_rc_flag = false;
            warnx("\t input_rc: off");
        } else {
            input_rc_flag = true;
            warnx("\t input_rc: on");
        }
        return 0;
    }

    if (!strcmp(argv[1], "vehicle_control_mode")) {
        if (vehicle_control_mode_flag) {
            vehicle_control_mode_flag = false;
            warnx("\t vehicle_control_mode: off");
        } else {
            vehicle_control_mode_flag = true;
            warnx("\t vehicle_control_mode: on");
        }
        return 0;
    }

    if (!strcmp(argv[1], "vehicle_rates_setpoint")) {
        if (vehicle_rates_setpoint_flag) {
            vehicle_rates_setpoint_flag = false;
            warnx("\t vehicle_rates_setpoint: off");
        } else {
            vehicle_rates_setpoint_flag = true;
            warnx("\t vehicle_rates_setpoint: on");
        }
        return 0;
    }

    usage("unrecognized command");
    return 1;
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

    int actuator_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
    struct actuator_armed_s actuator_armed_data;
    memset(&actuator_armed_data, 0, sizeof(actuator_armed_data));

    int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    struct manual_control_setpoint_s manual_control_setpoint_data;
    memset(&manual_control_setpoint_data, 0, sizeof(manual_control_setpoint_data));

    int actuator_controls_sub = orb_subscribe(ORB_ID(actuator_controls_0));
    struct actuator_controls_s actuator_controls_data;
    memset(&actuator_controls_data, 0, sizeof(actuator_controls_data));

    int input_rc_sub = orb_subscribe(ORB_ID(input_rc));
    struct rc_input_values input_rc_data;
    memset(&input_rc_data, 0, sizeof(input_rc_data));

    int vehicle_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    struct vehicle_control_mode_s vehicle_control_mode_data;
    memset(&vehicle_control_mode_data, 0, sizeof(vehicle_control_mode_data));

    int vehicle_rates_setpoint_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
    struct vehicle_rates_setpoint_s vehicle_rates_setpoint_data;
    memset(&vehicle_rates_setpoint_data, 0, sizeof(vehicle_rates_setpoint_data));

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

        orb_check(actuator_armed_sub, &updated);
        if (updated & actuator_armed_flag) {
            orb_copy(ORB_ID(actuator_armed), actuator_armed_sub, &actuator_armed_data);
            print_actuator_armed(actuator_armed_data);
        }

        orb_check(manual_control_setpoint_sub, &updated);
        if (updated & manual_control_setpoint_flag) {
            orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual_control_setpoint_data);
            print_manual_control_setpoint(manual_control_setpoint_data);
        }

        orb_check(actuator_controls_sub, &updated);
        if (updated & actuator_controls_flag) {
            orb_copy(ORB_ID(actuator_controls_0), actuator_controls_sub, &actuator_controls_data);
            print_actuator_controls(actuator_controls_data);
        }

        orb_check(input_rc_sub, &updated);
        if (updated & input_rc_flag) {
            orb_copy(ORB_ID(input_rc), input_rc_sub, &input_rc_data);
            print_input_rc(input_rc_data);
        }

        orb_check(vehicle_control_mode_sub, &updated);
        if (updated & vehicle_control_mode_flag) {
            orb_copy(ORB_ID(vehicle_control_mode), vehicle_control_mode_sub, &vehicle_control_mode_data);
            print_vehicle_control_mode(vehicle_control_mode_data);
        }

        orb_check(vehicle_rates_setpoint_sub, &updated);
        if (updated & vehicle_rates_setpoint_flag) {
            orb_copy(ORB_ID(vehicle_rates_setpoint), vehicle_rates_setpoint_sub, &vehicle_rates_setpoint_data);
            print_vehicle_rates_setpoint(vehicle_rates_setpoint_data);
        }

        usleep(500000);
    }

    warnx("[echo_msgs] exiting\n");
    thread_running = false;
    return 0;
}

void print_vehicle_vicon_position(struct vehicle_vicon_position_s vehicle_vicon_position_data)
{
    printf("vehicle_vicon_position:\n\t x: %8.4f\n\t y: %8.4f\n\t z: %8.4f\n\t roll: %8.4f\n\t pitch: %8.4f\n\t yaw: %8.4f\n",
                   (double)vehicle_vicon_position_data.x,
                   (double)vehicle_vicon_position_data.y,
                   (double)vehicle_vicon_position_data.z,
                   (double)vehicle_vicon_position_data.roll,
                   (double)vehicle_vicon_position_data.pitch,
                   (double)vehicle_vicon_position_data.yaw);
}

void print_vehicle_status(struct vehicle_status_s vehicle_status_data)
{
    printf("vehicle_status:\n\t system_type: %d\n\t system_id: %d\n\t component_id: %d\n\t main_state: %d\n\t arming_state: %d\n\t ",
           vehicle_status_data.system_type,
           vehicle_status_data.system_id,
           vehicle_status_data.component_id,
           vehicle_status_data.main_state,
           vehicle_status_data.arming_state);
}

void print_safety(struct safety_s safety_data)
{
    printf("safety:\n\t safety_switch_available: %d\n\t safety_mode: %d\n",
           safety_data.safety_switch_available,
           safety_data.safety_mode);
}

void print_rc_channels(struct rc_channels_s rc_channels_data)
{
    printf("rc_channels:\n");
    printf("\t timestamp: %ld\n", (long)rc_channels_data.timestamp);
    printf("\t timestamp_last_valid: %d\n", rc_channels_data.timestamp_last_valid);
    printf("\t channel_count: %d\n", rc_channels_data.channel_count);
    printf("\t rssi: %d\n", rc_channels_data.rssi);
    printf("\t signal_lost: %d\n", rc_channels_data.signal_lost);
    for (int i=0; i<RC_CHANNELS_FUNCTION_MAX; i++) {
        printf("\t channel %d: %8.4f\n", i, (double)rc_channels_data.channels[i]);
    }
}

void print_vehicle_command(struct vehicle_command_s vehicle_command_data)
{
    printf("vehicle_command:\n");
    printf("\t command: %d\n", vehicle_command_data.command);
    printf("\t param1: %8.4f\n", (double)vehicle_command_data.param1);
    printf("\t param2: %8.4f\n", (double)vehicle_command_data.param2);
    printf("\t param3: %8.4f\n", (double)vehicle_command_data.param3);
    printf("\t param4: %8.4f\n", (double)vehicle_command_data.param4);
    printf("\t param5: %8.4f\n", (double)vehicle_command_data.param5);
    printf("\t param6: %8.4f\n", (double)vehicle_command_data.param6);
    printf("\t param7: %8.4f\n", (double)vehicle_command_data.param7);
    printf("\t target_system: %d\n", vehicle_command_data.target_system);
    printf("\t target_component: %d\n", vehicle_command_data.target_component);
    printf("\t source_system: %d\n", vehicle_command_data.source_system);
    printf("\t source_component: %d\n", vehicle_command_data.source_component);
}

void print_actuator_armed(struct actuator_armed_s actuator_armed_data)
{
    printf("actuator_armed:\n");
    printf("\t timestamp: %ld\n", (long)actuator_armed_data.timestamp);
    printf("\t armed: %d\n", actuator_armed_data.armed);
    printf("\t ready_to_arm: %d\n", actuator_armed_data.ready_to_arm);
    printf("\t lockdown: %d\n", actuator_armed_data.lockdown);
    printf("\t force_failsafe: %d\n", actuator_armed_data.force_failsafe);
}

void print_manual_control_setpoint(struct manual_control_setpoint_s manual_control_setpoint_data)
{
    printf("manual_control_setpoint:\n");
    printf("\t timestamp: %ld\n", (long)manual_control_setpoint_data.timestamp);
    printf("\t x: %8.4f\n", (double)manual_control_setpoint_data.x);
    printf("\t y: %8.4f\n", (double)manual_control_setpoint_data.y);
    printf("\t z: %8.4f\n", (double)manual_control_setpoint_data.z);
    printf("\t r: %8.4f\n", (double)manual_control_setpoint_data.r);
    printf("\t flaps: %8.4f\n", (double)manual_control_setpoint_data.flaps);
    printf("\t aux1: %8.4f\n", (double)manual_control_setpoint_data.aux1);
    printf("\t aux2: %8.4f\n", (double)manual_control_setpoint_data.aux2);
    printf("\t aux3: %8.4f\n", (double)manual_control_setpoint_data.aux3);
    printf("\t aux4: %8.4f\n", (double)manual_control_setpoint_data.aux4);
    printf("\t aux5: %8.4f\n", (double)manual_control_setpoint_data.aux5);
    printf("\t mode_switch: %d\n", manual_control_setpoint_data.mode_switch);
    printf("\t return_switch: %d\n", manual_control_setpoint_data.return_switch);
    printf("\t posctl_switch: %d\n", manual_control_setpoint_data.posctl_switch);
    printf("\t loiter_switch: %d\n", manual_control_setpoint_data.loiter_switch);
    printf("\t acro_switch: %d\n", manual_control_setpoint_data.acro_switch);
    printf("\t offboard_switch: %d\n", manual_control_setpoint_data.offboard_switch);
}

void print_actuator_controls(struct actuator_controls_s actuator_controls_data)
{
    printf("actuator_controls:\n");
    printf("\t timestamp: %ld\n", (long)actuator_controls_data.timestamp);
    for (int i=0; i<NUM_ACTUATOR_CONTROLS; ++i) {
        printf("\t control[%d]: %8.4f\n", i, (double)actuator_controls_data.control[i]);
    }
}

void print_input_rc(struct rc_input_values input_rc_data)
{
    printf("input_rc:\n");
    printf("\t timestamp_publication: %d\n", input_rc_data.timestamp_publication);
    printf("\t timestamp_last_signal: %d\n", input_rc_data.timestamp_last_signal);
    printf("\t channel_count: %d\n", input_rc_data.channel_count);
    printf("\t rssi: %d\n", input_rc_data.rssi);
    printf("\t rc_failsafe: %d\n", input_rc_data.rc_failsafe);
    printf("\t rc_lost: %d\n", input_rc_data.rc_lost);
    for (int i=0; i<6; i++) {
        printf("\t values %d: %d\n", i, input_rc_data.values[6]);
    }
}

void print_vehicle_control_mode(struct vehicle_control_mode_s vehicle_control_mode_data)
{
    printf("vehicle_control_mode:\n");
    printf("\t timestamp: %ld\n", (long)vehicle_control_mode_data.timestamp);
    printf("\t flag_armed: %d\n", vehicle_control_mode_data.flag_armed);
    printf("\t flag_external_manual_override_ok: %d\n", vehicle_control_mode_data.flag_external_manual_override_ok);
    printf("\t flag_system_hil_enabled: %d\n", vehicle_control_mode_data.flag_system_hil_enabled);
    printf("\t flag_control_manual_enabled: %d\n", vehicle_control_mode_data.flag_control_manual_enabled);
    printf("\t flag_control_auto_enabled: %d\n", vehicle_control_mode_data.flag_control_auto_enabled);
    printf("\t flag_control_offboard_enabled: %d\n", vehicle_control_mode_data.flag_control_offboard_enabled);
    printf("\t flag_control_rates_enabled: %d\n", vehicle_control_mode_data.flag_control_rates_enabled);
    printf("\t flag_control_attitude_enabled: %d\n", vehicle_control_mode_data.flag_control_attitude_enabled);
    printf("\t flag_control_force_enabled: %d\n", vehicle_control_mode_data.flag_control_force_enabled);
    printf("\t flag_control_velocity_enabled: %d\n", vehicle_control_mode_data.flag_control_velocity_enabled);
    printf("\t flag_control_altitude_enabled: %d\n", vehicle_control_mode_data.flag_control_altitude_enabled);
    printf("\t flag_control_climb_rate_enabled: %d\n", vehicle_control_mode_data.flag_control_climb_rate_enabled);
    printf("\t flag_control_termination_enabled: %d\n", vehicle_control_mode_data.flag_control_termination_enabled);
}

void print_vehicle_rates_setpoint(struct vehicle_rates_setpoint_s vehicle_rates_setpoint_data)
{
    printf("vehicle_rates_setpoint:\n");
    printf("\t timestamp: %ld\n", (long)vehicle_rates_setpoint_data.timestamp);
    printf("\t roll: %8.4f\n", (double)vehicle_rates_setpoint_data.roll);
    printf("\t pitch: %8.4f\n", (double)vehicle_rates_setpoint_data.pitch);
    printf("\t yaw: %8.4f\n", (double)vehicle_rates_setpoint_data.yaw);
    printf("\t thrust: %8.4f\n", (double)vehicle_rates_setpoint_data.thrust);
}
