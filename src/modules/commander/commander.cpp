/**
 * @file commander.cpp
 * @author Rowland O'Flaherty
 * @date 11/17/2014
 */

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "commander.h"

#include <fcntl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------


//==============================================================================
// Commander Class
//==============================================================================
//------------------------------------------------------------------------------
// Constructors and Destructors
//------------------------------------------------------------------------------
Commander::Commander()
:
m_stick_arm_disarm_counter(0)
{
    finite_state[FLIGHT_STATE] = TELEOP; // TODO: remove

    // Variables for RC stick checking for arm/disarm
    float commander_monitoring_loops_per_msec = 1/(sleep_interval/1000.0);
    m_stick_arm_disarm_counter_limit = stick_arm_disarm_wait_time_ms * commander_monitoring_loops_per_msec;
}

Commander::~Commander()
{

}

//--------------------------------------------------------------------------
// Public Member Getters and Setters
//--------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Public Methods
//------------------------------------------------------------------------------
void Commander::init()
{
    // Init subscriptions
    m_init_orb_subscribers();

    // Init publishers
    m_init_orb_publishers();

    // Init variables and finite state
    m_init_vars_and_finiti_state();

    // Init led
    led.init();
}

void Commander::deinit()
{
    m_close_orb_subscribers();

    led.set_color(RGBLED_COLOR_OFF);
    led.set_mode(RGBLED_MODE_OFF);
    led.deinit();
}

void Commander::msg(const char* info)
{
    warnx("%s", info);
}

void Commander::update()
{
    m_update_finite_state();
    m_update_vars_based_on_state();
    m_orb_publish();
}

int Commander::toggle_arm()
{
    if (finite_state[ARM_STATE] == SAFETY) {
        msg("Unable to toggle state due to safety switch");
        return 1;
    } else if (finite_state[ARM_STATE] == DISARMED) {
        finite_state[ARM_STATE] = ARMED;
    } else if (finite_state[ARM_STATE] == ARMED) {
        finite_state[ARM_STATE] = DISARMED;
    }
    return 0;
}

int Commander::arm()
{
    if (finite_state[ARM_STATE] == SAFETY) {
        msg("Unable to toggle state due to safety switch");
        return 1;
    } else {
        finite_state[ARM_STATE] = ARMED;
    }
    return 0;
}

int Commander::disarm()
{
    finite_state[ARM_STATE] = DISARMED;
    return 0;
}

//------------------------------------------------------------------------------
// Private Methods
//------------------------------------------------------------------------------
void Commander::m_init_orb_subscribers()
{
    // safety topic
    m_safety_sub = orb_subscribe(ORB_ID(safety));
    memset(&m_safety, 0, sizeof(m_safety));

    // manual_control_setpoint topic (for RC inputs)
    m_manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    memset(&m_manual_control_setpoint, 0, sizeof(m_manual_control_setpoint));
}

void Commander::m_close_orb_subscribers()
{
    close(m_safety_sub);
    close(m_manual_control_setpoint_sub);
}

void Commander::m_init_orb_publishers()
{
    m_vehicle_control_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode),
                                               &m_vehicle_control_mode);
    memset(&m_vehicle_control_mode, 0, sizeof(m_vehicle_control_mode));

    m_actuator_armed_pub = orb_advertise(ORB_ID(actuator_armed),
                                         &m_actuator_armed);
    memset(&m_actuator_armed, 0, sizeof(m_actuator_armed));
}

void Commander::m_orb_publish()
{
    orb_publish(ORB_ID(vehicle_control_mode),
                m_vehicle_control_mode_pub, &m_vehicle_control_mode);


    orb_publish(ORB_ID(actuator_armed),
                m_actuator_armed_pub, &m_actuator_armed);
}

void Commander::m_init_vars_and_finiti_state()
{
    bool updated;

    // ArmState
    orb_check(m_safety_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(safety), m_safety_sub, &m_safety);
        if (m_safety.safety_mode) {
            finite_state[ARM_STATE] = SAFETY;
        } else {
            finite_state[ARM_STATE] = DISARMED;
        }
    }

    // FlightState
    if (finite_state[FLIGHT_STATE] == TELEOP) {
        m_vehicle_control_mode.flag_control_manual_enabled = true;
        m_vehicle_control_mode.flag_control_rates_enabled = true;
    } else {
        m_vehicle_control_mode.flag_control_manual_enabled = false;
        m_vehicle_control_mode.flag_control_rates_enabled = true;
    }
}

void Commander::m_update_finite_state()
{
    // Safety
    m_check_safety();

    // RC arm/disarm
    if (finite_state[FLIGHT_STATE] == TELEOP) {
        if (finite_state[ARM_STATE] == DISARMED) {
            m_check_rc_arm_disarm(ARMED);
        } else if (finite_state[ARM_STATE] == ARMED) {
            m_check_rc_arm_disarm(DISARMED);
        }
    }
}

void Commander::m_update_vars_based_on_state()
{
    hrt_abstime t1 = hrt_absolute_time();

    // m_vehicle_control_mode
    m_vehicle_control_mode.timestamp = t1;
    if (finite_state[ARM_STATE] == ARMED) {
        m_vehicle_control_mode.flag_armed = true;
    } else {
        m_vehicle_control_mode.flag_armed = false;
    }

    // m_actuator_armed
    m_actuator_armed.timestamp = t1;
    if (finite_state[ARM_STATE] == SAFETY) {
        m_actuator_armed.armed = false;
        m_actuator_armed.ready_to_arm = true;
    } else if (finite_state[ARM_STATE] == DISARMED) {
        m_actuator_armed.armed = false;
        m_actuator_armed.ready_to_arm = true;
    } else if (finite_state[ARM_STATE] == ARMED) {
        m_actuator_armed.armed = true;
        m_actuator_armed.ready_to_arm = true;
    }

    // led
    if (finite_state[ARM_STATE] == SAFETY) {
        led.set_color(RGBLED_COLOR_AMBER);
        led.set_mode(RGBLED_MODE_ON);
    } else if (finite_state[ARM_STATE] == DISARMED) {
        led.set_color(RGBLED_COLOR_BLUE);
        led.set_mode(RGBLED_MODE_ON);
    } else if (finite_state[ARM_STATE] == ARMED) {
        led.set_color(RGBLED_COLOR_GREEN);
        led.set_mode(RGBLED_MODE_ON);
    }
}

void Commander::m_check_safety()
{
    bool updated;
    orb_check(m_safety_sub, &updated);
    if (updated) {
        bool safety_mode_prev = m_safety.safety_mode;
        orb_copy(ORB_ID(safety), m_safety_sub, &m_safety);
        if (m_safety.safety_mode != safety_mode_prev) {
            if (m_safety.safety_mode) {
                finite_state[ARM_STATE] = SAFETY;
            } else {
                finite_state[ARM_STATE] = DISARMED;
            }
        }
    }
}

void Commander::m_check_rc_arm_disarm(int arm_state_check)
{
    // for arming check if left stick is in lower left position
    // for disarming check if left stick is in lower left position
    int side = ((arm_state_check == ARMED) ? 1 : -1);

    bool updated;
    orb_check(m_manual_control_setpoint_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(manual_control_setpoint),
                 m_manual_control_setpoint_sub, &m_manual_control_setpoint);

        if (side*m_manual_control_setpoint.r > stick_arm_disarm_limit &&
            m_manual_control_setpoint.z < (1-stick_arm_disarm_limit)) {

            if (m_stick_arm_disarm_counter > m_stick_arm_disarm_counter_limit) {
                if (arm_state_check == ARMED) {
                    arm();
                } else {
                    disarm();
                }
                m_stick_arm_disarm_counter = 0;
            } else {
                m_stick_arm_disarm_counter++;
            }
        }
    }
}

//------------------------------------------------------------------------------
// Helper Functions
//------------------------------------------------------------------------------
