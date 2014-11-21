/**
 * @brief Quadrotor commander class
 *
 * @file commander.h
 * @author Rowland O'Flaherty
 * @date 11/17/2014
 **/

#ifndef COMMANDER_H
#define COMMANDER_H

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <string.h>

// Subscribers
#include <uORB/topics/safety.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/battery_status.h>

// Publishers
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>

#include "finite_state.h"
#include "led.h"

//==============================================================================
// Commander Class
//==============================================================================
class Commander
{
public:
    //--------------------------------------------------------------------------
    // Constructors and Destructors
    //--------------------------------------------------------------------------
    Commander();

    // ~Commander();

    //--------------------------------------------------------------------------
    // Public Member Getters and Setters
    //--------------------------------------------------------------------------
    // Flag getter
    bool low_battery() { return (m_flag & LOW_BATTERY); };
    bool critical_battery() { return (m_flag & CRITICAL_BATTERY); };

    //--------------------------------------------------------------------------
    // Public Member Constants and Variables
    //--------------------------------------------------------------------------
    // Constants
    const int sleep_interval = 50000;
    const float stick_arm_disarm_limit = 0.9f;
    const int stick_arm_disarm_wait_time_ms = 1000;
    const float low_battery_voltage = 11.1;
    const float critical_battery_voltage = 10.9;

    const rgbled_color_t warning_color = RGBLED_COLOR_RED;
    const rgbled_color_t safety_color = RGBLED_COLOR_AMBER;
    const rgbled_color_t disarmed_color = RGBLED_COLOR_BLUE;
    const rgbled_color_t armed_color = RGBLED_COLOR_GREEN;

    // Variables
    FiniteState finite_state;
    Led led;

    //--------------------------------------------------------------------------
    // Public Methods
    //--------------------------------------------------------------------------
    void init();
    void deinit();
    void msg(const char* info);
    void print_state();
    void update();
    int toggle_arm();
    int arm();
    int disarm();

private:
    //--------------------------------------------------------------------------
    // Private Member Variables
    //--------------------------------------------------------------------------
    Flag m_flag;

    int m_stick_arm_disarm_counter_limit;
    int m_stick_arm_disarm_counter;

    // Subscription variables
    int m_safety_sub;
    struct safety_s m_safety;

    int m_manual_control_setpoint_sub;
    struct manual_control_setpoint_s m_manual_control_setpoint;

    int m_vehicle_command_sub;
    struct vehicle_command_s m_vehicle_command;

    int m_battery_status_sub;
    struct battery_status_s m_battery_status;

    // Publisher variables
    orb_advert_t m_vehicle_control_mode_pub;
    struct vehicle_control_mode_s m_vehicle_control_mode;

    orb_advert_t m_actuator_armed_pub;
    struct actuator_armed_s m_actuator_armed;

    //--------------------------------------------------------------------------
    // Private Methods
    //--------------------------------------------------------------------------
    void m_init_orb_subscribers();
    void m_close_orb_subscribers();
    void m_init_orb_publishers();
    void m_orb_publish();
    void m_init_vars_and_finiti_state();
    void m_update_finite_state();
    void m_update_vars_based_on_state();
    void m_check_safety();
    void m_check_rc_arm_disarm(int arm_state_check);
    void m_check_vehicle_command();
    void m_check_battery_status();
};

#endif
