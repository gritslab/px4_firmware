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

#include <uORB/topics/safety.h>
#include <uORB/topics/manual_control_setpoint.h>
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

    ~Commander();

    //--------------------------------------------------------------------------
    // Public Member Getters and Setters
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // Public Member Constants and Variables
    //--------------------------------------------------------------------------
    // Constants
    const int sleep_interval = 50000;
    const float stick_arm_disarm_limit = 0.9f;
    const int stick_arm_disarm_wait_time_ms = 1000;

    // Variables
    FiniteState finite_state;
    Led led;

    //--------------------------------------------------------------------------
    // Public Methods
    //--------------------------------------------------------------------------
    void init();
    void deinit();
    void msg(const char* info);
    void print_state() { finite_state.print_state(); };
    void update();
    int toggle_arm();
    int arm();
    int disarm();

private:
    //--------------------------------------------------------------------------
    // Private Member Variables
    //--------------------------------------------------------------------------
    int m_stick_arm_disarm_counter_limit;
    int m_stick_arm_disarm_counter;

    // Subscription variables
    int m_safety_sub;
    struct safety_s m_safety;

    int m_manual_control_setpoint_sub;
    struct manual_control_setpoint_s m_manual_control_setpoint;

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
};

#endif
