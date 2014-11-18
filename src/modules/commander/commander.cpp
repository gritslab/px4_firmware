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
{

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
    m_setup_orb_subscribers();

    // Init finite state
    m_init_finite_state();

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
    bool updated;

    // Safety
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

    m_set_led_based_on_state();
}

int Commander::toggle_arm()
{
    if (finite_state[ARM_STATE] == SAFETY) {
        msg("Unable to toggle state due to safety switch");
    } else if (finite_state[ARM_STATE] == DISARMED) {
        finite_state[ARM_STATE] = ARMED;
    } else if (finite_state[ARM_STATE] == ARMED) {
        finite_state[ARM_STATE] = DISARMED;
    }
    return 0;
}

//------------------------------------------------------------------------------
// Private Methods
//------------------------------------------------------------------------------
void Commander::m_setup_orb_subscribers()
{
    // Safety topic
    m_safety_sub = orb_subscribe(ORB_ID(safety));
    memset(&m_safety, 0, sizeof(m_safety));
}

void Commander::m_close_orb_subscribers()
{
    close(m_safety_sub);
}

void Commander::m_init_finite_state()
{
    bool updated;

    // Safety
    orb_check(m_safety_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(safety), m_safety_sub, &m_safety);
        if (m_safety.safety_mode) {
            finite_state[ARM_STATE] = SAFETY;
        } else {
            finite_state[ARM_STATE] = DISARMED;
        }
    }
}

void Commander::m_set_led_based_on_state()
{
    if (finite_state[ARM_STATE] == SAFETY) {
        led.set_color(RGBLED_COLOR_RED);
        led.set_mode(RGBLED_MODE_BLINK_NORMAL);
    } else if (finite_state[ARM_STATE] == DISARMED) {
        led.set_color(RGBLED_COLOR_GREEN);
        led.set_mode(RGBLED_MODE_BLINK_NORMAL);
    } else {
        led.set_color(RGBLED_COLOR_GREEN);
        led.set_mode(RGBLED_MODE_ON);
    }
}


//------------------------------------------------------------------------------
// Helper Functions
//------------------------------------------------------------------------------
