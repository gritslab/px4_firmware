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
    m_setup_orb_subscribers();
    led.init();
}

void Commander::deinit()
{
    m_close_orb_subscribers();

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

    orb_check(m_safety_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(safety), m_safety_sub, &m_safety);
        if (m_safety.safety_off) {
            finite_state[ARM_STATE] == DISARMED;
        } else {
            finite_state[ARM_STATE] == SAFETY;
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
    int m_safety_sub = orb_subscribe(ORB_ID(safety));
    memset(&m_safety, 0, sizeof(m_safety));
    m_safety.safety_switch_available = false;
    m_safety.safety_off = false;
}

void Commander::m_close_orb_subscribers()
{
    close(m_safety_sub);
}

void Commander::m_set_led_based_on_state()
{
    if (finite_state[ARM_STATE] == SAFETY) {
        led.set_color(RGBLED_COLOR_BLUE);
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
