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
    // Public Member Variables
    //--------------------------------------------------------------------------
    static const int sleep_interval = 50000;
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

private:
    //--------------------------------------------------------------------------
    // Private Member Variables
    //--------------------------------------------------------------------------

    // Subscription variables
    int m_safety_sub;
    struct safety_s m_safety;

    //--------------------------------------------------------------------------
    // Private Methods
    //--------------------------------------------------------------------------
    void m_setup_orb_subscribers();
    void m_close_orb_subscribers();
    void m_set_led_based_on_state();
};

#endif
