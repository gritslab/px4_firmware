/**
 * @file led.cpp
 * @author Rowland O'Flaherty
 * @date 11/17/2014
 */

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "led.h"

#include <fcntl.h>
#include <unistd.h>
#include <systemlib/err.h>

#include <stdio.h>

//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------


//==============================================================================
// Led Class
//==============================================================================
//------------------------------------------------------------------------------
// Constructors and Destructors
//------------------------------------------------------------------------------
// Led::Led()
// {
// }

// Led::~Led()
// {
// }

//--------------------------------------------------------------------------
// Public Member Getters and Setters
//--------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Public Methods
//------------------------------------------------------------------------------
void Led::init()
{
    m_rgbleds = open(RGBLED_DEVICE_PATH, 0);
    if (m_rgbleds == -1) {
        warnx("No RGB LED found at %s", RGBLED_DEVICE_PATH);
    }
}

void Led::deinit()
{
    if (m_rgbleds != -1) {
        close(m_rgbleds);
    }
}

void Led::set_color(rgbled_color_t color)
{
    if (m_rgbleds != -1) {
        ioctl(m_rgbleds, RGBLED_SET_COLOR, (unsigned long)color);
    }
}

void Led::set_mode(rgbled_mode_t mode)
{
    if (m_rgbleds != -1) {
        ioctl(m_rgbleds, RGBLED_SET_MODE, (unsigned long)mode);
    }
}

void Led::set_pattern(rgbled_pattern_t *pattern)
{
    if (m_rgbleds != -1) {
        ioctl(m_rgbleds, RGBLED_SET_PATTERN, (unsigned long)pattern);
    }
}

//------------------------------------------------------------------------------
// Private Methods
//------------------------------------------------------------------------------

