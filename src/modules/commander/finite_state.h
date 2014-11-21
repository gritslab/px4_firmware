/*
@brief Enums and class defining the finite states

@author Rowland O'Flaherty
@date 11/17/2014
*/

#ifndef FINITE_STATE_H
#define FINITE_STATE_H

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <cassert>
#include <stdio.h>

//------------------------------------------------------------------------------
// Constants and Enums
//------------------------------------------------------------------------------

// Finite State
enum FiniteStateNames
{
    ARM_STATE=0,
    FLIGHT_STATE,
    SETPOINT_STATE,
};

// DIM 0
enum ArmState
{
    SAFETY=0,
    DISARMED,
    ARMED
};

// DIM 1
enum FlightState
{
    GROUNDED=0,
    HOLDING,
    AUTONOMOUS,
    TELEOP
};

// DIM 2
enum SetpointState
{
    AT_SETPOINT = 0,
    GOING_TO_SETPOINT
};

// Transitions
enum Commands
{
    TOGGLE_ARM=0,
    TAKEOFF,
    FLY_TO,
    LAND,
    CALIBRATE,
    RETURN,
    CHANGE_MODE
};

// Flags
enum Flags
{
    LOW_BATTERY = 1,
    CRITICAL_BATTERY = 2
};

typedef unsigned Flag; // Up to 32 flags

//==============================================================================
// FiniteState Class
//==============================================================================
class FiniteState
{
private:
    static const int m_state_dim = 3;
    static const int m_state_dim_max_size = 4;
    int m_finite_state[m_state_dim];
    const char *finite_state_names_str[m_state_dim];
    const char *finite_state_strs[m_state_dim][m_state_dim_max_size];

public:
    FiniteState() {
        finite_state_names_str[ARM_STATE]      = "ARM_STATE";
        finite_state_names_str[FLIGHT_STATE]   = "FLIGHT_STATE";
        finite_state_names_str[SETPOINT_STATE] = "SETPOINT_STATE";

        finite_state_strs[ARM_STATE][SAFETY] = "SAFETY";
        finite_state_strs[ARM_STATE][DISARMED] = "DISARMED";
        finite_state_strs[ARM_STATE][ARMED] = "ARMED";

        finite_state_strs[FLIGHT_STATE][GROUNDED] = "GROUNDED";
        finite_state_strs[FLIGHT_STATE][HOLDING] = "HOLDING";
        finite_state_strs[FLIGHT_STATE][AUTONOMOUS] = "AUTONOMOUS";
        finite_state_strs[FLIGHT_STATE][TELEOP] = "TELEOP";

        finite_state_strs[SETPOINT_STATE][AT_SETPOINT]       = "AT_SETPOINT";
        finite_state_strs[SETPOINT_STATE][GOING_TO_SETPOINT] = "GOING_TO_SETPOINT";
    }

    int& operator[] (const int n_index) {
        assert(n_index >= 0 && n_index < m_state_dim);
        return m_finite_state[n_index];
    }

    void print_state() {
        printf("** CURRENT FINITE STATE **\n");
        for (int i = 0; i < m_state_dim; ++i) {
            printf("%s: %s\n",
                   finite_state_names_str[i],
                   finite_state_strs[i][m_finite_state[i]]);
        }
    }
};

#endif
