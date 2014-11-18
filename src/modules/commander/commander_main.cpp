/**
 * @file commander.cpp
 * Main system program.
 *
 * @author Rowland O'Flaherty <rowoflo@gmail.com>
 */

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include "commander.h"
#include "led.h"

//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------
static bool thread_running = false;     /**< daemon status flag */
static bool thread_should_exit = false;     /**< daemon exit flag */
static int daemon_task;             /**< Handle of daemon task / thread */

Commander cmdr;

//------------------------------------------------------------------------------
// Function Definitions
//------------------------------------------------------------------------------
static void usage(const char *reason);
extern "C" __EXPORT int commander_main(int argc, char *argv[]);
int commander_thread_main(int argc, char *argv[]);

//------------------------------------------------------------------------------
// Function Implementations
//------------------------------------------------------------------------------
// usage -----------------------------------------------------------------------
void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	errx(1, "usage: commander {start|stop|status|toggle_arm}\n\n");
}
// usage -----------------------------------------------------------------------

// commander_main --------------------------------------------------------------
int commander_main(int argc, char *argv[])
{
	if (argc < 1) {
		usage("Missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("Already running");
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("commander",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MAX - 40,
					     2950,
					     commander_thread_main,
					     (argv) ? (const char **)&argv[2] : (const char **)NULL);

		while (!thread_running) {
			usleep(200);
		}
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {

		if (!thread_running) {
			errx(0, "Already stopped");
		}

		thread_should_exit = true;

		while (thread_running) {
			usleep(200000);
			warnx(".");
		}

		cmdr.deinit();
		warnx("Terminated");

		exit(0);
	}

	/* commands that need the app to be running */
	if (!thread_running) {
		warnx("Not started");
		exit(1);
	}

	if (!strcmp(argv[1], "status")) {
		warnx("\tRunning\n");
		cmdr.print_state();
		exit(0);
	}

	if (!strcmp(argv[1], "toggle_arm")) {
		cmdr.toggle_arm();
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
// commander_main --------------------------------------------------------------

// commander_thread_main -------------------------------------------------------
int commander_thread_main(int argc, char *argv[])
{
	cmdr.init();

	thread_running = true;
	while(!thread_should_exit) {
		cmdr.update();
		usleep(cmdr.sleep_interval);
	}

	warnx("Exiting...");
    thread_running = false;
    return 0;
}
// commander_thread_main -------------------------------------------------------
