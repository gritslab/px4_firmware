/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/vehicle_command.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	printf("Hello Sky!\n");

	/* subscribe to sensor_combined topic */
	// int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	// orb_set_interval(sensor_combined_sub, 100);

	// int vehicel_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	// orb_set_interval(vehicel_command_sub, 100);

	/* advertise attitude topic */
	// struct vehicle_attitude_s att;
	// memset(&att, 0, sizeof(att));
	// orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_set_interval(vehicle_attitude_sub, 100);

	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[] = {
		// { .fd = vehicel_command_sub,   .events = POLLIN }
		// { .fd = sensor_combined_sub,   .events = POLLIN },
		{ .fd = vehicle_attitude_sub,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 600; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = poll(fds, 1, 100);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			// printf("[px4_simple_app] Got no data within a second\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[px4_simple_app] ERROR return value from poll(): %d\n"
					, poll_ret);
			}
			error_counter++;
		} else {

			if (fds[0].revents & POLLIN) {
			// 	struct vehicle_command_s raw;
			// 	orb_copy(ORB_ID(vehicle_command), vehicel_command_sub, &raw);
			// 	printf("%d-[px4_simple_app] Command:\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%d\t%d\t%d\t%d\t%d\t%d\n",
			// 	       i,
			// 	       (double)raw.param1,
			// 	       (double)raw.param2,
			// 	       (double)raw.param3,
			// 	       (double)raw.param4,
			// 	       (double)raw.param5,
			// 	       (double)raw.param6,
			// 	       (double)raw.param7,
			// 	       (int)raw.command,
			// 	       (uint8_t)raw.target_system,
			// 	       (uint8_t)raw.target_component,
			// 	       (uint8_t)raw.source_system,
			// 	       (uint8_t)raw.source_component,
			// 	       (uint8_t)raw.confirmation);

				struct vehicle_attitude_s att;
				orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
				printf("%d-[px4_simple_app] Attitude:\t%8.4f\t%8.4f\t%8.4f\n",
				       i,
				       (double)att.roll,
				       (double)att.pitch,\
				       (double)att.yaw);


				// /* obtained data for the first file descriptor */
				// struct sensor_combined_s raw;
				// /* copy sensors raw data into local buffer */
				// orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &raw);
				// printf("[px4_simple_app] Accelerometer:\t%8.4f\t%8.4f\t%8.4f\n",
				// 	(double)raw.accelerometer_m_s2[0],
				// 	(double)raw.accelerometer_m_s2[1],
				// 	(double)raw.accelerometer_m_s2[2]);

				// /* set att and publish this information for other apps */
				// att.roll = raw.accelerometer_m_s2[0];
				// att.pitch = raw.accelerometer_m_s2[1];
				// att.yaw = raw.accelerometer_m_s2[2];
				// orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
			}
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	return 0;
}
