/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file drop_deleaves.cpp
 * Check safety to unlatch DeLeaves mechanism.
 *
 * @author Thomas Courteau <thomas.robichaud.courteau@gmail.com>
 * @author Louis Lepitre <louis.lepitre@usherbrooke.ca>
 */

#include "drop_deleaves.hpp"

#include <drivers/drv_hrt.h>


extern "C" __EXPORT int drop_deleaves_main(int argc, char **argv);


DropDeLeaves::DropDeLeaves() :
    ModuleParams(nullptr),
    _loop_perf(perf_alloc(PC_ELAPSED, "drop_deleaves"))
{
}

int DropDeLeaves::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
Put new description.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

void DropDeLeaves::rc_channels_poll()
{
    bool updated;
    orb_check(_rc_channels_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(rc_channels), _rc_channels_sub, &_rc_channels);
    }
}

void
DropDeLeaves::run()
{
    _rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
    orb_set_interval(_rc_channels_sub, 100);


    _outputs_id = ORB_ID(actuator_outputs);
    _outputs_pub = orb_advertise(_outputs_id, &_outputs);




    rc_channels_poll();

    /* wakeup source */
    px4_pollfd_struct_t poll_fds = {};

    /* Setup of loop */
    poll_fds.fd = _rc_channels_sub;
    poll_fds.events = POLLIN;

    const hrt_abstime task_start = hrt_absolute_time();
    hrt_abstime last_run = task_start;

    while(!should_exit()) {
        /* wait for up to 100ms for data */
        int pret = px4_poll(&poll_fds, 1, 100);

        /* timed out - periodic check for should_exit() */
        if (pret == 0) {
            continue;
        }

        /* this is undesirable but not much we can do - might want to flag unhappy status */
        if (pret < 0) {
            PX4_ERR("poll error %d, %d", pret, errno);
            /* sleep a bit before next try */
            usleep(1000);
            continue;
        }

        perf_begin(_loop_perf);

        /* run controller on gyro changes */
        if (poll_fds.revents & POLLIN) {
            const hrt_abstime now = hrt_absolute_time();
            float dt = (now - last_run) / 1e6f;
            last_run = now;

            /* guard against too small (< 2ms) and too large (> 20ms) dt's */
            if (dt < 0.002f) {
                dt = 0.002f;

            } else if (dt > 0.02f) {
                dt = 0.02f;
            }

            rc_channels_poll();

            PX4_INFO("Input RC : %f", (double)_rc_channels.channels[9]);

//            _outputs.output[10] = 1.0f;

//            PX4_INFO("%f", (double)_outputs.output[10]);

//            orb_publish(_outputs_id, _outputs_pub, &_outputs);
        }

//        request_stop();

        perf_end(_loop_perf);
    }

    orb_unsubscribe(_rc_channels_sub);
}

int DropDeLeaves::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("drop_deleaves",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_ATTITUDE_CONTROL,
                                  1700,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

DropDeLeaves *DropDeLeaves::instantiate(int argc, char *argv[])
{
    return new DropDeLeaves();
}

int DropDeLeaves::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int drop_deleaves_main(int argc, char *argv[])
{
    return DropDeLeaves::main(argc, argv);
}
