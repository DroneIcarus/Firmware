//
// Created by eskimo on 18-07-05.
//


#include <lib/mixer/mixer.h>
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_tasks.h>




#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/rc_channels.h>

extern "C" __EXPORT int drop_deleaves_main(int argc, char **argv);

class DropDeLeaves : public ModuleBase<DropDeLeaves>, public ModuleParams
{
public:
    DropDeLeaves();

    virtual ~DropDeLeaves() = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static DropDeLeaves *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

private:
    /**
    * Check for parameter update and handle it.
    */
    void		rc_channels_poll();


    int _rc_channels_sub{-1};

    struct rc_channels_s _rc_channels{};

    struct actuator_outputs_s		_outputs {};		/**< actuator controls */


    orb_id_t _outputs_id{nullptr};	/**< pointer to correct actuator controls0 uORB metadata structure */

    orb_advert_t	_outputs_pub{nullptr};		/**< attitude actuator controls publication */




    perf_counter_t	_loop_perf;			/**< loop performance counter */
};

