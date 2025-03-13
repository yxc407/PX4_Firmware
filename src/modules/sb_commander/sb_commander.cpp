/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *   (版權宣告省略)
 *
 ****************************************************************************/

#include "sb_commander.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/switchblade_command.h>
#include <uORB/topics/switchblade_status.h>

SwitchbladeCommander *SwitchbladeCommander::_instance = nullptr;

SwitchbladeCommander::SwitchbladeCommander()
{
    memset(&_cmd, 0, sizeof(_cmd));
    _cmd.timestamp = hrt_absolute_time();
    _cmd.attack_position_type = 0;
    _cmd.attack_position[0] = 0.0f;
    _cmd.attack_position[1] = 0.0f;
    _cmd.attack_position[2] = 0.0f;
    _cmd.attack_trigger = false;
}

SwitchbladeCommander::~SwitchbladeCommander()
{
}

int SwitchbladeCommander::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("sb_commander",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  1024,
                                  (px4_main_t)&ModuleBase<SwitchbladeCommander>::run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }
    return 0;
}

SwitchbladeCommander *SwitchbladeCommander::instantiate(int argc, char *argv[])
{
    SwitchbladeCommander *instance = new SwitchbladeCommander();
    if (instance) {
        _instance = instance;
    }
    return instance;
}

int SwitchbladeCommander::custom_command(int argc, char *argv[])
{
    if (_instance == nullptr) {
        PX4_ERR("Module not running");
        return PX4_ERROR;
    }
    if (argc < 1) {
        PX4_ERR("No command provided");
        return PX4_ERROR;
    }
    if (strcmp(argv[0], "attack") == 0) {
        if (argc != 1) {
            PX4_ERR("Usage: sb_commander attack");
            return PX4_ERROR;
        }
        _instance->_cmd.attack_trigger = true;
        PX4_INFO("Updated: attack_trigger");
    } else if (strcmp(argv[0], "cancel") == 0) {
        if (argc != 1) {
            PX4_ERR("Usage: sb_commander cancel");
            return PX4_ERROR;
        }
        _instance->_cmd.attack_trigger = false;
        PX4_INFO("Updated: attack_cancel");
    } else if (strcmp(argv[0], "setpoint") == 0) {
        if (argc != 5) {
            PX4_ERR("Usage: sb_commander setpoint [local|global] x y z");
            return PX4_ERROR;
        }
        if (strcmp(argv[1], "local") == 0) {
            _instance->_cmd.attack_position_type = 0;
        } else if (strcmp(argv[1], "global") == 0) {
            _instance->_cmd.attack_position_type = 1;
        } else {
            PX4_ERR("Invalid attack_position type '%s'", argv[1]);
            return PX4_ERROR;
        }
        double x = atof(argv[2]);
        double y = atof(argv[3]);
        double z = atof(argv[4]);
        _instance->_cmd.attack_position[0] = static_cast<float>(x);
        _instance->_cmd.attack_position[1] = static_cast<float>(y);
        _instance->_cmd.attack_position[2] = static_cast<float>(z);
        _instance->_cmd.attack_trigger = false;
        PX4_INFO("Updated: attack_position");
    } else {
        PX4_ERR("Unknown command: %s", argv[0]);
        return PX4_ERROR;
    }
    return PX4_OK;
}

int SwitchbladeCommander::stop()
{
    if (_instance) {
        _instance->request_stop();
        while (_instance->is_running()) {
            px4_usleep(10000);
        }
        _instance = nullptr;
        PX4_INFO("sb_commander stopped");
        return 0;
    } else {
        PX4_WARN("sb_commander not running");
        return PX4_ERROR;
    }
}

int SwitchbladeCommander::status()
{
    if (_instance) {
        _instance->print_status();
    } else {
        PX4_INFO("sb_commander not running");
    }
    return 0;
}

int SwitchbladeCommander::print_status()
{
    PX4_INFO("sb_commander status:");
    PX4_INFO("  attack_trigger: %s", (_cmd.attack_trigger ? "true" : "false"));
    const char *type_str = (_cmd.attack_position_type == 0) ? "local" : "global";
    PX4_INFO("  attack_position (%s): [%.3f, %.3f, %.3f]",
             type_str,
             (double)_cmd.attack_position[0],
             (double)_cmd.attack_position[1],
             (double)_cmd.attack_position[2]);
    return 0;
}

int SwitchbladeCommander::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s", reason);
    }
    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
This is the switchblade control interface module.
Commands:
    start       - Start the module (with default parameters)
    stop        - Stop the module
    status      - Show current status
    setpoint    - Update attack position: setpoint [local|global] x y z
    attack      - Set attack_trigger true
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("sb_commander", "module");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND("stop");
    PRINT_MODULE_USAGE_COMMAND("status");
    PRINT_MODULE_USAGE_COMMAND("setpoint");
    PRINT_MODULE_USAGE_COMMAND("attack");
    PRINT_MODULE_USAGE_COMMAND("cancel");
    return 0;
}

void SwitchbladeCommander::run()
{
    PX4_INFO("sb_commander started");

    orb_advert_t pub_handle = nullptr;

    while (!should_exit()) {
        _cmd.timestamp = hrt_absolute_time();

        if (pub_handle == nullptr) {
            pub_handle = orb_advertise(ORB_ID(switchblade_command), &_cmd);
            if (pub_handle == nullptr) {
                PX4_ERR("Failed to advertise switchblade_command");
                break;
            }
        } else {
            orb_publish(ORB_ID(switchblade_command), pub_handle, &_cmd);
        }

        px4_usleep(50000);
    }

    PX4_INFO("sb_commander exiting");

    if (pub_handle != nullptr) {
        orb_unadvertise(pub_handle);
    }
}

int sb_commander_main(int argc, char *argv[])
{
    if (argc < 2) {
        SwitchbladeCommander::print_usage("missing command");
        return -1;
    }
    if (!strcmp(argv[1], "start")) {
        return SwitchbladeCommander::task_spawn(argc - 1, argv + 1);
    }
    if (!strcmp(argv[1], "stop")) {
        return SwitchbladeCommander::stop();
    }
    if (!strcmp(argv[1], "status")) {
        return SwitchbladeCommander::status();
    }
    return SwitchbladeCommander::custom_command(argc - 1, argv + 1);
}