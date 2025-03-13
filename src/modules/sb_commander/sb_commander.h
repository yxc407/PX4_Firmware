/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *   （以下省略版權宣告）
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>

#include <uORB/topics/switchblade_command.h>
#include <uORB/topics/switchblade_status.h>

using namespace time_literals;

extern "C" __EXPORT int sb_commander_main(int argc, char *argv[]);

class SwitchbladeCommander : public ModuleBase<SwitchbladeCommander>
{
public:
    SwitchbladeCommander();
    virtual ~SwitchbladeCommander();

    static int task_spawn(int argc, char *argv[]);
    static SwitchbladeCommander *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    static int stop();
    static int status();

    void run() override;
    int print_status() override;

private:
    switchblade_command_s _cmd{};

    uORB::SubscriptionInterval _switchblade_status_sub{ORB_ID(switchblade_status), 1_s};

    static SwitchbladeCommander *_instance;
};