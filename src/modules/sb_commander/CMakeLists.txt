px4_add_module(
    MODULE modules__sb_commander
    MAIN sb_commander
    SRCS 
        sb_commander.cpp
    MODULE_CONFIG
        module.yaml
)
