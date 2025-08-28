idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES micro_ros_espidf_component usb_serial_transport stepper end_effector battery_monitor dc_motor) 