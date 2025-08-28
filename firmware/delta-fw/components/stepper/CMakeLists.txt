idf_component_register(
    SRCS "stepper.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer log
)
