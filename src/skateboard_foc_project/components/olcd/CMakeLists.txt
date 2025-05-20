idf_component_register(SRCS "olcd.c" "lvgl_demo_ui.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver communication esp_lvgl_port
)
