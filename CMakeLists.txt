idf_component_register(
    SRCS "imx462.c"
    INCLUDE_DIRS "include"
    REQUIRES espressif__esp_cam_sensor espressif__esp_sccb_intf
)

# The detect function is found through a linker section, and nothing calls it
# by name, so the linker would otherwise drop the object entirely.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u imx462_detect")
