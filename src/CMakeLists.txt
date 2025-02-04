add_executable(pico-sensor-hub 
                main.c
                ../lib/pmicro-rf/protocol/protocol.c
        )
target_include_directories(pico-sensor-hub   
                                PRIVATE ${CMAKE_CURRENT_LIST_DIR}
                                ../lib/pmicro-rf/protocol
        )

# pull in common dependencies
target_link_libraries(pico-sensor-hub
        pico-simple-mqtt
        pmicro-rf
        pico_stdlib
        )

# Setting up credentials.
include(device.cmake)

 #Compile definitions needed by the project
 target_compile_definitions(pico-sensor-hub PRIVATE
    WIFI_SSID=\"${WIFI_SSID_SET}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD_SET}\"
    MQTT_CLIENT=\"${MQTT_CLIENT_SET}\"
    MQTT_USER=\"${MQTT_USER_SET}\"
    MQTT_PASSWD=\"${MQTT_PASSWD_SET}\"
    MQTT_HOST=\"${MQTT_HOST_SET}\"
    MQTT_PORT=${MQTT_PORT_SET}
)

# Enable print functionality on usb & disable on uart
pico_enable_stdio_usb(pico-sensor-hub 0) 
pico_enable_stdio_uart(pico-sensor-hub 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(pico-sensor-hub)

