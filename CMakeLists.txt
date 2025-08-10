# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if(DEFINED ENV{FIRMWARE_VERSION})
        add_compile_definitions(FIRMWARE_VERSION=\"$ENV{FIRMWARE_VERSION}\")
else()
        add_compile_definitions(FIRMWARE_VERSION="0.1.0")
        message(WARNING "FIRMWARE_VERSION is not set, using default value")
endif()

if(DEFINED ENV{DEVICE_TYPE})
        add_compile_definitions(DEVICE_TYPE=\"$ENV{DEVICE_TYPE}\")
else()
        add_compile_definitions(DEVICE_TYPE="esp32s2")
        message(WARNING "DEVICE_TYPE is not set, using default value")
endif()

if(DEFINED ENV{APPLICATION_TYPE})
        add_compile_definitions(APPLICATION_TYPE=\"$ENV{APPLICATION_TYPE}\")
else()
        add_compile_definitions(APPLICATION_TYPE="thermo")
        message(WARNING "APPLICATION_TYPE is not set, using default value")
endif()

if(DEFINED ENV{WIFI_SSID})
	add_compile_definitions(WIFI_SSID=\"$ENV{WIFI_SSID}\")
else()
	add_compile_definitions(WIFI_SSID="guest")
	message(WARNING "WIFI_SSID is not set, using default value")
endif()

if(DEFINED ENV{WIFI_PASS})
	add_compile_definitions(WIFI_PASS=\"$ENV{WIFI_PASS}\")
else()
	add_compile_definitions(WIFI_PASS="guest")
	message(WARNING "WIFI_PASS is not set, using default value")
endif()

if(DEFINED ENV{OTA_SECURE})
	add_compile_definitions(OTA_SECURE="1")
	message("Secure OTA update enabled")
else()
	message(WARNING "Secure OTA update disabled")
endif()

project(esp32-cam-motion)
