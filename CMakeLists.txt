# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
set(EXTRA_COMPONENT_DIRS "main" "components/user_analogOutput" "components/user_ble")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gatt_server_service_table_demo)
