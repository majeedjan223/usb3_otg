cmake_minimum_required(VERSION 3.16)
project(usb3_drd_tlm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# SystemC: prefer pkg-config (e.g. libsystemc-dev); else find_path/find_library
# with optional SYSTEMC_HOME.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(SYSTEMC IMPORTED_TARGET systemc)
endif()

if(TARGET PkgConfig::SYSTEMC)
    set(_usb3_systemc PkgConfig::SYSTEMC)
else()
    if(DEFINED ENV{SYSTEMC_HOME})
        set(_root "$ENV{SYSTEMC_HOME}")
        set(_hints
            "${_root}" "${_root}/include" "${_root}/lib"
            "${_root}/lib-linux64" "${_root}/lib64")
    endif()
    find_path(SYSTEMC_INCLUDE_DIR NAMES systemc HINTS ${_hints})
    find_library(SYSTEMC_LIBRARY NAMES systemc HINTS ${_hints})
    if(NOT SYSTEMC_INCLUDE_DIR OR NOT SYSTEMC_LIBRARY)
        message(FATAL_ERROR
            "SystemC not found. On Ubuntu install libsystemc-dev, or set SYSTEMC_HOME.")
    endif()
    add_library(SystemC::systemc UNKNOWN IMPORTED)
    set_target_properties(SystemC::systemc PROPERTIES
        IMPORTED_LOCATION "${SYSTEMC_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${SYSTEMC_INCLUDE_DIR}")
    set(_usb3_systemc SystemC::systemc)
endif()

find_package(Threads REQUIRED)

add_executable(usb3_drd_sim
    src/usb3_protocol_layer.cpp
    src/usb3_host_controller.cpp
    src/usb3_device_controller.cpp
    src/usb3_role_manager.cpp
    src/usb3_drd_controller.cpp
    src/sc_main.cpp
)

target_include_directories(usb3_drd_sim PRIVATE include)
target_link_libraries(usb3_drd_sim PRIVATE ${_usb3_systemc} Threads::Threads)

if(MSVC)
    target_compile_options(usb3_drd_sim PRIVATE /W3 /wd4251)
    target_compile_definitions(usb3_drd_sim PRIVATE _SCL_SECURE_NO_WARNINGS)
else()
    target_compile_options(usb3_drd_sim PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS usb3_drd_sim DESTINATION bin)
