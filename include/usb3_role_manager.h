#pragma once
//  usb 3.0 dual role device -- role manager
//  monitors usb-c cc pin state and vbus; decides host / device / none role.
//  supports software-triggered role override via request_role().
#ifndef USB3_ROLE_MANAGER_H
#define USB3_ROLE_MANAGER_H

#include "usb3_types.h"

using namespace std;
using namespace sc_core;
using namespace tlm;
using namespace tlm_utils;

//   role_changed : event pulse when role transition completes
SC_MODULE(usb3_role_manager_t) {

    sc_in<usb_c_cc_state_t> cc_state;       //< usb-c cc state
    sc_in<bool>          vbus;          //< vbus level

    sc_out<usb3_role_t>  current_role;  //< active role
    sc_out<bool>         role_changed;  //< pulse on transition

    SC_CTOR(usb3_role_manager_t) :
        cc_state("cc_state"),
        vbus("vbus"),
        current_role("current_role"),
        m_role(usb3_role_t::NONE),
        m_host_requested(false),
        m_device_requested(false)
    {
        SC_THREAD(role_detection_process);
    }

    usb3_role_t get_role() const { return m_role; }

    // request a role switch (e.g., from software / test-bench)
    void request_role(usb3_role_t r);

private:
    usb3_role_t m_role;
    bool        m_host_requested;
    bool        m_device_requested;

    sc_event m_role_request_event;

    void role_detection_process();
    void apply_role(usb3_role_t new_role);
};

#endif // usb3_role_manager_h
