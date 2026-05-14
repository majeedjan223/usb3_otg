#include "usb3_role_manager.h"
#include <sstream>


using namespace std;
using namespace sc_core;
using namespace tlm;
using namespace tlm_utils;

// monitors cc_state and vbus

void usb3_role_manager_t::role_detection_process() {
    current_role.write(usb3_role_t::NONE);
    role_changed.write(false);

    while (true) {
        wait(cc_state.value_changed_event()
           | vbus.value_changed_event()
           | m_role_request_event);

        usb_c_cc_state_t cc = cc_state.read();
        bool vb = vbus.read();
        usb3_role_t desired;
        if (cc == usb_c_cc_state_t::CC_UFP_ATTACHED && vb) {
            desired = usb3_role_t::HOST;
        } else if (cc == usb_c_cc_state_t::CC_DFP_ATTACHED && vb) {
            desired = usb3_role_t::DEVICE;
        } else {
            desired = usb3_role_t::NONE;
        }

        if (desired != m_role || m_host_requested || m_device_requested) {
            if (m_host_requested)   desired = usb3_role_t::HOST;
            if (m_device_requested) desired = usb3_role_t::DEVICE;
            m_host_requested   = false;
            m_device_requested = false;
            apply_role(desired);
        }
    }
}

void usb3_role_manager_t::apply_role(usb3_role_t new_role) {
    if (new_role == m_role) return;

    ostringstream oss;
    oss << "Moving from " << to_cstr(m_role) << " to " << to_cstr(new_role);
    USB3_LOG(oss.str());

    m_role = new_role;
    current_role.write(new_role);

    // pulse role_changed
    role_changed.write(true);
    wait(sc_time(10, SC_NS));
    role_changed.write(false);
}

void usb3_role_manager_t::request_role(usb3_role_t r) {
    ostringstream oss;
    oss << "Firmware requested " << to_cstr(r);
    USB3_LOG(oss.str());
    if (r == usb3_role_t::HOST)   m_host_requested   = true;
    if (r == usb3_role_t::DEVICE) m_device_requested = true;
    m_role_request_event.notify(SC_ZERO_TIME);
}

