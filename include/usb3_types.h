#pragma once
#ifndef USB3_TYPES_H
#define USB3_TYPES_H

#include <systemc>
#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>


using namespace std;
using namespace sc_core;
using namespace tlm;
using namespace tlm_utils;


// usb descriptor constants
static constexpr uint8_t  USB_DESC_DEVICE          = 0x01;
static constexpr uint8_t  USB_DESC_CONFIGURATION   = 0x02;
static constexpr uint8_t  USB_DESC_STRING          = 0x03;
static constexpr uint8_t  USB_DESC_INTERFACE       = 0x04;
static constexpr uint8_t  USB_DESC_ENDPOINT        = 0x05;
static constexpr uint8_t  USB_DESC_BOS             = 0x0F;
static constexpr uint8_t  USB_DESC_SS_EP_COMP      = 0x30;

// brequest codes (only those used in this model)
static constexpr uint8_t  USB_REQ_GET_STATUS       = 0x00;
static constexpr uint8_t  USB_REQ_SET_ADDRESS      = 0x05;
static constexpr uint8_t  USB_REQ_GET_DESCRIPTOR   = 0x06;
static constexpr uint8_t  USB_REQ_SET_CONFIGURATION= 0x09;

//  enumerations

// usb operating role for the drd
enum usb3_role_t : uint8_t {
    NONE   = 0,   //< undefined / after reset
    HOST   = 1,   //< acts as usb host (xhci-like)
    DEVICE = 2    //< acts as usb peripheral
};

inline const char* to_cstr(usb3_role_t r) {
    switch (r) {
        case usb3_role_t::HOST:   return "HOST";
        case usb3_role_t::DEVICE: return "DEVICE";
        default:                  return "NONE";
    }
}

// usb-c configuration channel (cc) state
enum usb_c_cc_state_t : uint8_t {
    CC_DISCONNECTED = 0,
    CC_UFP_ATTACHED = 1,  //< upstream facing port attached (we are dfp/host)
    CC_DFP_ATTACHED = 2   //< downstream facing port attached (we are ufp/device)
};

inline const char* to_cstr(usb_c_cc_state_t c) {
    switch (c) {
        case usb_c_cc_state_t::CC_UFP_ATTACHED: return "UFP_ATTACHED(Host_Mode)";
        case usb_c_cc_state_t::CC_DFP_ATTACHED: return "DFP_ATTACHED(Device_Mode)";
        default:                                return "DISCONNECTED";
    }
}

// usb 3.x superspeed link speed
enum usb3_speed_t : uint8_t {
    DISCONNECTED  = 0,
    FULL_SPEED    = 1,   // fs  12 mbps  (fallback)
    HIGH_SPEED    = 2,   // hs 480 mbps  (fallback)
    SUPER_SPEED   = 3,   // ss   5 gbps  usb 3.0
    SUPER_SPEED_PLUS = 4 // ss+ 10 gbps  usb 3.1
};

inline const char* to_cstr(usb3_speed_t s) {
    switch (s) {
        case usb3_speed_t::FULL_SPEED:       return "FS";
        case usb3_speed_t::HIGH_SPEED:       return "HS";
        case usb3_speed_t::SUPER_SPEED:      return "SS";
        case usb3_speed_t::SUPER_SPEED_PLUS: return "SS+";
        default:                             return "DISCONNECTED";
    }
}

// usb packet types (simplified)
enum usb3_pkt_type_t : uint8_t {
    // transaction packets
    TP_ACK      = 0x01,
    TP_NRDY     = 0x02,
    TP_ERDY     = 0x03,
    TP_STALL    = 0x05,
    // data packets
    DP_DATA     = 0x10,
    SETUP       = 0x40,
    CTRL_DATA   = 0x41,
    CTRL_STATUS = 0x42,
    // internal model-only
    BULK_IN     = 0x50,
    BULK_OUT    = 0x51
};

inline const char* to_cstr(usb3_pkt_type_t t) {
    switch (t) {
        case usb3_pkt_type_t::TP_ACK:      return "ACK";
        case usb3_pkt_type_t::TP_NRDY:     return "NRDY";
        case usb3_pkt_type_t::TP_ERDY:     return "ERDY";
        case usb3_pkt_type_t::TP_STALL:    return "STALL";
        case usb3_pkt_type_t::SETUP:       return "SETUP";
        case usb3_pkt_type_t::CTRL_DATA:   return "CTRL_DATA";
        case usb3_pkt_type_t::CTRL_STATUS: return "CTRL_STATUS";
        case usb3_pkt_type_t::DP_DATA:     return "DP_DATA";
        case usb3_pkt_type_t::BULK_IN:     return "BULK_IN";
        case usb3_pkt_type_t::BULK_OUT:    return "BULK_OUT";
        default:                           return "UNKNOWN";
    }
}

// usb transfer direction (from host perspective)
enum usb3_dir_t : uint8_t {
    HOST_TO_DEVICE = 0,
    DEVICE_TO_HOST = 1
};

// usb endpoint type
enum usb3_ep_type_t : uint8_t {
    CONTROL   = 0,
    ISOCH     = 1,
    BULK      = 2,
    INTERRUPT = 3
};

// completion codes used in this model
enum usb3_completion_t : uint8_t {
    SUCCESS   = 0,
    STALL_ERR = 5
};

//  usb setup packet (8 bytes, usb 2.0 §9.3 / usb 3.0 §9.3)
struct usb3_setup_pkt_t {
    uint8_t  bmRequestType = 0;
    uint8_t  bRequest      = 0;
    uint16_t wValue        = 0;
    uint16_t wIndex        = 0;
    uint16_t wLength       = 0;
};

//  tlm-2.0 extension: usb3 packet metadata
class usb3_pkt_ext_t : public tlm_extension<usb3_pkt_ext_t> {
public:
    usb3_pkt_type_t  pkt_type   = usb3_pkt_type_t::DP_DATA;
    uint8_t          dev_addr   = 0;    //< usb device address (0-127)
    uint8_t          endpoint   = 0;    //< endpoint number (0-15)
    usb3_dir_t       direction  = usb3_dir_t::HOST_TO_DEVICE;
    usb3_ep_type_t   ep_type    = usb3_ep_type_t::BULK;
    usb3_completion_t code      = usb3_completion_t::SUCCESS;
    uint32_t         seq_num    = 0;    //< packet sequence number
    usb3_setup_pkt_t setup      = {};   //< valid when pkt_type == setup

    tlm_extension_base* clone() const override {
        return new usb3_pkt_ext_t(*this);
    }
    void copy_from(tlm_extension_base const& ext) override {
        *this = static_cast<const usb3_pkt_ext_t&>(ext);
    }
};

inline usb3_pkt_ext_t make_usb3_ext(usb3_pkt_type_t pkt_type,
                                    uint8_t dev_addr,
                                    uint8_t endpoint,
                                    usb3_dir_t direction,
                                    usb3_ep_type_t ep_type) {
    usb3_pkt_ext_t ext;
    ext.pkt_type = pkt_type;
    ext.dev_addr = dev_addr;
    ext.endpoint = endpoint;
    ext.direction = direction;
    ext.ep_type = ep_type;
    return ext;
}

//  simple logging helper
inline void usb3_log(const string& module, const string& msg) {
    cout << "[" << sc_time_stamp().to_string() << "] " << module << ": " << msg << "\n";
}

// macro for convenience
#define USB3_LOG(msg) usb3_log(name(), msg)

#endif // usb3_types_h
