#pragma once
// simple gadget: control pipe, bulk in on 0x81, bulk out on 0x02, loopback between them.
#ifndef USB3_DEVICE_CONTROLLER_H
#define USB3_DEVICE_CONTROLLER_H

#include "usb3_types.h"
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>


using namespace std;
using namespace sc_core;
using namespace tlm;
using namespace tlm_utils;

#pragma pack(push, 1)

struct usb_device_descriptor_t {
    uint8_t  bLength            = 18;
    uint8_t  bDescriptorType    = USB_DESC_DEVICE;
    uint16_t bcdUSB             = 0x0300;
    uint8_t  bDeviceClass       = 0xFF;
    uint8_t  bDeviceSubClass    = 0x00;
    uint8_t  bDeviceProtocol    = 0x00;
    uint8_t  bMaxPacketSize0    = 9;      // 2^9 = 512 for ss ep0
    uint16_t idVendor           = 0xAB12;
    uint16_t idProduct          = 0xCD34;
    uint16_t bcdDevice          = 0x0100;
    uint8_t  iManufacturer      = 1;
    uint8_t  iProduct           = 2;
    uint8_t  iSerialNumber      = 3;
    uint8_t  bNumConfigurations = 1;
};

struct usb_config_descriptor_t {
    uint8_t  bLength             = 9;
    uint8_t  bDescriptorType     = USB_DESC_CONFIGURATION;
    uint16_t wTotalLength        = 9 + 9 + 7 + 6 + 7 + 6;
    uint8_t  bNumInterfaces      = 1;
    uint8_t  bConfigurationValue = 1;
    uint8_t  iConfiguration      = 0;
    uint8_t  bmAttributes        = 0xC0;
    uint8_t  bMaxPower           = 0;
};

struct usb_interface_descriptor_t {
    uint8_t bLength            = 9;
    uint8_t  bDescriptorType    = USB_DESC_INTERFACE;
    uint8_t bInterfaceNumber   = 0;
    uint8_t bAlternateSetting  = 0;
    uint8_t bNumEndpoints      = 2;
    uint8_t bInterfaceClass    = 0xFF;
    uint8_t bInterfaceSubClass = 0x00;
    uint8_t bInterfaceProtocol = 0x00;
    uint8_t iInterface         = 0;
};

struct usb_endpoint_descriptor_t {
    uint8_t  bLength          = 7;
    uint8_t  bDescriptorType  = USB_DESC_ENDPOINT;
    uint8_t  bEndpointAddress = 0;
    uint8_t  bmAttributes     = 0x02;
    uint16_t wMaxPacketSize   = 1024;
    uint8_t  bInterval        = 0;
};

struct usb_ss_ep_companion_descriptor_t {
    uint8_t bLength           = 6;
    uint8_t  bDescriptorType   = USB_DESC_SS_EP_COMP;
    uint8_t bMaxBurst         = 0;
    uint8_t bmAttributes      = 0;
    uint16_t wBytesPerInterval= 0;
};

#pragma pack(pop)

struct usb3_ep_fifo_t {
    vector<uint8_t> buffer;
    bool                 has_data = false;

    void write(const uint8_t* data, size_t len) {
        buffer.assign(data, data + len);
        has_data = true;
    }
    bool read(vector<uint8_t>& out) {
        if (!has_data) return false;
        out = buffer;
        buffer.clear();
        has_data = false;
        return true;
    }
};

SC_MODULE(usb3_device_controller_t) {

    simple_target_socket<usb3_device_controller_t>    from_protocol;
    simple_initiator_socket<usb3_device_controller_t> to_protocol;

    sc_in<bool>    enable;
    sc_in<uint8_t> dev_addr_in;
    sc_out<bool>   bulk_irq;
    sc_out<bool>   configured;

    SC_CTOR(usb3_device_controller_t) :
        from_protocol("from_protocol"),
        to_protocol("to_protocol"),
        enable("enable"),
        dev_addr_in("dev_addr_in"),
        bulk_irq("bulk_irq"),
        configured("configured"),
        m_configured(false)
    {
        from_protocol.register_b_transport(this, &usb3_device_controller_t::from_proto_b_transport);

        init_descriptors();

        SC_THREAD(loopback_process);
        sensitive << m_out_ep_event;
    }

    bool is_configured() const { return m_configured; }
    uint8_t assigned_address() const { return m_assigned_addr; }

private:
    bool    m_configured;
    uint8_t m_assigned_addr = 0;

    usb3_ep_fifo_t m_ep1_in;
    usb3_ep_fifo_t m_ep2_out;

    sc_event m_out_ep_event;

    usb_device_descriptor_t    m_dev_desc;
    usb_config_descriptor_t    m_cfg_desc;
    usb_interface_descriptor_t m_intf_desc;
    usb_endpoint_descriptor_t  m_ep1_desc;
    usb_endpoint_descriptor_t  m_ep2_desc;
    usb_ss_ep_companion_descriptor_t m_ep1_comp;
    usb_ss_ep_companion_descriptor_t m_ep2_comp;

    vector<uint8_t> m_config_blob;

    void init_descriptors();
    void loopback_process();
    void from_proto_b_transport(tlm_generic_payload& trans,
                                sc_time& delay);
    void handle_setup(const usb3_setup_pkt_t& setup,
                      tlm_generic_payload& trans,
                      sc_time& delay);
    void handle_bulk_out(tlm_generic_payload& trans,
                         sc_time& delay);
    void send_bulk_in(const vector<uint8_t>& data,
                      uint8_t ep, sc_time& delay);
    void stall_endpoint(uint8_t ep);
    void send_to_protocol(usb3_pkt_type_t pkt_type,
                          uint8_t endpoint,
                          usb3_dir_t direction,
                          vector<uint8_t>& data,
                          usb3_ep_type_t ep_type = usb3_ep_type_t::CONTROL);
    void handle_get_descriptor(const usb3_setup_pkt_t& setup,
                               tlm_generic_payload& trans,
                               sc_time& delay);
    void handle_set_address(const usb3_setup_pkt_t& setup,
                            sc_time& delay);
    void handle_set_configuration(const usb3_setup_pkt_t& setup,
                                  sc_time& delay);
};

#endif // usb3_device_controller_h
