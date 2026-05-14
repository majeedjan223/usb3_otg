#pragma once
//  routes transaction packets (tp), data packets (dp), header packets (hp),
//  the link layer.
#ifndef USB3_PROTOCOL_LAYER_H
#define USB3_PROTOCOL_LAYER_H

#include "usb3_types.h"
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>

using namespace std;
using namespace sc_core;
using namespace tlm;
using namespace tlm_utils;

SC_MODULE(usb3_protocol_layer_t) {

    simple_target_socket<usb3_protocol_layer_t>    from_host;
    simple_initiator_socket<usb3_protocol_layer_t> to_host;
    simple_target_socket<usb3_protocol_layer_t>    from_device;
    simple_initiator_socket<usb3_protocol_layer_t> to_device;
    simple_initiator_socket<usb3_protocol_layer_t> to_link;
    simple_target_socket<usb3_protocol_layer_t>    from_link;

    sc_in<usb3_role_t>       current_role;  //< host or device

    SC_CTOR(usb3_protocol_layer_t) :
        from_host("from_host"),
        to_host("to_host"),
        from_device("from_device"),
        to_device("to_device"),
        to_link("to_link"),
        from_link("from_link"),
        current_role("current_role"),
        m_seq_num(0)
    {
        from_host.register_b_transport(this, &usb3_protocol_layer_t::host_b_transport);
        from_device.register_b_transport(this, &usb3_protocol_layer_t::device_b_transport);
        from_link.register_b_transport(this, &usb3_protocol_layer_t::link_b_transport);
    }

    uint32_t next_seq() { return m_seq_num++; }

private:
    uint32_t m_seq_num;        //< packet sequence number generator

    void host_b_transport(tlm_generic_payload& trans,
                          sc_time& delay);
    void device_b_transport(tlm_generic_payload& trans,
                            sc_time& delay);
    void link_b_transport(tlm_generic_payload& trans,
                          sc_time& delay);

    void controller_tx_b_transport(tlm_generic_payload& trans,
                                   sc_time& delay);
    void route_tx(tlm_generic_payload& trans, sc_time& delay);
    void handle_rx(tlm_generic_payload& trans, sc_time& delay);
    void forward_to_active_controller(tlm_generic_payload& trans,
                                      sc_time& delay);

    static bool uses_sequence(usb3_pkt_type_t pkt_type);
};

#endif // usb3_protocol_layer_h
