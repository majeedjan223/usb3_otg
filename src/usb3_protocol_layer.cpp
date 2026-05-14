#include "usb3_protocol_layer.h"
#include <sstream>


using namespace std;
using namespace sc_core;
using namespace tlm;
using namespace tlm_utils;


void usb3_protocol_layer_t::controller_tx_b_transport(tlm_generic_payload& trans,
                                                       sc_time& delay) {
    route_tx(trans, delay);
}

void usb3_protocol_layer_t::host_b_transport(tlm_generic_payload& trans,
                                              sc_time& delay) {
    controller_tx_b_transport(trans, delay);
}

void usb3_protocol_layer_t::device_b_transport(tlm_generic_payload& trans,
                                                sc_time& delay) {
    controller_tx_b_transport(trans, delay);
}

void usb3_protocol_layer_t::link_b_transport(tlm_generic_payload& trans,
                                              sc_time& delay) {
    handle_rx(trans, delay);
}

void usb3_protocol_layer_t::route_tx(tlm_generic_payload& trans,
                                      sc_time& delay) {
    usb3_pkt_ext_t* ext = trans.get_extension<usb3_pkt_ext_t>();
    if (!ext) {
        // attach default extension
        usb3_pkt_ext_t def_ext;
        def_ext.pkt_type = usb3_pkt_type_t::DP_DATA;
        trans.set_extension(&def_ext);
        to_link->b_transport(trans, delay);
        trans.clear_extension(&def_ext);
        return;
    }

    if (uses_sequence(ext->pkt_type)) {
        ext->seq_num = next_seq();
    }

    ostringstream oss;
    oss << "Sending " << to_cstr(ext->pkt_type) << ", dev " << (int)ext->dev_addr
        << " ep" << (int)ext->endpoint << ", " << trans.get_data_length()
        << " bytes, seq " << ext->seq_num;
    USB3_LOG(oss.str());

    to_link->b_transport(trans, delay);
}

void usb3_protocol_layer_t::handle_rx(tlm_generic_payload& trans,
                                       sc_time& delay) {
    usb3_pkt_ext_t* ext = trans.get_extension<usb3_pkt_ext_t>();
    if (!ext) {
        forward_to_active_controller(trans, delay);
        return;
    }

    // handle transaction packet responses
    switch (ext->pkt_type) {
        case usb3_pkt_type_t::TP_ACK: {
            ostringstream oss;
            oss << "Got ACK, seq " << ext->seq_num;
            USB3_LOG(oss.str());
            break;
        }
        case usb3_pkt_type_t::TP_NRDY:
            USB3_LOG("Peer sent NRDY (their buffer wasn't ready)");
            // in a full model, we'd schedule a retry. here just log.
            break;
        case usb3_pkt_type_t::TP_ERDY:
            USB3_LOG("Peer sent ERDY (they're ready for another try)");
            break;
        case usb3_pkt_type_t::TP_STALL: {
            ostringstream oss;
            oss << "Got STALL on ep " << (int)ext->endpoint;
            USB3_LOG(oss.str());
            ext->code = usb3_completion_t::STALL_ERR;
            forward_to_active_controller(trans, delay);
            break;
        }

        default:
            forward_to_active_controller(trans, delay);
            break;
    }
}

void usb3_protocol_layer_t::forward_to_active_controller(tlm_generic_payload& trans,
                                                          sc_time& delay) {
    switch (current_role.read()) {
        case usb3_role_t::HOST:
            to_host->b_transport(trans, delay);
            break;
        case usb3_role_t::DEVICE:
            to_device->b_transport(trans, delay);
            break;
        default:
            trans.set_response_status(TLM_INCOMPLETE_RESPONSE);
            break;
    }
}

bool usb3_protocol_layer_t::uses_sequence(usb3_pkt_type_t pkt_type) {
    switch (pkt_type) {
        case usb3_pkt_type_t::DP_DATA:
        case usb3_pkt_type_t::BULK_OUT:
        case usb3_pkt_type_t::BULK_IN:
        case usb3_pkt_type_t::SETUP:
            return true;
        default:
            return false;
    }
}
