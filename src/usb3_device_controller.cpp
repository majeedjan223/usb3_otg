// device side: ep0 control plus bulk loopback (ep2 out -> ep1 in).
#include "usb3_device_controller.h"
#include <sstream>
#include <cstring>
#include <iomanip>

using namespace std;
using namespace sc_core;
using namespace tlm;
using namespace tlm_utils;

static void append_bytes(vector<uint8_t>& dest, const void* ptr, size_t nbytes) {
    const uint8_t* b = static_cast<const uint8_t*>(ptr);
    for (size_t i = 0; i < nbytes; ++i) {
        dest.push_back(b[i]);
    }
}

static string hex_u8(uint8_t x) {
    ostringstream o;
    o << hex << setw(2) << setfill('0') << (int)x;
    return o.str();
}

static vector<uint8_t> dev_desc_to_bytes(const usb_device_descriptor_t& d) {
    vector<uint8_t> v(sizeof(d));
    memcpy(v.data(), &d, sizeof(d));
    return v;
}

void usb3_device_controller_t::init_descriptors() {
    m_ep1_desc.bEndpointAddress = 0x81;
    m_ep1_desc.bmAttributes     = 0x02; // bulk

    m_ep2_desc.bEndpointAddress = 0x02;
    m_ep2_desc.bmAttributes     = 0x02;

    m_config_blob.clear();

    append_bytes(m_config_blob, &m_cfg_desc, sizeof(m_cfg_desc));
    append_bytes(m_config_blob, &m_intf_desc, sizeof(m_intf_desc));
    append_bytes(m_config_blob, &m_ep1_desc, sizeof(m_ep1_desc));
    append_bytes(m_config_blob, &m_ep1_comp, sizeof(m_ep1_comp));
    append_bytes(m_config_blob, &m_ep2_desc, sizeof(m_ep2_desc));
    append_bytes(m_config_blob, &m_ep2_comp, sizeof(m_ep2_comp));

    uint16_t total = static_cast<uint16_t>(m_config_blob.size());
    m_config_blob[2] = total & 0xFF;
    m_config_blob[3] = (total >> 8) & 0xFF;
}

void usb3_device_controller_t::from_proto_b_transport(
    tlm_generic_payload& trans, sc_time& delay)
{
    if (!enable.read()) {
        USB3_LOG("Got a packet while the device block is off so dropping it");
        trans.set_response_status(TLM_GENERIC_ERROR_RESPONSE);
        return;
    }

    usb3_pkt_ext_t* ext = trans.get_extension<usb3_pkt_ext_t>();
    if (!ext) {
        USB3_LOG("Inbound packet had no usb3 extension so cannot handle it");
        trans.set_response_status(TLM_GENERIC_ERROR_RESPONSE);
        return;
    }

    ostringstream oss;
    oss << "Device side saw " << to_cstr(ext->pkt_type) << " on ep "
        << (int)ext->endpoint << ", " << trans.get_data_length() << " bytes";
    USB3_LOG(oss.str());

    switch (ext->pkt_type) {
        case usb3_pkt_type_t::SETUP:
            handle_setup(ext->setup, trans, delay);
            break;
        case usb3_pkt_type_t::BULK_OUT:
        case usb3_pkt_type_t::CTRL_DATA:
            handle_bulk_out(trans, delay);
            break;
        case usb3_pkt_type_t::CTRL_STATUS:
            {
                vector<uint8_t> zlp(1, 0);
                send_to_protocol(usb3_pkt_type_t::CTRL_STATUS,
                                 0,
                                 usb3_dir_t::DEVICE_TO_HOST,
                                 zlp);
            }
            trans.set_response_status(TLM_OK_RESPONSE);
            break;
        case usb3_pkt_type_t::BULK_IN:
            {
                vector<uint8_t> data;
                if (m_ep1_in.read(data)) {
                    send_bulk_in(data, 1, delay);
                } else {
                    USB3_LOG("EP1 FIFO was empty, would have NRDY on real hardware");
                    // real link would nrdy; we just skip for now.
                }
            }
            trans.set_response_status(TLM_OK_RESPONSE);
            break;
        default:
            USB3_LOG("Do not know what to do with this packet type");
            trans.set_response_status(TLM_GENERIC_ERROR_RESPONSE);
            break;
    }
}

void usb3_device_controller_t::handle_setup(const usb3_setup_pkt_t& setup,
                                              tlm_generic_payload& trans,
                                              sc_time& delay) {
    USB3_LOG(string("SETUP: bmRequestType 0x") + hex_u8(setup.bmRequestType) +
             ", bRequest 0x" + hex_u8(setup.bRequest));

    switch (setup.bRequest) {
        case USB_REQ_GET_DESCRIPTOR:
            handle_get_descriptor(setup, trans, delay);
            break;
        case USB_REQ_SET_ADDRESS:
            handle_set_address(setup, delay);
            trans.set_response_status(TLM_OK_RESPONSE);
            break;
        case USB_REQ_SET_CONFIGURATION:
            handle_set_configuration(setup, delay);
            trans.set_response_status(TLM_OK_RESPONSE);
            break;
        case USB_REQ_GET_STATUS: {
            vector<uint8_t> status(2, 0);
            send_to_protocol(usb3_pkt_type_t::CTRL_DATA,
                             0,
                             usb3_dir_t::DEVICE_TO_HOST,
                             status);
            trans.set_response_status(TLM_OK_RESPONSE);
            break;
        }
        default:
            USB3_LOG("Request is not implemented; stalling EP0");
            stall_endpoint(0);
            trans.set_response_status(TLM_GENERIC_ERROR_RESPONSE);
            break;
    }
}

void usb3_device_controller_t::handle_get_descriptor(
    const usb3_setup_pkt_t& setup,
    tlm_generic_payload& trans,
    sc_time& delay)
{
    (void)delay;
    uint8_t desc_type  = (setup.wValue >> 8) & 0xFF;
    uint8_t desc_index = setup.wValue & 0xFF;
    (void)desc_index;

    vector<uint8_t> resp_data;

    switch (desc_type) {
        case USB_DESC_DEVICE:
            resp_data = dev_desc_to_bytes(m_dev_desc);
            USB3_LOG("replying with the device descriptor (" +
                     to_string(resp_data.size()) + " bytes)");
            break;
        case USB_DESC_CONFIGURATION:
            resp_data = m_config_blob;
            if (resp_data.size() > setup.wLength) {
                resp_data.resize(setup.wLength);
            }
            USB3_LOG("replying with the configuration blob (" +
                     to_string(resp_data.size()) + " bytes)");
            break;
        case USB_DESC_BOS: {
            // hand-rolled bos + ss device capability
            static const uint8_t bos[] = {
                0x05, 0x0F,
                0x16, 0x00,
                0x01,
                0x0A, 0x10, 0x03,
                0x00,
                0x0E, 0x00,
                0x01,
                0x0A,
                0xFF, 0x07
            };
            resp_data.assign(bos, bos + sizeof(bos));
            if (resp_data.size() > setup.wLength)
                resp_data.resize(setup.wLength);
            USB3_LOG("replying with BOS (" +
                     to_string(resp_data.size()) + " bytes)");
            break;
        }
        case USB_DESC_STRING: {
            const char* text = nullptr;
            if (desc_index == 1) text = "MyVendor";
            else if (desc_index == 2) text = "USB3 DRD";
            else if (desc_index == 3) text = "SN-001";

            if (desc_index == 0) {
                resp_data = {0x04, 0x03, 0x09, 0x04};
            } else if (text != nullptr) {
                size_t n = strlen(text);
                resp_data.push_back(static_cast<uint8_t>(2 + n * 2));
                resp_data.push_back(0x03);
                for (size_t i = 0; i < n; ++i) {
                    resp_data.push_back(static_cast<uint8_t>(text[i]));
                    resp_data.push_back(0x00);
                }
            }
            if (resp_data.size() > setup.wLength)
                resp_data.resize(setup.wLength);
            break;
        }
        default:
            USB3_LOG(string("unknown descriptor type 0x") + hex_u8(desc_type));
            stall_endpoint(0);
            trans.set_response_status(TLM_GENERIC_ERROR_RESPONSE);
            return;
    }

    send_to_protocol(usb3_pkt_type_t::CTRL_DATA,
                     0,
                     usb3_dir_t::DEVICE_TO_HOST,
                     resp_data);
    trans.set_response_status(TLM_OK_RESPONSE);
}

void usb3_device_controller_t::handle_set_address(const usb3_setup_pkt_t& setup,
                                                   sc_time& delay) {
    (void)delay;
    m_assigned_addr = static_cast<uint8_t>(setup.wValue & 0x7F);
    ostringstream oss;
    oss << "Address is now " << (int)m_assigned_addr;
    USB3_LOG(oss.str());
    wait(sc_time(2, SC_MS));
}

void usb3_device_controller_t::handle_set_configuration(
    const usb3_setup_pkt_t& setup, sc_time& delay)
{
    (void)delay;
    ostringstream oss;
    oss << "Configuration set to " << (int)(setup.wValue & 0xFF);
    USB3_LOG(oss.str());
    m_configured = true;
    configured.write(true);
}

void usb3_device_controller_t::handle_bulk_out(tlm_generic_payload& trans,
                                                sc_time& delay) {
    (void)delay;
    uint8_t* data = trans.get_data_ptr();
    size_t   len  = trans.get_data_length();

    ostringstream oss;
    oss << "Bulk OUT on EP2: " << len << " bytes landed";
    USB3_LOG(oss.str());

    m_ep2_out.write(data, len);
    m_out_ep_event.notify(SC_ZERO_TIME);
    trans.set_response_status(TLM_OK_RESPONSE);
}

void usb3_device_controller_t::send_bulk_in(const vector<uint8_t>& data,
                                              uint8_t ep,
                                              sc_time& delay) {
    (void)delay;
    ostringstream oss;
    oss << "Bulk IN on EP1: pushing " << data.size() << " bytes upstream";
    USB3_LOG(oss.str());

    vector<uint8_t> tx_buf(data);
    send_to_protocol(usb3_pkt_type_t::BULK_IN,
                     ep,
                     usb3_dir_t::DEVICE_TO_HOST,
                     tx_buf,
                     usb3_ep_type_t::BULK);
}

void usb3_device_controller_t::stall_endpoint(uint8_t ep) {
    vector<uint8_t> buf(4, 0);
    send_to_protocol(usb3_pkt_type_t::TP_STALL,
                     ep,
                     usb3_dir_t::DEVICE_TO_HOST,
                     buf);
}

void usb3_device_controller_t::send_to_protocol(usb3_pkt_type_t pkt_type,
                                                 uint8_t endpoint,
                                                 usb3_dir_t direction,
                                                 vector<uint8_t>& data,
                                                 usb3_ep_type_t ep_type) {
    tlm_generic_payload trans;
    usb3_pkt_ext_t ext = make_usb3_ext(pkt_type,
                                       m_assigned_addr,
                                       endpoint,
                                       direction,
                                       ep_type);

    trans.set_data_ptr(data.data());
    trans.set_data_length(data.size());
    trans.set_command(TLM_WRITE_COMMAND);
    trans.set_address(0);
    trans.set_extension(&ext);

    sc_time delay = SC_ZERO_TIME;
    to_protocol->b_transport(trans, delay);
    trans.clear_extension(&ext);
}

void usb3_device_controller_t::loopback_process() {
    while (true) {
        wait(m_out_ep_event);

        if (!m_configured) {
            USB3_LOG("Loopback skipped (device not configured yet)");
            continue;
        }

        vector<uint8_t> data;
        if (m_ep2_out.read(data)) {
            USB3_LOG("Loopback copied " + to_string(data.size()) +
                     " bytes from EP2 to EP1");

            m_ep1_in.write(data.data(), data.size());

            vector<uint8_t> erdy = {1, 0, 0, 0};
            send_to_protocol(usb3_pkt_type_t::TP_ERDY,
                             1,
                             usb3_dir_t::DEVICE_TO_HOST,
                             erdy);

            bulk_irq.write(true);
            wait(sc_time(10, SC_NS));
            bulk_irq.write(false);
        }
    }
}
