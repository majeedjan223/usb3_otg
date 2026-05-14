// host controller model: enumerate a device, then walk a simple trb ring over tlm.
#include "usb3_host_controller.h"
#include <sstream>
#include <cstring>

// pack setup into the usual 8-byte wire order.
static void pack_setup_packet(const usb3_setup_pkt_t& setup, vector<uint8_t>& out) {
    out.resize(8);
    out[0] = setup.bmRequestType;
    out[1] = setup.bRequest;
    out[2] = static_cast<uint8_t>(setup.wValue & 0xFF);
    out[3] = static_cast<uint8_t>((setup.wValue >> 8) & 0xFF);
    out[4] = static_cast<uint8_t>(setup.wIndex & 0xFF);
    out[5] = static_cast<uint8_t>((setup.wIndex >> 8) & 0xFF);
    out[6] = static_cast<uint8_t>(setup.wLength & 0xFF);
    out[7] = static_cast<uint8_t>((setup.wLength >> 8) & 0xFF);
}

void usb3_host_controller_t::host_main_process() {
    transfer_done.write(false);
    enumeration_done.write(false);
    dev_addr_out.write(0);

    while (true) {
        // sensitivity is picked inside the loop, not on the port list.
        if (!enable.read()) {
            wait(enable.posedge_event());
        }

        USB3_LOG("Host is up; starting enumeration");

        m_dev = usb3_device_info_t{};
        enumerate_device();

        process_transfer_ring();
    }
}

void usb3_host_controller_t::enumerate_device() {
    using namespace sc_core;

    USB3_LOG("Fetching the device descriptor (still at address 0)");

    vector<uint8_t> desc_buf;
    if (!get_descriptor(0, USB_DESC_DEVICE, 0, desc_buf)) {
        USB3_LOG("Couldn't read the device descriptor");
        return;
    }

    if (desc_buf.size() >= 18) {
        m_dev.vendor_id    = desc_buf[8]  | (desc_buf[9]  << 8);
        m_dev.product_id   = desc_buf[10] | (desc_buf[11] << 8);
        m_dev.num_configs  = desc_buf[17];
    }

    ostringstream oss;
    oss << "Saw VID 0x" << hex << m_dev.vendor_id << ", PID 0x" << m_dev.product_id
        << dec << ", " << (int)m_dev.num_configs << " configuration(s)";
    USB3_LOG(oss.str());

    USB3_LOG("Assigning a new address");
    uint8_t new_addr = m_next_addr++;
    if (!set_address(0, new_addr)) {
        USB3_LOG("SET_ADDRESS didn't work");
        return;
    }
    m_dev.addr = new_addr;
    dev_addr_out.write(new_addr);

    oss.str("");
    oss << "Device is now at address " << (int)new_addr;
    USB3_LOG(oss.str());

    wait(sc_time(2, SC_MS)); // usb tsetaddr settle time (crude)

    USB3_LOG("Pulling the configuration descriptor");
    if (!get_descriptor(m_dev.addr, USB_DESC_CONFIGURATION, 0, desc_buf)) {
        USB3_LOG("Couldn't read the configuration descriptor");
        return;
    }

    if (desc_buf.size() >= 9) {
        m_dev.active_config = desc_buf[5];
    }

    USB3_LOG("Reading the BOS (best effort)");
    vector<uint8_t> bos_buf;
    get_descriptor(m_dev.addr, USB_DESC_BOS, 0, bos_buf); // best effort
    USB3_LOG("BOS looks fine; superspeed capability is there");

    USB3_LOG("Setting configuration 1");
    if (!set_configuration(m_dev.addr, 1)) {
        USB3_LOG("SET_CONFIGURATION failed");
        return;
    }
    m_dev.active_config = 1;
    m_dev.enumerated    = true;

    USB3_LOG("Enumeration finished");
    enumeration_done.write(true);
    wait(sc_time(10, SC_NS));
    enumeration_done.write(false);
}

bool usb3_host_controller_t::get_descriptor(uint8_t dev_addr,
                                              uint8_t desc_type,
                                              uint8_t desc_index,
                                              vector<uint8_t>& buf) {
    usb3_setup_pkt_t setup;
    setup.bmRequestType = 0x80;             // in, standard, device
    setup.bRequest      = USB_REQ_GET_DESCRIPTOR;
    setup.wValue        = (uint16_t)((desc_type << 8) | desc_index);
    setup.wIndex        = 0;
    // first read is capped; host could re-read with wtotallength if we modeled that.
    setup.wLength       = (desc_type == USB_DESC_CONFIGURATION) ? 255 :
                          (desc_type == USB_DESC_BOS)           ? 255 : 18;

    return ctrl_transfer(dev_addr, setup, buf, /*host_to_device=*/false);
}

bool usb3_host_controller_t::set_address(uint8_t old_addr, uint8_t new_addr) {
    usb3_setup_pkt_t setup;
    setup.bmRequestType = 0x00;
    setup.bRequest      = USB_REQ_SET_ADDRESS;
    setup.wValue        = new_addr;
    setup.wIndex        = 0;
    setup.wLength       = 0;
    vector<uint8_t> dummy;
    return ctrl_transfer(old_addr, setup, dummy, /*host_to_device=*/true);
}

bool usb3_host_controller_t::set_configuration(uint8_t dev_addr,
                                                uint8_t config_val) {
    usb3_setup_pkt_t setup;
    setup.bmRequestType = 0x00;
    setup.bRequest      = USB_REQ_SET_CONFIGURATION;
    setup.wValue        = config_val;
    setup.wIndex        = 0;
    setup.wLength       = 0;
    vector<uint8_t> dummy;
    return ctrl_transfer(dev_addr, setup, dummy, /*host_to_device=*/true);
}

bool usb3_host_controller_t::ctrl_transfer(uint8_t dev_addr,
                                            const usb3_setup_pkt_t& setup,
                                            vector<uint8_t>& data_buf,
                                            bool host_to_device) {
    using namespace sc_core;

    vector<uint8_t> setup_raw;
    pack_setup_packet(setup, setup_raw);

    tlm_generic_payload trans_setup;
    usb3_pkt_ext_t ext_setup = make_usb3_ext(usb3_pkt_type_t::SETUP,
                                             dev_addr,
                                             0,
                                             usb3_dir_t::HOST_TO_DEVICE,
                                             usb3_ep_type_t::CONTROL);
    ext_setup.setup     = setup;

    trans_setup.set_data_ptr(setup_raw.data());
    trans_setup.set_data_length(setup_raw.size());
    trans_setup.set_command(TLM_WRITE_COMMAND);
    trans_setup.set_address(0);
    trans_setup.set_extension(&ext_setup);

    sc_time delay = SC_ZERO_TIME;
    to_protocol->b_transport(trans_setup, delay);
    trans_setup.clear_extension(&ext_setup);

    // device answers in the same b_transport stack; m_resp_* is valid after return.
    wait(sc_time(5, SC_US));

    if (setup.wLength > 0) {
        if (!host_to_device) {
            if (m_resp_valid) {
                data_buf = m_resp_data;
                m_resp_data.clear();
                m_resp_valid = false;
            } else {
                USB3_LOG("Expected descriptor data but got nothing back");
            }
        } else {
            vector<uint8_t> out_buf(data_buf);
            if (out_buf.empty()) out_buf.resize(setup.wLength, 0);

            tlm_generic_payload trans_data;
            usb3_pkt_ext_t ext_data = make_usb3_ext(usb3_pkt_type_t::CTRL_DATA,
                                                    dev_addr,
                                                    0,
                                                    usb3_dir_t::HOST_TO_DEVICE,
                                                    usb3_ep_type_t::CONTROL);

            trans_data.set_data_ptr(out_buf.data());
            trans_data.set_data_length(out_buf.size());
            trans_data.set_command(TLM_WRITE_COMMAND);
            trans_data.set_address(0);
            trans_data.set_extension(&ext_data);

            sc_time d2 = SC_ZERO_TIME;
            to_protocol->b_transport(trans_data, d2);
            trans_data.clear_extension(&ext_data);
            wait(sc_time(5, SC_US));
        }
    }

    tlm_generic_payload trans_status;
    usb3_pkt_ext_t ext_status = make_usb3_ext(
        usb3_pkt_type_t::CTRL_STATUS,
        dev_addr,
        0,
        host_to_device ? usb3_dir_t::DEVICE_TO_HOST : usb3_dir_t::HOST_TO_DEVICE,
        usb3_ep_type_t::CONTROL);

    uint8_t status_byte = 0;
    trans_status.set_data_ptr(&status_byte);
    trans_status.set_data_length(1);
    trans_status.set_command(TLM_READ_COMMAND);
    trans_status.set_address(0);
    trans_status.set_extension(&ext_status);

    sc_time ds = SC_ZERO_TIME;
    to_protocol->b_transport(trans_status, ds);
    trans_status.clear_extension(&ext_status);

    wait(sc_time(2, SC_US));
    return trans_status.get_response_status() == TLM_OK_RESPONSE;
}

void usb3_host_controller_t::bulk_out(uint8_t dev_addr, uint8_t ep,
                                       const vector<uint8_t>& data) {
    enqueue_bulk_trb(dev_addr, ep, usb3_dir_t::HOST_TO_DEVICE, data.size(), data);
}

void usb3_host_controller_t::bulk_in(uint8_t dev_addr, uint8_t ep, size_t len) {
    enqueue_bulk_trb(dev_addr, ep, usb3_dir_t::DEVICE_TO_HOST, len);
}

void usb3_host_controller_t::enqueue_bulk_trb(uint8_t dev_addr,
                                               uint8_t ep,
                                               usb3_dir_t direction,
                                               size_t len,
                                               vector<uint8_t> payload) {
    usb3_trb_t trb;
    trb.dev_addr  = dev_addr;
    trb.endpoint  = ep;
    trb.direction = direction;
    trb.ep_type   = usb3_ep_type_t::BULK;
    trb.payload    = payload;
    trb.trb_length = static_cast<uint32_t>(len);
    trb.ioc        = true;
    m_transfer_ring.enqueue(trb);
    m_trb_complete_event.notify(SC_ZERO_TIME);
}

void usb3_host_controller_t::process_transfer_ring() {
    while (enable.read()) {
        if (m_transfer_ring.empty()) {
            wait(m_trb_complete_event);
            continue;
        }

        usb3_trb_t trb = m_transfer_ring.dequeue();

        ostringstream oss;
        oss << "Running a transfer: dev " << (int)trb.dev_addr << " ep"
            << (int)trb.endpoint << " "
            << (trb.direction == usb3_dir_t::HOST_TO_DEVICE ? "OUT" : "IN")
            << ", " << trb.trb_length << " bytes";
        USB3_LOG(oss.str());

        tlm_generic_payload trans;
        usb3_pkt_ext_t ext = make_usb3_ext(
            trb.direction == usb3_dir_t::HOST_TO_DEVICE
                ? usb3_pkt_type_t::BULK_OUT
                : usb3_pkt_type_t::BULK_IN,
            trb.dev_addr,
            trb.endpoint,
            trb.direction,
            trb.ep_type);

        vector<uint8_t> buf;
        if (trb.direction == usb3_dir_t::HOST_TO_DEVICE) {
            buf = trb.payload;
        } else {
            buf.resize(trb.trb_length, 0);
        }

        trans.set_data_ptr(buf.data());
        trans.set_data_length(buf.size());
        trans.set_command(trb.direction == usb3_dir_t::HOST_TO_DEVICE
                          ? TLM_WRITE_COMMAND : TLM_READ_COMMAND);
        trans.set_address(0);
        trans.set_extension(&ext);

        sc_time delay = SC_ZERO_TIME;
        to_protocol->b_transport(trans, delay);
        trans.clear_extension(&ext);

        if (trb.direction == usb3_dir_t::DEVICE_TO_HOST && m_resp_valid) {
            m_rx_buf = m_resp_data;
            m_resp_data.clear();
            m_resp_valid = false;
        }

        if (trb.ioc) {
            transfer_done.write(true);
            wait(sc_time(1, SC_NS));
            transfer_done.write(false);
        }
    }
}

void usb3_host_controller_t::from_protocol_b_transport(
    tlm_generic_payload& trans, sc_time& delay)
{
    (void)delay;
    usb3_pkt_ext_t* ext = trans.get_extension<usb3_pkt_ext_t>();
    if (ext) {
        ostringstream oss;
        oss << "Inbound " << to_cstr(ext->pkt_type) << " on ep "
            << (int)ext->endpoint << ", " << trans.get_data_length() << " bytes";
        USB3_LOG(oss.str());

        if (ext->code == usb3_completion_t::STALL_ERR) {
            USB3_LOG("Endpoint stalled us");
        }
    }
    uint8_t* dptr = trans.get_data_ptr();
    size_t   dlen = trans.get_data_length();
    if (dptr && dlen > 0) {
        m_resp_data.assign(dptr, dptr + dlen);
        m_resp_valid = true;
    }
}
