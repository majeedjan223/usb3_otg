#pragma once
#ifndef USB3_HOST_CONTROLLER_H
#define USB3_HOST_CONTROLLER_H

#include "usb3_types.h"
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include <cassert>
#include <deque>
#include <vector>


using namespace std;
using namespace sc_core;
using namespace tlm;
using namespace tlm_utils;

struct usb3_trb_t {
    uint64_t         data_ptr   = 0;
    uint32_t         trb_length = 0;
    uint8_t          dev_addr   = 0;
    uint8_t          endpoint   = 0;
    usb3_dir_t       direction  = usb3_dir_t::HOST_TO_DEVICE;
    usb3_ep_type_t   ep_type    = usb3_ep_type_t::BULK;
    bool             ioc        = true;
    bool             chain      = false;
    usb3_completion_t completion = usb3_completion_t::SUCCESS;
    vector<uint8_t> payload;
};

// software ring of trbs (hard cap so we never grow forever).
struct usb3_transfer_ring_t {
    static constexpr size_t RING_DEPTH = 256;
    deque<usb3_trb_t> queue;

    void enqueue(const usb3_trb_t& trb) {
        assert(queue.size() < RING_DEPTH && "Transfer ring overflow");
        queue.push_back(trb);
    }
    usb3_trb_t dequeue() {
        assert(!queue.empty() && "Transfer ring underflow");
        usb3_trb_t t = queue.front();
        queue.pop_front();
        return t;
    }
    bool empty() const { return queue.empty(); }
};

struct usb3_device_info_t {
    uint8_t  addr          = 0;
    uint16_t vendor_id     = 0;
    uint16_t product_id    = 0;
    uint8_t  num_configs   = 0;
    uint8_t  active_config = 0;
    bool     enumerated    = false;
    usb3_speed_t speed     = usb3_speed_t::SUPER_SPEED;
};

SC_MODULE(usb3_host_controller_t) {

    simple_initiator_socket<usb3_host_controller_t> to_protocol;
    simple_target_socket<usb3_host_controller_t>    from_protocol;

    sc_in<bool>              enable;
    sc_out<bool>             transfer_done;
    sc_out<bool>             enumeration_done;
    sc_out<uint8_t>          dev_addr_out;

    SC_CTOR(usb3_host_controller_t) :
        to_protocol("to_protocol"),
        from_protocol("from_protocol"),
        enable("enable"),
        transfer_done("transfer_done"),
        enumeration_done("enumeration_done"),
        dev_addr_out("dev_addr_out"),
        m_next_addr(1)
    {
        from_protocol.register_b_transport(this, &usb3_host_controller_t::from_protocol_b_transport);

        SC_THREAD(host_main_process);
    }

    // both just enqueue a trb; transfer_done pulses when it completes. in data ends up in rx_buffer().
    void bulk_out(uint8_t dev_addr, uint8_t ep, const vector<uint8_t>& data);
    void bulk_in(uint8_t dev_addr, uint8_t ep, size_t len);

    const vector<uint8_t>& rx_buffer() const { return m_rx_buf; }
    const usb3_device_info_t& device_info() const { return m_dev; }

private:
    uint8_t                 m_next_addr;
    usb3_device_info_t      m_dev;
    usb3_transfer_ring_t    m_transfer_ring;
    vector<uint8_t>    m_rx_buf;
    vector<uint8_t>    m_resp_data;
    bool                    m_resp_valid = false;
    sc_event       m_trb_complete_event;

    void host_main_process();
    void enumerate_device();
    void process_transfer_ring();
    void enqueue_bulk_trb(uint8_t dev_addr,
                          uint8_t ep,
                          usb3_dir_t direction,
                          size_t len,
                          vector<uint8_t> payload = {});

    bool ctrl_transfer(uint8_t dev_addr,
                       const usb3_setup_pkt_t& setup,
                       vector<uint8_t>& data_buf,
                       bool host_to_device = false);

    bool get_descriptor(uint8_t dev_addr, uint8_t desc_type,
                        uint8_t desc_index, vector<uint8_t>& buf);
    bool set_address(uint8_t old_addr, uint8_t new_addr);
    bool set_configuration(uint8_t dev_addr, uint8_t config_val);

    void from_protocol_b_transport(tlm_generic_payload& trans,
                                   sc_time& delay);
};

#endif // usb3_host_controller_h
