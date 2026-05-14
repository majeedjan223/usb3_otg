#pragma once
// ============================================================================
//  usb3_drd_controller.h
//  usb 3.0 dual role device -- top-level controller
//  instantiates and interconnects: protocol layer, host controller,
//  device controller, and role manager.
//  phy and link layers are not modelled.
// ============================================================================
#ifndef USB3_DRD_CONTROLLER_H
#define USB3_DRD_CONTROLLER_H

#include "usb3_types.h"
// phy and link layers removed for simpler abstraction
#include "usb3_protocol_layer.h"
#include "usb3_host_controller.h"
#include "usb3_device_controller.h"
#include "usb3_role_manager.h"
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>


using namespace std;
using namespace sc_core;
using namespace tlm;
using namespace tlm_utils;

//  drd register map (cfg_socket address space, 4-byte registers)
namespace usb3_drd_regs {
    static constexpr uint64_t CTRL         = 0x00;  //< global control (reset, enable)
    static constexpr uint64_t STATUS       = 0x04;  //< role, speed, flags
    static constexpr uint64_t ROLE_CTRL    = 0x08;  //< force role (1=host, 2=device)
    static constexpr uint64_t INTR_STATUS  = 0x0C;  //< interrupt status (w1c)
    static constexpr uint64_t INTR_ENABLE  = 0x10;  //< interrupt enable mask
    static constexpr uint64_t DEV_ADDR     = 0x14;  //< assigned device address
    static constexpr uint64_t SPEED        = 0x1C;  //< negotiated speed
}

// interrupt bits
static constexpr uint32_t DRD_INTR_ROLE_CHANGED  = (1u << 0);
static constexpr uint32_t DRD_INTR_ENUM_DONE     = (1u << 3);
static constexpr uint32_t DRD_INTR_XFER_DONE     = (1u << 4);
static constexpr uint32_t DRD_INTR_BULK_IRQ      = (1u << 5);

SC_MODULE(usb3_drd_controller_t) {
    sc_in<usb_c_cc_state_t>  cc_state;
    sc_in<bool>  vbus;
    sc_out<bool> irq;          // level-sensitive interrupt

    simple_target_socket<usb3_drd_controller_t>    cfg_socket;

    // upstream from peer (connect peer initiator → this target)
    simple_target_socket<usb3_drd_controller_t>    upstream;

    usb3_protocol_layer_t   protocol;
    usb3_host_controller_t  host_ctrl;
    usb3_device_controller_t dev_ctrl;
    usb3_role_manager_t     role_mgr;

    // role manager
    sc_signal<usb3_role_t> sig_role;
    sc_signal<bool>        sig_role_changed;
    sc_signal<usb_c_cc_state_t> sig_cc_state_int;
    sc_signal<bool>             sig_vbus_int;

    // host controller
    sc_signal<bool>    sig_host_enable;
    sc_signal<bool>    sig_xfer_done;
    sc_signal<bool>    sig_enum_done;
    sc_signal<uint8_t> sig_dev_addr;

    // device controller
    sc_signal<bool>    sig_dev_enable;
    sc_signal<bool>    sig_bulk_irq;
    sc_signal<bool>    sig_configured;

    // internal initiator to forward upstream packets into protocol.from_link
    simple_initiator_socket<usb3_drd_controller_t> m_to_protocol_link;


    SC_CTOR(usb3_drd_controller_t) :
        // external ports
        cc_state("cc_state"), vbus("vbus"),
        irq("irq"),
        cfg_socket("cfg_socket"), upstream("upstream"),
        // submodules
        protocol("protocol"),
        host_ctrl("host_ctrl"), dev_ctrl("dev_ctrl"), role_mgr("role_mgr"),
        // internal signals
        sig_role("sig_role"), sig_role_changed("sig_role_changed"),
        sig_cc_state_int("sig_cc_state_int"), sig_vbus_int("sig_vbus_int"),
        sig_host_enable("sig_host_enable"), sig_xfer_done("sig_xfer_done"),
        sig_enum_done("sig_enum_done"), sig_dev_addr("sig_dev_addr"),
        sig_dev_enable("sig_dev_enable"), sig_bulk_irq("sig_bulk_irq"),
        sig_configured("sig_configured"),
        m_to_protocol_link("m_to_protocol_link"),
        m_regs{}
    {
        // register cfg_socket callback
        cfg_socket.register_b_transport(this, &usb3_drd_controller_t::cfg_b_transport);
        upstream.register_b_transport(this, &usb3_drd_controller_t::upstream_b_transport);

        bind_internal();

        SC_THREAD(role_switch_monitor);
        sensitive << sig_role_changed.posedge_event();

        SC_METHOD(interrupt_collector);
        sensitive << sig_role_changed.value_changed_event()
                  << sig_enum_done.value_changed_event()
                  << sig_xfer_done.value_changed_event()
                  << sig_bulk_irq.posedge_event();
        dont_initialize();

        SC_METHOD(forward_inputs);
        sensitive << cc_state << vbus;
    }

private:
    uint32_t m_regs[8];   //< shadow of drd register space
    uint32_t m_intr_enable  = 0xFFFFFFFF;
    uint32_t m_intr_status  = 0;

    void bind_internal();

    void role_switch_monitor();
    void interrupt_collector();
    void forward_inputs();

    void cfg_b_transport(tlm_generic_payload& trans,
                         sc_time& delay);

    void upstream_b_transport(tlm_generic_payload& trans,
                              sc_time& delay);

    void raise_irq(uint32_t bits);
    uint32_t build_status_reg() const;
};

#endif // usb3_drd_controller_h
