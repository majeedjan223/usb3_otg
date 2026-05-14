#include "usb3_drd_controller.h"
#include <cstring>
#include <sstream>


using namespace std;
using namespace sc_core;
using namespace tlm;
using namespace tlm_utils;

void usb3_drd_controller_t::bind_internal() {

  role_mgr.cc_state(sig_cc_state_int);
  role_mgr.vbus(sig_vbus_int);
  role_mgr.current_role(sig_role);
  role_mgr.role_changed(sig_role_changed);

  protocol.current_role(sig_role);

  // protocol ↔ host controller
  host_ctrl.to_protocol(protocol.from_host);
  protocol.to_host(host_ctrl.from_protocol);

  host_ctrl.enable(sig_host_enable);
  host_ctrl.transfer_done(sig_xfer_done);
  host_ctrl.enumeration_done(sig_enum_done);
  host_ctrl.dev_addr_out(sig_dev_addr);

  dev_ctrl.enable(sig_dev_enable);
  dev_ctrl.dev_addr_in(sig_dev_addr);
  dev_ctrl.bulk_irq(sig_bulk_irq);
  dev_ctrl.configured(sig_configured);

  // device ↔ protocol sockets
  dev_ctrl.to_protocol(protocol.from_device);
  protocol.to_device(dev_ctrl.from_protocol);

  // internal initiator → protocol.from_link (for upstream packet forwarding)
  m_to_protocol_link(protocol.from_link);
}

void usb3_drd_controller_t::role_switch_monitor() {
  sig_host_enable.write(false);
  sig_dev_enable.write(false);

  while (true) {
    wait(); // posedge of sig_role_changed

    usb3_role_t role = sig_role.read();

    ostringstream oss;
    oss << "Role is now " << to_cstr(role);
    USB3_LOG(oss.str());

    // disable both controllers first
    sig_host_enable.write(false);
    sig_dev_enable.write(false);
    wait(sc_time(1, SC_US));
    // enable the appropriate controller
    if (role == usb3_role_t::HOST) {
      sig_host_enable.write(true);
    } else if (role == usb3_role_t::DEVICE) {
      sig_dev_enable.write(true);
    }
    raise_irq(DRD_INTR_ROLE_CHANGED);
  }
}

void usb3_drd_controller_t::interrupt_collector() {
  if (sig_enum_done.read()) {
    raise_irq(DRD_INTR_ENUM_DONE);

    // update dev_addr register
    m_regs[usb3_drd_regs::DEV_ADDR / 4] = sig_dev_addr.read();
  }
  if (sig_xfer_done.read()) {
    raise_irq(DRD_INTR_XFER_DONE);
  }
  if (sig_bulk_irq.read()) {
    raise_irq(DRD_INTR_BULK_IRQ);
  }
}

void usb3_drd_controller_t::raise_irq(uint32_t bits) {
  m_intr_status |= bits;
  if (m_intr_status & m_intr_enable) {
    irq.write(true);
    // update status register
    m_regs[usb3_drd_regs::INTR_STATUS / 4] = m_intr_status;

    ostringstream oss;
    oss << "IRQ asserted, INTR_STATUS=0x" << hex << m_intr_status;
    USB3_LOG(oss.str());
  }
}

uint32_t usb3_drd_controller_t::build_status_reg() const {
  uint32_t s = 0;
  s |= (static_cast<uint8_t>(sig_role.read()) & 0x03) << 4;
  s |= (static_cast<uint8_t>(usb3_speed_t::SUPER_SPEED) & 0x07) << 8;
  s |= (1u) << 16;
  s |= (sig_configured.read() ? 1u : 0u) << 17;
  return s;
}

void usb3_drd_controller_t::cfg_b_transport(tlm_generic_payload &trans,
                                            sc_time &delay) {
  uint64_t addr = trans.get_address() & ~3u; // align to 4 bytes
  uint8_t *data = trans.get_data_ptr();
  uint32_t dlen = trans.get_data_length();

  if (dlen < 4) {
    trans.set_response_status(TLM_GENERIC_ERROR_RESPONSE);
    return;
  }

  if (trans.get_command() == TLM_READ_COMMAND) {
    uint32_t val = 0;
    switch (addr) {
    case usb3_drd_regs::STATUS:
      val = build_status_reg();
      break;
    case usb3_drd_regs::INTR_STATUS:
      val = m_intr_status;
      break;
    case usb3_drd_regs::INTR_ENABLE:
      val = m_intr_enable;
      break;
    case usb3_drd_regs::DEV_ADDR:
      val = sig_dev_addr.read();
      break;
    case usb3_drd_regs::SPEED:
      val = static_cast<uint32_t>(usb3_speed_t::SUPER_SPEED);
      break;
    default:
      val = (addr / 4 < 8) ? m_regs[addr / 4] : 0x00;
      break;
    }
    memcpy(data, &val, 4);
    trans.set_response_status(TLM_OK_RESPONSE);

  } else { // write
    uint32_t val = 0;
    memcpy(&val, data, 4);

    switch (addr) {
    case usb3_drd_regs::CTRL: {
      ostringstream o;
      o << hex << val;
      USB3_LOG(string("CTRL MMIO write, value 0x") + o.str());
      // bit 0 = soft reset (not modeled in detail)
      break;
    }
    case usb3_drd_regs::ROLE_CTRL:
      if (val == 1)
        role_mgr.request_role(usb3_role_t::HOST);
      else if (val == 2)
        role_mgr.request_role(usb3_role_t::DEVICE);
      break;
    case usb3_drd_regs::INTR_STATUS:
      // w1c
      m_intr_status &= ~val;
      if (!m_intr_status)
        irq.write(false);
      break;
    case usb3_drd_regs::INTR_ENABLE:
      m_intr_enable = val;
      break;
    default:
      if (addr / 4 < 8)
        m_regs[addr / 4] = val;
      break;
    }
    trans.set_response_status(TLM_OK_RESPONSE);
  }

  delay += sc_time(10, SC_NS);
}

void usb3_drd_controller_t::upstream_b_transport(
    tlm_generic_payload &trans, sc_time &delay) {
  // route upstream packet directly to protocol layer
  m_to_protocol_link->b_transport(trans, delay);
}

void usb3_drd_controller_t::forward_inputs() {
  sig_cc_state_int.write(cc_state.read());
  sig_vbus_int.write(vbus.read());
}
