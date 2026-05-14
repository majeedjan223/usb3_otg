// ============================================================================
//
//  scenario 1 (0–25 µs):
//    * drd boots as host (cc_state=ufp_attached, vbus=1)
//    * role manager detects host role
//    * host enumerates peer device (get_descriptor, set_address, set_config)
//    * host writes 64-byte bulk pattern to device ep2
//    * device loopback copies it to ep1
//    * host reads 64 bytes from ep1, verifies data match
//
//  scenario 2 (25–50 µs):
//    * cc_state toggled to dfp_attached; role_ctrl=2 written via cfg_socket
//    * role manager switches drd to device role
//    * peer (now acting as host) could enumerate drd-as-device
// ============================================================================

#include "usb3_drd_controller.h"
#include <iomanip>
#include <cassert>
#include <cstring>

using namespace sc_core;
using namespace std;
//  simple peer usb device (placeholder for scenario 1)
//  accepts packets from drd host and responds with descriptor / data.
struct peer_device_t : sc_module {
    simple_target_socket<peer_device_t>    from_drd;
    simple_initiator_socket<peer_device_t> to_drd;

    SC_CTOR(peer_device_t)
      : from_drd("from_drd"), to_drd("to_drd"),
        m_assigned_addr(0), m_configured(false)
    {
        from_drd.register_b_transport(this, &peer_device_t::b_transport);
    }

    uint8_t  m_assigned_addr;
    bool     m_configured;

    // build minimal device descriptor
    static vector<uint8_t> make_dev_desc() {
        usb_device_descriptor_t d;
        vector<uint8_t> v(sizeof(d));
        memcpy(v.data(), &d, sizeof(d));
        return v;
    }

    // build minimal config descriptor blob
    static vector<uint8_t> make_cfg_desc() {
        static const uint8_t blob[] = {
            // configuration
            0x09, 0x02, 0x31, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,
            // interface
            0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x00, 0x00, 0x00,
            // ep1 bulk-in
            0x07, 0x05, 0x81, 0x02, 0x00, 0x04, 0x00,
            // ep1 ss companion
            0x06, 0x30, 0x00, 0x00, 0x00, 0x00,
            // ep2 bulk-out
            0x07, 0x05, 0x02, 0x02, 0x00, 0x04, 0x00,
            // ep2 ss companion
            0x06, 0x30, 0x00, 0x00, 0x00, 0x00
        };
        return vector<uint8_t>(blob, blob + sizeof(blob));
    }

    void b_transport(tlm_generic_payload& trans, sc_time& delay) {
        usb3_pkt_ext_t* ext = trans.get_extension<usb3_pkt_ext_t>();
        if (!ext) {
            trans.set_response_status(TLM_GENERIC_ERROR_RESPONSE);
            return;
        }

        switch (ext->pkt_type) {
            case usb3_pkt_type_t::SETUP:
                handle_setup(ext->setup, trans, delay);
                break;

            case usb3_pkt_type_t::CTRL_STATUS:
                trans.set_response_status(TLM_OK_RESPONSE);
                break;

            case usb3_pkt_type_t::BULK_OUT: {
                // loopback: store data and immediately send bulk_in response
                size_t len = trans.get_data_length();
                m_loopback_buf.assign(trans.get_data_ptr(),
                                      trans.get_data_ptr() + len);
                cout << "[peer] caught " << len << " bytes on bulk OUT\n";
                trans.set_response_status(TLM_OK_RESPONSE);
                break;
            }

            case usb3_pkt_type_t::BULK_IN: {
                // respond with loopback data
                if (!m_loopback_buf.empty()) {
                    send_bulk_in(m_loopback_buf, ext->endpoint, delay);
                    m_loopback_buf.clear();
                }
                trans.set_response_status(TLM_OK_RESPONSE);
                break;
            }

            default:
                trans.set_response_status(TLM_OK_RESPONSE);
                break;
        }
    }

private:
    vector<uint8_t> m_loopback_buf;

    void handle_setup(const usb3_setup_pkt_t& setup,
                      tlm_generic_payload& trans,
                      sc_time& delay) {
        vector<uint8_t> resp;

        switch (setup.bRequest) {
            case USB_REQ_GET_DESCRIPTOR: {
                uint8_t dtype = (setup.wValue >> 8) & 0xFF;
                if (dtype == USB_DESC_DEVICE)        resp = make_dev_desc();
                else if (dtype == USB_DESC_CONFIGURATION) resp = make_cfg_desc();
                else if (dtype == USB_DESC_BOS) {
                    static const uint8_t bos[] = {
                        0x05, 0x0F, 0x16, 0x00, 0x01,
                        0x0A, 0x10, 0x03, 0x00, 0x0E, 0x00, 0x01, 0x0A, 0xFF, 0x07
                    };
                    resp.assign(bos, bos + sizeof(bos));
                }
                if (resp.size() > setup.wLength) resp.resize(setup.wLength);
                send_ctrl_data(resp, delay);
                break;
            }
            case USB_REQ_SET_ADDRESS:
                m_assigned_addr = setup.wValue & 0x7F;
                cout << "[peer] new device address is " << (int)m_assigned_addr << "\n";
                break;
            case USB_REQ_SET_CONFIGURATION:
                m_configured = true;
                cout << "[peer] configuration applied\n";
                break;
            default:
                break;
        }
        trans.set_response_status(TLM_OK_RESPONSE);
    }

    void send_ctrl_data(vector<uint8_t>& data, sc_time& delay) {
        (void)delay;
        if (data.empty()) return;
        tlm_generic_payload resp;
        usb3_pkt_ext_t ext;
        ext.pkt_type  = usb3_pkt_type_t::CTRL_DATA;
        ext.dev_addr  = m_assigned_addr;
        ext.endpoint  = 0;
        ext.direction = usb3_dir_t::DEVICE_TO_HOST;
        resp.set_data_ptr(data.data());
        resp.set_data_length(data.size());
        resp.set_command(TLM_WRITE_COMMAND);
        resp.set_address(0);
        resp.set_extension(&ext);
        sc_time d = SC_ZERO_TIME;
        to_drd->b_transport(resp, d);
        resp.clear_extension(&ext);
    }

    void send_bulk_in(vector<uint8_t>& data, uint8_t ep,
                      sc_time& delay) {
        (void)delay;
        tlm_generic_payload resp;
        usb3_pkt_ext_t ext;
        ext.pkt_type  = usb3_pkt_type_t::BULK_IN;
        ext.dev_addr  = m_assigned_addr;
        ext.endpoint  = ep;
        ext.direction = usb3_dir_t::DEVICE_TO_HOST;
        ext.ep_type   = usb3_ep_type_t::BULK;
        resp.set_data_ptr(data.data());
        resp.set_data_length(data.size());
        resp.set_command(TLM_WRITE_COMMAND);
        resp.set_address(0);
        resp.set_extension(&ext);
        sc_time d = SC_ZERO_TIME;
        to_drd->b_transport(resp, d);
        resp.clear_extension(&ext);
        cout << "[peer] answered bulk IN with " << data.size() << " bytes\n";
    }
};

//testbench top module
SC_MODULE(tb_top_t) {

    usb3_drd_controller_t dut;

    peer_device_t peer;

    simple_initiator_socket<tb_top_t> cfg_init;

    sc_signal<usb_c_cc_state_t> sig_cc_state;
    sc_signal<bool>  sig_vbus;
    sc_signal<bool, SC_MANY_WRITERS>  sig_irq;

    SC_CTOR(tb_top_t) :
        dut("dut"), peer("peer"),
        cfg_init("cfg_init"),
        sig_cc_state("sig_cc_state"), sig_vbus("sig_vbus"),
        sig_irq("sig_irq")
    {
        dut.cc_state(sig_cc_state);
        dut.vbus(sig_vbus);
        dut.irq(sig_irq);

        // dut downstream <----> peer upstream
        dut.protocol.to_link(peer.from_drd);
        peer.to_drd(dut.upstream);

        // testbench initiator ------> dut cfg_socket
        cfg_init(dut.cfg_socket);

        SC_THREAD(run_scenarios);
    }

    //testbench thread
    void run_scenarios() {
        // test 1: drd as host
        
        cout << "\n--- Test 1: DRD as host ---\n\n";

        sig_cc_state.write(usb_c_cc_state_t::CC_UFP_ATTACHED);  // ufp (device) attached → host mode
        sig_vbus.write(false);
        wait(sc_time(100, SC_NS));

        wait(sc_time(500, SC_NS));
        sig_vbus.write(true);

        cout << "[tb] CC says UFP + VBUS high (so we should be host)\n";

        // wait for role detection and enumeration to settle (~10 ms sim time)
        wait(sc_time(15.0, SC_MS));

        cout << "\n[tb] bulk loopback check\n";

        // build 64-byte test pattern
        vector<uint8_t> tx_data(64);
        for (size_t i = 0; i < 64; ++i) tx_data[i] = static_cast<uint8_t>(i);

        // queue bulk-out to peer device ep2
        dut.host_ctrl.bulk_out(dut.host_ctrl.device_info().addr, 2, tx_data);
        wait(sc_time(200, SC_US));

        // queue bulk-in from peer device ep1
        dut.host_ctrl.bulk_in(dut.host_ctrl.device_info().addr, 1, 64);
        wait(sc_time(200, SC_US));

        // verify loopback
        const vector<uint8_t>& rx = dut.host_ctrl.rx_buffer();
        bool loopback_ok = (rx.size() == 64);
        for (size_t i = 0; i < rx.size() && loopback_ok; ++i) {
            if (rx[i] != tx_data[i]) { loopback_ok = false; }
        }

        cout << "[tb] loopback " << (loopback_ok ? "passed" : "failed")
                  << " (" << rx.size() << " bytes back)\n";

        cout << "\n--- Test 2: flip DRD to device role ---\n\n";

        // toggle cc_state to dfp_attached
        sig_cc_state.write(usb_c_cc_state_t::CC_DFP_ATTACHED);
        cout << "[tb] CC is DFP now; asking the block to go device\n";
        wait(sc_time(1.0, SC_US));

        // also force device role via role_ctrl register (exercises cfg_socket write path)
        {
            uint32_t val = 2;
            uint8_t val_bytes[4];
            memcpy(val_bytes, &val, sizeof(val));
            tlm_generic_payload trans;
            trans.set_command(TLM_WRITE_COMMAND);
            trans.set_address(usb3_drd_regs::ROLE_CTRL);
            trans.set_data_ptr(val_bytes);
            trans.set_data_length(4);
            sc_time d = SC_ZERO_TIME;
            cfg_init->b_transport(trans, d);
        }

        wait(sc_time(5.0, SC_US));

        // read status register
        {
            uint32_t status = 0;
            uint8_t status_bytes[4];
            tlm_generic_payload trans;
            trans.set_command(TLM_READ_COMMAND);
            trans.set_address(usb3_drd_regs::STATUS);
            trans.set_data_ptr(status_bytes);
            trans.set_data_length(4);
            sc_time d = SC_ZERO_TIME;
            cfg_init->b_transport(trans, d);
            memcpy(&status, status_bytes, sizeof(status));

            uint8_t role = (status >> 4) & 0x03;
            cout << "[tb] STATUS readback 0x" << hex << status << dec
                      << " (role field says "
                      << (role == 2 ? "device" : role == 1 ? "host" : "none")
                      << ")\n";
        }

        // wait for role switch to settle
        wait(sc_time(15.0, SC_MS));

        // clear interrupts
        {
            uint32_t all_ones = 0xFFFFFFFF;
            uint8_t wbytes[4];
            memcpy(wbytes, &all_ones, sizeof(all_ones));
            tlm_generic_payload trans;
            trans.set_command(TLM_WRITE_COMMAND);
            trans.set_address(usb3_drd_regs::INTR_STATUS);
            trans.set_data_ptr(wbytes);
            trans.set_data_length(4);
            sc_time d = SC_ZERO_TIME;
            cfg_init->b_transport(trans, d);
        }

        cout << "\n[tb] cleared interrupt status\n";

        cout << "\nDone. Stopping at " << sc_time_stamp().to_string() << "\n";
        
        sc_stop();
    }
};

int sc_main(int argc, char* argv[]) {
    (void)argc; (void)argv;

    tb_top_t tb("tb"); //testbench

    sc_start();
    return 0;
}
