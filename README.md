## Scope of the model

The device under test is a **Dual Role Device (DRD)**. It may operate as a **USB host** or a **USB device**, depending on simplified **Type-C CC** and **VBUS** inputs and on a software **role** register when the role is forced by software.

The DRD comprises a **host controller**, a **device controller**, a **protocol layer** that routes traffic and a **role manager** that selects the active role. The top-level module integrates these blocks.

The testbench in `sc_main.cpp` instantiates a **peer** that responds sufficiently for enumeration and a bulk loopback test.

**Out of scope:** physical-layer modelling, link training, LTSSM and most analogue behaviour. Omitting these keeps the model compact and leaves control transfers, routing and role switching as the primary focus.

## Testbench and DUT connectivity

The testbench (`tb_top_t` in `sc_main.cpp`) surrounds the DUT and drives stimulus:

```text
+----------------------------- tb_top_t ------------------------------+
|                                                                      |
|  run_scenarios (SC_THREAD)                                           |
|       |                                                              |
|       +--- drives signals: cc_state, vbus ---> DUT ports            |
|                                                                      |
|  +----------------+                    +------------------+          |
|  | cfg_init       |                    | peer_device_t     |          |
|  | (initiator)    |                    | from_drd (tgt)   |          |
|  +--------+-------+                    | to_drd (init)    |          |
|           |                            +---------+--------+          |
+-----------|--------------------------------------|------------------+
            |                                      |
            v                                      v
     dut.cfg_socket                         protocol.to_link
     (register access)                     dut.upstream
```

Inside the DRD, the protocol layer sits between the host and device controllers:

```text
+------------------------ usb3_drd_controller_t ------------------------+
|                                                                       |
|   role_mgr          host_ctrl <-----> protocol <-----> dev_ctrl       |
|   (CC / VBUS)              |              |                           |
|                            |         to_link -----> (to peer)         |
|                            |         from_link <--- (from peer)       |
|                            |         from_host / to_host              |
|                            |         from_device / to_device          |
|                                                                       |
|   cfg_socket (target)              upstream (target)                  |
+-----------------------------------------------------------------------+
```


## TLM usage

A transfer is represented primarily by a `tlm_generic_payload` (buffer pointer, length, read versus write and optionally an address). USB-specific fields—packet type, endpoint, device address, setup packet—are carried in a **TLM user extension**, `usb3_pkt_ext_t` in `include/usb3_types.h`. The usual sequence is to attach the extension, invoke `b_transport` on a socket, then detach the extension when the transaction completes.

`b_transport` is the blocking transport style: a single call models one transfer and the `sc_time& delay` argument advances simulation time for loosely timed behaviour.

In the protocol module, **target** sockets receive incoming `b_transport` calls; **initiator** sockets are used to forward transactions downstream.

## Recommended reading order

1. `include/usb3_types.h` — shared types, packet kinds and the extension structure. Other modules depend on these definitions.
2. `include/usb3_drd_controller.h` and `src/usb3_drd_controller.cpp` — ports, register map, `bind_internal()` (internal binding), MMIO handler, interrupts and the role-switch thread.
3. `include/usb3_role_manager.h` and `src/usb3_role_manager.cpp` — selection of host versus device from CC/VBUS and software control and assertion of role-changed events.
4. `include/usb3_protocol_layer.h` and `src/usb3_protocol_layer.cpp` — transaction sequencing, routing to host or device and forwarding on the peer-facing path.
5. Host: `include/usb3_host_controller.h` and `src/usb3_host_controller.cpp` — enumeration and a minimal transfer ring.
6. Device: `include/usb3_device_controller.h` and `src/usb3_device_controller.cpp` — standard requests on endpoint 0 and the bulk loopback path.
7. `src/sc_main.cpp` — `peer_device_t` and `tb_top_t::run_scenarios()`; scenario definitions and diagnostic output.


## Simulation scenarios

**Scenario 1 — DRD as host**

The testbench drives CC so the role manager selects **host**, asserts VBUS with defined timing, then waits for enumeration to complete (including an intentional **15 ms** delay). It then issues a **64-byte** bulk OUT transfer to endpoint **2**, performs bulk IN transfers from endpoint **1** and verifies loopback data via the peer.

**Scenario 2 — transition to device**

CC is driven to a pattern that selects the device role; the testbench writes **`ROLE_CTRL = 2`** through `cfg_socket` to force **device**, reads **`STATUS`** to observe the role field, waits, then clears **`INTR_STATUS`** using a write-1-to-clear update.

The block comment at the top of `sc_main.cpp` summarizes the timing phases; the implemented delays are the `wait(sc_time(...))` calls in `run_scenarios()`.

## Registers and interrupts

Offsets correspond to `usb3_drd_regs` in `usb3_drd_controller.h`.

| Offset | Name | Purpose |
|--------|------|---------|
| `0x00` | `CTRL` | Control and soft reset (abbreviated model) |
| `0x04` | `STATUS` | Role, speed and status flags (derived when read) |
| `0x08` | `ROLE_CTRL` | Write `1` for host, `2` for device |
| `0x0C` | `INTR_STATUS` | Pending interrupt bits; **write 1 to clear** |
| `0x10` | `INTR_ENABLE` | Interrupt mask |
| `0x14` | `DEV_ADDR` | Device address assigned by the host |
| `0x1C` | `SPEED` | Informational speed field |

## Directory layout

```
├── CMakeLists.txt
├── README.md
├── include/
│   ├── usb3_types.h
│   ├── usb3_role_manager.h
│   ├── usb3_protocol_layer.h
│   ├── usb3_host_controller.h
│   ├── usb3_device_controller.h
│   └── usb3_drd_controller.h
└── src/
    ├── usb3_*.cpp
    └── sc_main.cpp
```

## Build and run

SystemC is resolved via pkg-config when available (for example after installing `libsystemc-dev` on Debian or Ubuntu). Otherwise set the `SYSTEMC_HOME` environment variable to the SystemC installation root before configuring.

```bash
export SYSTEMC_HOME="/path/to/systemc"   # optional; omit if pkg-config finds SystemC
cmake -S . -B build
cmake --build build
./build/usb3_drd_sim
```
