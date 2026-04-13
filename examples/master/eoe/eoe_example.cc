//
// EoE (Ethernet over EtherCAT) master example
//
// Demonstrates:
//   1. Detecting EoE-capable slaves
//   2. Reading the current IP configuration from a slave (GET IP)
//   3. Writing a new IP configuration to a slave  (SET IP)
//   4. Installing a receive callback for frames sent by the slave
//   5. Sending raw Ethernet frames to the slave
//
// Usage:
//   sudo ./eoe_example -i eth0
//   sudo ./eoe_example -i tap:client    (simulation with slave_simulator tap:server)
//
// The example runs a short cyclic loop after configuration.  On each iteration
// it sends a minimal test frame to the first EoE-capable slave and prints any
// frames received from that slave.
//

#include <cstdio>
#include <cstring>
#include <iostream>
#include <atomic>
#include <vector>

#include <argparse/argparse.hpp>

#include "kickcat/Link.h"
#include "kickcat/Bus.h"
#include "kickcat/Prints.h"
#include "kickcat/helpers.h"
#include "kickcat/MailboxSequencer.h"
#include "kickcat/OS/Timer.h"

using namespace kickcat;

static void printIpParam(char const* prefix, EoE::IpParam const& p)
{
    if (p.mac_set)
    {
        printf("%s MAC     : %02x:%02x:%02x:%02x:%02x:%02x\n", prefix,
               p.mac[0], p.mac[1], p.mac[2], p.mac[3], p.mac[4], p.mac[5]);
    }
    if (p.ip_set)
    {
        printf("%s IP      : %d.%d.%d.%d\n", prefix,
               p.ip[0], p.ip[1], p.ip[2], p.ip[3]);
    }
    if (p.subnet_set)
    {
        printf("%s Subnet  : %d.%d.%d.%d\n", prefix,
               p.subnet[0], p.subnet[1], p.subnet[2], p.subnet[3]);
    }
    if (p.gateway_set)
    {
        printf("%s Gateway : %d.%d.%d.%d\n", prefix,
               p.gateway[0], p.gateway[1], p.gateway[2], p.gateway[3]);
    }
    if (p.dns_ip_set)
    {
        printf("%s DNS IP  : %d.%d.%d.%d\n", prefix,
               p.dns_ip[0], p.dns_ip[1], p.dns_ip[2], p.dns_ip[3]);
    }
    if (p.dns_name_set)
    {
        printf("%s DNS Name: %.32s\n", prefix, p.dns_name);
    }
}


int main(int argc, char* argv[])
{
    argparse::ArgumentParser program("eoe_example");

    std::string nom_interface_name;
    program.add_argument("-i", "--interface")
        .help("network interface name (or tap:client for simulation)")
        .required()
        .store_into(nom_interface_name);

    std::string red_interface_name;
    program.add_argument("-r", "--redundancy")
        .help("redundancy network interface name")
        .default_value(std::string{""})
        .store_into(red_interface_name);

    try
    {
        program.parse_args(argc, argv);
    }
    catch (std::runtime_error const& err)
    {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    std::shared_ptr<AbstractSocket> socket_nominal;
    std::shared_ptr<AbstractSocket> socket_redundancy;
    try
    {
        auto [nominal, redundancy] = createSockets(nom_interface_name, red_interface_name);
        socket_nominal    = nominal;
        socket_redundancy = redundancy;
    }
    catch (std::exception const& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }

    auto report_redundancy = []()
    {
        printf("Redundancy activated due to cable loss\n");
    };

    auto link = std::make_shared<Link>(socket_nominal, socket_redundancy, report_redundancy);
    link->setTimeout(2ms);
    link->checkRedundancyNeeded();

    Bus bus(link);

    uint8_t io_buffer[2048]{};

    Slave* eoe_slave = nullptr;

    try
    {
        printf("Initializing bus...\n");
        bus.init(100ms);
        printf("Detected %d slave(s)\n", bus.detectedSlaves());

        bus.createMapping(io_buffer);

        printf("Switching to SAFE_OP...\n");
        bus.requestState(State::SAFE_OP);
        bus.waitForState(State::SAFE_OP, 3s);

        // Find first EoE-capable slave
        for (auto& slave : bus.slaves())
        {
            if (slave.sii.supported_mailbox & eeprom::MailboxProtocol::EoE)
            {
                eoe_slave = &slave;
                printf("Found EoE slave at address %d (vendor 0x%08x product 0x%08x)\n",
                       slave.address, slave.sii.vendor_id, slave.sii.product_code);
                break;
            }
        }

        if (eoe_slave == nullptr)
        {
            printf("No EoE-capable slave detected on the bus.\n");
            printf("(This example requires a slave that advertises EoE in its SII EEPROM.)\n");
            return 0;
        }

        // GET IP: query current IP configuration from the slave
        printf("\n--- GET IP ---\n");
        try
        {
            EoE::IpParam current{};
            bus.getEoEIp(*eoe_slave, 0, current);
            if (not current.mac_set and not current.ip_set)
            {
                printf("Slave returned empty IP configuration.\n");
            }
            else
            {
                printIpParam("  Current", current);
            }
        }
        catch (std::exception const& e)
        {
            printf("GET IP failed: %s\n", e.what());
        }

        // SET IP: configure the slave with a static address
        printf("\n--- SET IP ---\n");
        try
        {
            EoE::IpParam config{};
            config.ip_set   = true;
            config.ip[0]    = 192;
            config.ip[1]    = 168;
            config.ip[2]    = 1;
            config.ip[3]     = 100;

            config.subnet_set = true; 
            config.subnet[0] = 255;
            config.subnet[1] = 255;
            config.subnet[2] = 255;
            config.subnet[3] = 0;
            
            config.gateway_set = true;
            config.gateway[0] = 192;
            config.gateway[1] = 168;
            config.gateway[2] = 1;
            config.gateway[3] = 1;
            
            config.mac_set = true;
            config.mac[0] = 0x02;
            config.mac[1] = 0x00;
            config.mac[2] = 0x00;
            config.mac[3] = 0x00;
            config.mac[4] = 0x00;
            config.mac[5] = 0x01;

            bus.setEoEIp(*eoe_slave, 0, config);
            printf("SET IP succeeded:\n");
            printIpParam("  New", config);
        }
        catch (std::exception const& e)
        {
            printf("SET IP failed: %s\n", e.what());
        }

        for (auto& slave : bus.slaves())
        {
            for (int32_t i = 0; i < slave.output.bsize; ++i)
            {
                slave.output.data[i] = 0;
            }
        }

        auto cyclic = [&]()
        {
            auto noop = [](DatagramState const&){};
            bus.processDataRead(noop);
            bus.processDataWrite(noop);
        };

        printf("Switching to OPERATIONAL...\n");
        bus.requestState(State::OPERATIONAL);
        bus.waitForState(State::OPERATIONAL, 3s, cyclic);
        printf("Bus is OPERATIONAL\n");
    }
    catch (std::exception const& e)
    {
        std::cerr << "Initialization error: " << e.what() << "\n";
        return 1;
    }

    std::atomic<uint32_t> received_frames{0};

    bus.enableEoEReceive(*eoe_slave,
        [&](uint8_t const* data, uint16_t size)
        {
            ++received_frames;
            printf("[EoE RX] %d-byte frame  (total received: %u)\n", size, received_frames.load());
            printf("         dst=%02x:%02x:%02x:%02x:%02x:%02x  "
                   "src=%02x:%02x:%02x:%02x:%02x:%02x  "
                   "ethertype=%02x%02x\n",
                   data[0], data[1], data[2], data[3], data[4], data[5],
                   data[6], data[7], data[8], data[9], data[10], data[11],
                   data[12], data[13]);
        });

    printf("\n--- Starting cyclic loop (Ctrl-C to stop) ---\n");

    // Minimal valid Ethernet frame: 60 bytes (14 header + 46 payload)
    // dst=broadcast, src=02:00:00:00:00:00, ethertype=0x0800 (IPv4)
    static uint8_t test_frame[60]{};
    // Destination: broadcast
    std::memset(test_frame + 0, 0xFF, 6);
    // Source: locally administered unicast
    test_frame[6] = 0x02;
    // EtherType: 0x0800 (IPv4) - just a placeholder
    test_frame[12] = 0x08;
    test_frame[13] = 0x00;

    auto callback_error = [](DatagramState const&)
    {
        THROW_ERROR("EtherCAT datagram error in cyclic loop");
    };

    MailboxSequencer mailbox_sequencer(bus);
    link->setTimeout(10ms);

    Timer timer{4ms};
    timer.start();

    uint32_t iteration   = 0;
    uint32_t send_period = 250;  // send a test frame every 250 iterations (~1 s at 4 ms tick)

    while (true)
    {
        timer.wait_next_tick();
        ++iteration;

        try
        {
            bus.sendLogicalRead(callback_error);
            bus.sendLogicalWrite(callback_error);
            mailbox_sequencer.step(callback_error);
            bus.finalizeDatagrams();
            bus.processAwaitingFrames();

            // Periodically inject a test Ethernet frame
            if ((iteration % send_period) == 0)
            {
                test_frame[14] = static_cast<uint8_t>(iteration & 0xFF);  // simple sequence id
                bus.queueEoEFrame(*eoe_slave, 0, test_frame, sizeof(test_frame));
                printf("[EoE TX] sent test frame #%u\n", iteration / send_period);
            }
        }
        catch (std::exception const& e)
        {
            printf("Loop error: %s\n", e.what());
        }
    }

    return 0;
}
