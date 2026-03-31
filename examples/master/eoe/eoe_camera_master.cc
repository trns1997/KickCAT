//
// EoE Camera Master
//
// Receives JPEG frame chunks from an EoE camera slave (RPi / LAN9252),
// reassembles them, and displays the video stream at ≈30 fps using OpenCV.
//
// Usage:
//   sudo ./eoe_camera_master -i eth0
//
// The slave streams chunks over EoE mailbox messages.  Each chunk carries a
// 12-byte ChunkHeader followed by up to 980 bytes of JPEG payload, sized so
// that it fits in exactly one EoE mailbox exchange for a 1024-byte mailbox SM.
//
// The master polls the mailbox every loop iteration (no MailboxSequencer) to
// maximise throughput.  At ≈1 ms round-trip the link can sustain roughly
// 900 KB/s, which comfortably covers a 640×480 JPEG-60 stream at 30 fps
// (~8–15 KB per frame, ~240–450 KB/s).
//

#include <opencv2/opencv.hpp>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>

#include <argparse/argparse.hpp>

#include "kickcat/Bus.h"
#include "kickcat/Link.h"
#include "kickcat/OS/Timer.h"
#include "kickcat/Prints.h"
#include "kickcat/helpers.h"

using namespace kickcat;

// ---------------------------------------------------------------------------
// Application-level chunk protocol  (must match main_eoe_camera.cc)
// ---------------------------------------------------------------------------
struct ChunkHeader
{
    uint32_t frame_id;
    uint16_t chunk_idx;
    uint16_t total_chunks;
    uint32_t payload_len;
} __attribute__((packed));

// ---------------------------------------------------------------------------
// Frame reassembler
// ---------------------------------------------------------------------------
struct FrameAssembler
{
    uint32_t frame_id;
    uint16_t total_chunks;
    uint16_t chunks_received{0};
    std::map<uint16_t, std::vector<uint8_t>> chunks;

    FrameAssembler(uint32_t id, uint16_t total)
        : frame_id(id), total_chunks(total) {}

    bool isComplete() const { return chunks_received == total_chunks; }

    std::vector<uint8_t> reassemble() const
    {
        std::vector<uint8_t> result;
        for (uint16_t i = 0; i < total_chunks; ++i)
        {
            auto const& c = chunks.at(i);
            result.insert(result.end(), c.begin(), c.end());
        }
        return result;
    }
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    argparse::ArgumentParser program("eoe_camera_master");

    std::string nom_iface;
    program.add_argument("-i", "--interface")
        .help("EtherCAT network interface (e.g. eth0)")
        .required()
        .store_into(nom_iface);

    std::string red_iface;
    program.add_argument("-r", "--redundancy")
        .help("redundancy interface (optional)")
        .default_value(std::string{""})
        .store_into(red_iface);

    try { program.parse_args(argc, argv); }
    catch (std::runtime_error const& e)
    {
        std::cerr << e.what() << "\n" << program;
        return 1;
    }

    // ------------------------------------------------------------------
    // Bus setup
    // ------------------------------------------------------------------
    std::shared_ptr<AbstractSocket> sock_nom, sock_red;
    try
    {
        auto [n, r] = createSockets(nom_iface, red_iface);
        sock_nom = n;
        sock_red = r;
    }
    catch (std::exception const& e) { std::cerr << e.what() << "\n"; return 1; }

    auto report_red = []() { printf("Redundancy activated (cable loss detected)\n"); };
    auto link = std::make_shared<Link>(sock_nom, sock_red, report_red);
    link->setTimeout(2ms);
    link->checkRedundancyNeeded();

    Bus bus(link);
    uint8_t io_buf[128]{};

    Slave* eoe_slave = nullptr;

    // ------------------------------------------------------------------
    // Initialise: INIT → PRE-OP → SAFE-OP → OP
    // ------------------------------------------------------------------
    try
    {
        printf("Initializing bus...\n");
        bus.init(100ms);
        printf("Detected %d slave(s)\n", bus.detectedSlaves());

        bus.createMapping(io_buf);

        printf("Switching to SAFE_OP...\n");
        bus.requestState(State::SAFE_OP);
        bus.waitForState(State::SAFE_OP, 3s);

        // Find the camera slave — prefer one that advertises EoE in SII,
        // fall back to first available slave for quick PoC bring-up.
        for (auto& s : bus.slaves())
        {
            if (s.sii.supported_mailbox & eeprom::MailboxProtocol::EoE)
            {
                eoe_slave = &s;
                printf("Found EoE slave at address %d "
                       "(vendor=0x%08x product=0x%08x)\n",
                       s.address, s.sii.vendor_id, s.sii.product_code);
                break;
            }
        }
        if (eoe_slave == nullptr && !bus.slaves().empty())
        {
            eoe_slave = &bus.slaves().front();
            printf("No EoE-flagged slave; using slave at address %d\n",
                   eoe_slave->address);
        }
        if (eoe_slave == nullptr)
        {
            printf("No slaves detected.\n");
            return 1;
        }

        // Zero slave outputs so it can leave SAFE_OP
        for (auto& s : bus.slaves())
        {
            for (int32_t i = 0; i < s.output.bsize; ++i)
                s.output.data[i] = 0;
        }

        // Cyclic PDO helper used by waitForState
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

    // ------------------------------------------------------------------
    // EoE receive callback — reassembles application-level JPEG chunks
    // ------------------------------------------------------------------
    std::map<uint32_t, FrameAssembler> assemblers;
    uint32_t last_displayed_frame = 0;

    // Latest complete JPEG ready for display
    std::vector<uint8_t> pending_jpeg;
    bool                 pending_ready = false;

    bus.enableEoEReceive(*eoe_slave,
        [&](uint8_t const* data, uint16_t size)
        {
            if (size < static_cast<uint16_t>(sizeof(ChunkHeader)))
            {
                return; // too short to contain a valid header
            }

            auto const* hdr   = reinterpret_cast<ChunkHeader const*>(data);
            uint32_t    fid   = hdr->frame_id;
            uint16_t    cidx  = hdr->chunk_idx;
            uint16_t    total = hdr->total_chunks;
            uint32_t    plen  = hdr->payload_len;

            // Discard frames older than what we already displayed
            if (fid < last_displayed_frame)
            {
                return;
            }

            if (assemblers.find(fid) == assemblers.end())
            {
                assemblers.emplace(fid, FrameAssembler(fid, total));
            }

            auto& asmblr = assemblers.at(fid);

            if (asmblr.chunks.find(cidx) == asmblr.chunks.end())
            {
                // Clamp payload to what was actually received
                uint32_t payload_offset = static_cast<uint32_t>(sizeof(ChunkHeader));
                if (plen > static_cast<uint32_t>(size) - payload_offset)
                {
                    plen = static_cast<uint32_t>(size) - payload_offset;
                }

                asmblr.chunks[cidx] = std::vector<uint8_t>(
                    data + payload_offset,
                    data + payload_offset + plen);
                asmblr.chunks_received++;

                if (asmblr.isComplete())
                {
                    pending_jpeg  = asmblr.reassemble();
                    pending_ready = true;
                    last_displayed_frame = fid;

                    // Clean up completed and stale assemblers
                    for (auto it = assemblers.begin(); it != assemblers.end(); )
                    {
                        if (it->first <= fid)
                            it = assemblers.erase(it);
                        else
                            ++it;
                    }
                }
            }
        });

    // ------------------------------------------------------------------
    // Cyclic loop: fast mailbox polling + display
    // ------------------------------------------------------------------
    // We intentionally avoid MailboxSequencer here to maximise read
    // throughput: every iteration we check if the slave has data and
    // immediately read it.  checkMailboxes bundles the SM-status read
    // with the PDO LRD/LWR so there is no extra round-trip overhead.

    link->setTimeout(2ms);

    auto err_cb = [](DatagramState const& s)
    {
        THROW_ERROR_DATAGRAM("EtherCAT datagram error", s);
    };

    cv::Mat        display_frame;
    bool           window_opened = false;
    uint32_t       frames_shown  = 0;

    printf("Starting EoE camera receive loop. Press 'q' to quit.\n");

    while (true)
    {
        try
        {
            // PDO round-trip (keeps slave SM2 watchdog alive)
            bus.sendLogicalRead(err_cb);
            bus.sendLogicalWrite(err_cb);

            // Mailbox: read slave SM status → if data ready, fetch it
            // processMessages dispatches into the EoE receive callback above.
            bus.checkMailboxes(err_cb);
            bus.processMessages(err_cb);
        }
        catch (std::exception const& e)
        {
            printf("Loop error: %s\n", e.what());
        }

        // Display if a new complete JPEG arrived during this iteration
        if (pending_ready)
        {
            cv::Mat img = cv::imdecode(pending_jpeg, cv::IMREAD_COLOR);
            if (!img.empty())
            {
                display_frame = img;
                window_opened = true;
                ++frames_shown;
                if ((frames_shown % 30) == 0)
                {
                    printf("[Display] Showed %u frames  (latest frame_id=%u)\n",
                           frames_shown, last_displayed_frame);
                }
            }
            pending_ready = false;
        }

        if (window_opened)
        {
            cv::imshow("EoE Camera Stream", display_frame);
        }

        // cv::waitKey(1) pumps the OpenCV event loop (~1 ms delay).
        // 'q' or ESC quits.
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27)
        {
            break;
        }
    }

    cv::destroyAllWindows();
    printf("Done. Total frames displayed: %u\n", frames_shown);
    return 0;
}
