//
// EoE Camera Master
//
// Receives JPEG frame chunks from an EoE camera slave (RPi / LAN9252),
// reassembles them, and displays the video stream at ≈30 fps.
//
// Usage:
//   sudo ./eoe_camera_master -i eth0
//
// Mailbox optimisation
// --------------------
// configureMailboxStatusCheck(READ_CHECK) is called before createMapping().
// This maps the slave's SM1 "mailbox full" bit into a spare bit of the LRD
// logical frame, so can_read is updated as a zero-cost side-effect of the PDO
// exchange — no extra FPRD for the status poll.
//
// The cyclic loop therefore needs only:
//   1. sendLogicalRead + sendLogicalWrite + finalizeDatagrams + processAwaitingFrames
//      → PDO exchange + can_read update in a single round-trip
//   2. sendReadMessages + finalizeDatagrams + processAwaitingFrames
//      → FPRD to fetch the mailbox chunk (only if can_read == true)
//
// Lag control
// -----------
// The master always chases the latest frame_id.  Any assembler more than
// STALE_FRAME_WINDOW frames behind the newest seen frame_id is discarded
// immediately so partial reassemblies from slow frames never block the display.
// When a new complete JPEG arrives it always overwrites the pending display
// buffer (newest wins), so cv::imshow is always fed the freshest frame.
//
// Display is refreshed at ≈30 fps via a timestamp-gated cv::waitKey(1) call;
// this removes the per-iteration 1ms waitKey penalty and lets the EtherCAT
// loop spin as fast as the network allows.
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
#include "kickcat/OS/Time.h"
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

    FrameAssembler(uint32_t id, uint16_t total) : frame_id(id), total_chunks(total) {}

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
        .default_value(std::string{""})
        .store_into(red_iface);

    try { program.parse_args(argc, argv); }
    catch (std::runtime_error const& e) { std::cerr << e.what() << "\n" << program; return 1; }

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

    auto link = std::make_shared<Link>(sock_nom, sock_red,
                    []() { printf("Redundancy activated\n"); });
    link->setTimeout(2ms);
    link->checkRedundancyNeeded();

    Bus bus(link);
    uint8_t io_buf[128]{};
    Slave* eoe_slave = nullptr;

    // ------------------------------------------------------------------
    // INIT → PRE-OP → SAFE-OP → OP
    // ------------------------------------------------------------------
    try
    {
        printf("Initializing bus...\n");
        bus.init(100ms);
        printf("Detected %d slave(s)\n", bus.detectedSlaves());

        // Must be called after init() (PRE-OP) and BEFORE createMapping().
        // Maps slave SM1 "mailbox full" bit into every LRD logical frame so
        // sendLogicalRead updates can_read without an extra FPRD round-trip.
        bus.configureMailboxStatusCheck(MailboxStatusFMMU::READ_CHECK);

        bus.createMapping(io_buf);

        printf("Switching to SAFE_OP...\n");
        bus.requestState(State::SAFE_OP);
        bus.waitForState(State::SAFE_OP, 3s);

        for (auto& s : bus.slaves())
        {
            if (s.sii.supported_mailbox & eeprom::MailboxProtocol::EoE)
            {
                eoe_slave = &s;
                printf("Found EoE slave at address %d (vendor=0x%08x product=0x%08x)\n",
                       s.address, s.sii.vendor_id, s.sii.product_code);
                break;
            }
        }
        if (eoe_slave == nullptr && !bus.slaves().empty())
        {
            eoe_slave = &bus.slaves().front();
            printf("No EoE-flagged slave; using slave at address %d\n", eoe_slave->address);
        }
        if (eoe_slave == nullptr)
        {
            printf("No slaves detected.\n");
            return 1;
        }

        // Zero slave outputs so the slave can leave SAFE_OP
        for (auto& s : bus.slaves())
        {
            for (int32_t i = 0; i < s.output.bsize; ++i)
                s.output.data[i] = 0;
        }

        auto cyclic_pdo = [&]()
        {
            auto noop = [](DatagramState const&){};
            bus.processDataRead(noop);
            bus.processDataWrite(noop);
        };

        printf("Switching to OPERATIONAL...\n");
        bus.requestState(State::OPERATIONAL);
        bus.waitForState(State::OPERATIONAL, 3s, cyclic_pdo);
        printf("Bus is OPERATIONAL\n");
    }
    catch (std::exception const& e)
    {
        std::cerr << "Initialization error: " << e.what() << "\n";
        return 1;
    }

    // ------------------------------------------------------------------
    // EoE receive callback — reassembles application-level JPEG chunks
    // with stale-frame skipping for lag-free display
    // ------------------------------------------------------------------
    std::map<uint32_t, FrameAssembler> assemblers;

    // Number of frames a partial assembly may fall behind before we discard it.
    // At 30 fps this equals ~100 ms of tolerance before a frame is abandoned.
    static constexpr uint32_t STALE_FRAME_WINDOW = 3;

    uint32_t max_seen_fid   = 0;
    uint32_t last_shown_fid = 0;

    std::vector<uint8_t> pending_jpeg;
    bool                 pending_ready    = false;
    uint32_t             pending_frame_id = 0;

    bus.enableEoEReceive(*eoe_slave,
        [&](uint8_t const* data, uint16_t size)
        {
            if (size < static_cast<uint16_t>(sizeof(ChunkHeader)))
                return;

            auto const* hdr   = reinterpret_cast<ChunkHeader const*>(data);
            uint32_t    fid   = hdr->frame_id;
            uint16_t    cidx  = hdr->chunk_idx;
            uint16_t    total = hdr->total_chunks;
            uint32_t    plen  = hdr->payload_len;

            // Track the newest frame_id we have ever seen
            if (fid > max_seen_fid)
            {
                max_seen_fid = fid;

                // Evict assemblers that have fallen too far behind
                for (auto it = assemblers.begin(); it != assemblers.end(); )
                {
                    if (max_seen_fid - it->first > STALE_FRAME_WINDOW)
                        it = assemblers.erase(it);
                    else
                        ++it;
                }
            }

            // Discard chunks for frames that were already evicted
            if (max_seen_fid - fid > STALE_FRAME_WINDOW)
                return;

            if (assemblers.find(fid) == assemblers.end())
                assemblers.emplace(fid, FrameAssembler(fid, total));

            auto& asmblr = assemblers.at(fid);
            if (asmblr.chunks.find(cidx) != asmblr.chunks.end())
                return; // duplicate

            uint32_t payload_offset = static_cast<uint32_t>(sizeof(ChunkHeader));
            if (plen > static_cast<uint32_t>(size) - payload_offset)
                plen = static_cast<uint32_t>(size) - payload_offset;

            asmblr.chunks[cidx] = std::vector<uint8_t>(
                data + payload_offset,
                data + payload_offset + plen);
            asmblr.chunks_received++;

            if (asmblr.isComplete())
            {
                // Newest-wins: only update the display buffer if this frame
                // is newer than whatever is already pending decode.
                if (!pending_ready || fid > pending_frame_id)
                {
                    pending_jpeg     = asmblr.reassemble();
                    pending_frame_id = fid;
                    pending_ready    = true;
                }

                // Clean up this and any older assemblers
                for (auto it = assemblers.begin(); it != assemblers.end(); )
                {
                    if (it->first <= fid)
                        it = assemblers.erase(it);
                    else
                        ++it;
                }
            }
        });

    // ------------------------------------------------------------------
    // Cyclic loop: FMMU-driven mailbox read + timestamp-gated display
    // ------------------------------------------------------------------
    link->setTimeout(1ms);  // tighter timeout keeps the loop fast

    auto err_cb = [](DatagramState const& s)
    {
        THROW_ERROR_DATAGRAM("EtherCAT datagram error", s);
    };

    cv::Mat      display_frame;
    bool         window_opened  = false;
    uint32_t     frames_shown   = 0;
    nanoseconds  last_display_t = since_epoch();

    static constexpr nanoseconds DISPLAY_PERIOD = 33ms; // ≈30 fps

    printf("Starting EoE camera receive loop. Press 'q' to quit.\n");

    while (true)
    {
        try
        {
            // ── Round-trip 1: PDO exchange + can_read update via FMMU ──
            // sendLogicalRead reads the SM1 status bit that was mapped by
            // configureMailboxStatusCheck(READ_CHECK) into the LRD frame.
            // After processAwaitingFrames() returns, slave.mailbox.can_read
            // reflects whether the slave's output mailbox has data — no extra
            // FPRD needed.
            bus.sendLogicalRead(err_cb);
            bus.sendLogicalWrite(err_cb);
            bus.finalizeDatagrams();
            bus.processAwaitingFrames();

            // ── Round-trip 2 (only if mailbox has data): fetch the chunk ──
            // sendReadMessages checks can_read; if true it issues an FPRD to
            // read the SM content, which triggers the EoE receive callback.
            bus.sendReadMessages(err_cb);
            bus.finalizeDatagrams();
            bus.processAwaitingFrames();
        }
        catch (std::exception const& e)
        {
            printf("Loop error: %s\n", e.what());
        }

        // ── Display gate: only decode + show at ≈30 fps ──
        // Calling cv::waitKey(1) every iteration would add ≥1 ms of overhead
        // per cycle; we batch that cost to the display period instead.
        nanoseconds now = since_epoch();
        if ((now - last_display_t) >= DISPLAY_PERIOD)
        {
            last_display_t = now;

            if (pending_ready)
            {
                cv::Mat img = cv::imdecode(pending_jpeg, cv::IMREAD_COLOR);
                if (!img.empty())
                {
                    display_frame = std::move(img);
                    window_opened = true;
                    ++frames_shown;
                    if ((frames_shown % 30) == 0)
                    {
                        printf("[Display] %u frames shown, latest frame_id=%u  "
                               "(dropped up to fid %u)\n",
                               frames_shown, pending_frame_id,
                               max_seen_fid > pending_frame_id
                                   ? max_seen_fid - pending_frame_id : 0u);
                    }
                    last_shown_fid = pending_frame_id;
                }
                pending_ready = false;
            }

            if (window_opened)
                cv::imshow("EoE Camera Stream", display_frame);

            int key = cv::waitKey(1);
            if (key == 'q' || key == 27)
                break;
        }
    }

    cv::destroyAllWindows();
    printf("Done. Frames shown: %u  last frame_id: %u\n", frames_shown, last_shown_fid);
    return 0;
}
