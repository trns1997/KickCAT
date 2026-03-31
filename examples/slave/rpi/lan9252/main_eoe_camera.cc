//
// EoE Camera Slave — Raspberry Pi / LAN9252
//
// Captures camera frames, JPEG-encodes them, fragments into application-level
// chunks, and streams each chunk as an EoE Ethernet frame to the EtherCAT master.
//
// Build requirement: OpenCV must be available in the build environment.
//
// Usage (on the RPi, run as root for raw Ethernet):
//   sudo ./eoe_camera_slave_rpi [camera_index]
//
// Design notes:
//   - Camera capture runs in a dedicated thread so it never stalls
//     slave.routine() mailbox service.
//   - Each "application chunk" = ChunkHeader (12 B) + JPEG payload (≤980 B).
//     With a 1024-byte EoE mailbox, 992 bytes fit in a single EoE fragment,
//     so each chunk is delivered in exactly one mailbox exchange.
//   - Only one frame is buffered at a time; the camera thread always
//     overwrites with the latest encode result (frame-drop is preferred
//     over queue build-up which would increase latency).
//

#include <opencv2/opencv.hpp>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "kickcat/CoE/OD.h"
#include "kickcat/ESC/Lan9252.h"
#include "kickcat/OS/Time.h"
#include "kickcat/PDO.h"
#include "kickcat/protocol.h"
#include "kickcat/rpi/SPI.h"
#include "kickcat/slave/Slave.h"

using namespace kickcat;

// ---------------------------------------------------------------------------
// Application-level chunk protocol  (must match eoe_camera_master.cc)
// ---------------------------------------------------------------------------
struct ChunkHeader
{
    uint32_t frame_id;
    uint16_t chunk_idx;
    uint16_t total_chunks;
    uint32_t payload_len;
} __attribute__((packed));

// 992 bytes  = max EoE payload for a 1024-byte mailbox SM
//             ((1024 - sizeof(mailbox::Header=6) - sizeof(EoE::Header=4)) / 32) * 32
// One chunk fits in exactly one EoE mailbox exchange.
static constexpr uint16_t EOE_FRAG_CAPACITY    = 992;
static constexpr uint16_t MAX_CHUNK_PAYLOAD    = EOE_FRAG_CAPACITY - static_cast<uint16_t>(sizeof(ChunkHeader));

// ---------------------------------------------------------------------------
// Shared state between camera thread and EtherCAT main loop
// ---------------------------------------------------------------------------
struct FrameBuffer
{
    std::mutex           mtx;
    std::vector<uint8_t> jpeg;
    uint32_t             frame_id{0};
    bool                 ready{false};
};

// ---------------------------------------------------------------------------
// Camera capture thread
// ---------------------------------------------------------------------------
static void cameraThread(int cam_idx, FrameBuffer& fb, std::atomic<bool> const& running)
{
    cv::VideoCapture cap(cam_idx);
    if (!cap.isOpened())
    {
        printf("[Camera] Cannot open camera index %d\n", cam_idx);
        return;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH,  640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS,          30);

    std::vector<int> const params = {cv::IMWRITE_JPEG_QUALITY, 60};
    cv::Mat          frame;
    std::vector<uint8_t> encoded;

    printf("[Camera] Started (640×480 @ 30 fps, JPEG quality=60)\n");

    while (running.load(std::memory_order_relaxed))
    {
        cap >> frame;
        if (frame.empty())
        {
            continue;
        }

        cv::imencode(".jpg", frame, encoded, params);

        {
            std::lock_guard<std::mutex> lock(fb.mtx);
            fb.jpeg    = encoded;   // overwrite — keep only the latest
            fb.frame_id++;
            fb.ready   = true;
        }
    }

    cap.release();
    printf("[Camera] Stopped\n");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    int cam_idx = 0;
    if (argc > 1)
    {
        cam_idx = std::atoi(argv[1]);
    }

    printf("KickCAT EoE Camera Slave (RPi / LAN9252), camera=%d\n", cam_idx);

    // ------------------------------------------------------------------
    // ESC hardware init
    // ------------------------------------------------------------------
    auto spi = std::make_shared<SPI>();
    spi->setChipSelect(8); // CE0 (GPIO 8)
    spi->open("bcm2835-spi", 0, 0, 10000000);

    Lan9252 esc(spi);
    int32_t rc = esc.init();
    if (rc < 0)
    {
        printf("Lan9252 init failed: %d\n", rc);
        return rc;
    }

    uint8_t esc_cfg;
    esc.read(reg::ESC_CONFIG, &esc_cfg, sizeof(esc_cfg));
    printf("ESC config: 0x%02x  emulated=%d\n", esc_cfg, (esc_cfg & PDI_EMULATION) != 0);

    // ------------------------------------------------------------------
    // PDO setup  (minimal 1-byte in/out — actual data goes over EoE)
    // ------------------------------------------------------------------
    PDO pdo(&esc);
    slave::Slave slave_dev(&esc, &pdo);

    constexpr uint32_t PDO_SIZE = 4;   // matches od_populator.cc PDO mapping
    uint8_t buffer_in [PDO_SIZE]{};
    uint8_t buffer_out[PDO_SIZE]{0xFF, 0xFF, 0xFF, 0xFF};

    // ------------------------------------------------------------------
    // Mailbox: CoE + EoE
    // ------------------------------------------------------------------
    mailbox::response::Mailbox mbx(&esc, 1024);

    auto dictionary = CoE::createOD();
    mbx.enableCoE(std::move(dictionary));

    EoE::IpParam ip{};
    ip.ip_set     = true; ip.ip[0] = 192; ip.ip[1] = 168; ip.ip[2] = 10; ip.ip[3] = 2;
    ip.subnet_set = true; ip.subnet[0] = 255; ip.subnet[1] = 255; ip.subnet[2] = 255; ip.subnet[3] = 0;
    ip.mac_set    = true; ip.mac[0] = 0x02; ip.mac[5] = 0x02;

    // Receive callback — drop any frames the master sends (one-way stream PoC)
    mbx.enableEoE(ip, [](uint8_t const*, uint16_t) {});

    slave_dev.setMailbox(&mbx);
    pdo.setInput(buffer_in,  PDO_SIZE);
    pdo.setOutput(buffer_out, PDO_SIZE);

    slave_dev.start();

    // ------------------------------------------------------------------
    // Start camera thread
    // ------------------------------------------------------------------
    FrameBuffer           fb;
    std::atomic<bool>     running{true};
    std::thread           cam_thread(cameraThread, cam_idx, std::ref(fb), std::cref(running));

    // ------------------------------------------------------------------
    // Main EtherCAT loop
    // ------------------------------------------------------------------
    uint32_t last_sent_frame_id = UINT32_MAX;

    while (true)
    {
        slave_dev.routine();

        State state = slave_dev.state();

        if (state == State::SAFE_OP)
        {
            // Validate outputs once master zeroes them to let the slave go to OP
            if (buffer_out[0] != 0xFF)
            {
                slave_dev.validateOutputData();
            }
        }
        else if (state == State::OPERATIONAL)
        {
            // Grab latest JPEG from camera thread (non-blocking)
            bool                 has_frame = false;
            std::vector<uint8_t> jpeg;
            uint32_t             fid = 0;

            {
                std::lock_guard<std::mutex> lock(fb.mtx);
                if (fb.ready)
                {
                    jpeg      = fb.jpeg;    // copy once
                    fid       = fb.frame_id;
                    fb.ready  = false;
                    has_frame = true;
                }
            }

            if (has_frame && fid != last_sent_frame_id)
            {
                last_sent_frame_id = fid;

                size_t   total_bytes  = jpeg.size();
                uint16_t total_chunks = static_cast<uint16_t>(
                    (total_bytes + MAX_CHUNK_PAYLOAD - 1) / MAX_CHUNK_PAYLOAD);

                uint8_t pkt[EOE_FRAG_CAPACITY];

                for (uint16_t c = 0; c < total_chunks; ++c)
                {
                    auto* hdr        = reinterpret_cast<ChunkHeader*>(pkt);
                    hdr->frame_id    = fid;
                    hdr->chunk_idx   = c;
                    hdr->total_chunks = total_chunks;

                    size_t offset = static_cast<size_t>(c) * MAX_CHUNK_PAYLOAD;
                    size_t plen   = std::min(
                        static_cast<size_t>(MAX_CHUNK_PAYLOAD),
                        total_bytes - offset);
                    hdr->payload_len = static_cast<uint32_t>(plen);

                    std::memcpy(pkt + sizeof(ChunkHeader), jpeg.data() + offset, plen);

                    mbx.queueEoEFrame(0, pkt, static_cast<uint16_t>(sizeof(ChunkHeader) + plen));
                }

                printf("[EoE TX] frame #%u  size=%zu B  chunks=%u\n",
                       fid, total_bytes, total_chunks);
            }
        }

        sleep(100us); // avoid 100 % CPU; mailbox latency << 100 µs is fine
    }

    running.store(false, std::memory_order_relaxed);
    cam_thread.join();
    return 0;
}
