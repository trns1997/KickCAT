//
// EoE Camera Slave — Raspberry Pi / LAN9252
//
// Captures camera frames, JPEG-encodes them, and streams each frame as a
// sequence of EoE Ethernet frames to the EtherCAT master.
//
// Usage:
//   sudo ./eoe_camera_slave_rpi [options]
//   Options:
//     -c <idx>     Camera device index          (default 0)
//     -q <0-100>   JPEG quality                 (default 30)
//     -w <pixels>  Frame width                  (default 320)
//     -h <pixels>  Frame height                 (default 240)
//     -f <fps>     Target capture/send rate     (default 30)
//
// Start with low quality (q=30, 320x240) and increase incrementally:
//   -q 30  -w 320  -h 240  → ~1.5-3 KB/frame  → 2-3 chunks
//   -q 50  -w 640  -h 480  → ~5-8 KB/frame    → 6-9 chunks
//   -q 70  -w 640  -h 480  → ~10-15 KB/frame  → 11-16 chunks
//
// Lag control
// -----------
// The slave rate-limits chunk queuing to at most 1 frame per (1000/fps) ms.
// This prevents queue build-up: the slave never queues a new frame until the
// inter-frame budget has expired, so the master's mailbox drain always keeps
// pace with the camera produce rate.
//
// Design notes
// ------------
// Camera capture runs in a dedicated thread so cv::VideoCapture::read() never
// blocks slave.routine() mailbox service.  Only the latest encoded frame is
// kept — the camera thread overwrites the shared buffer on every capture, so
// the main loop always sends the freshest JPEG, not a stale queued one.
//
// Each chunk = ChunkHeader (12 B) + payload (≤980 B) = ≤992 B, which fits in
// exactly one EoE fragment for a 1024-byte mailbox SM.
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

// One chunk must fit inside a single EoE fragment.
// For a 1024-byte mailbox: max_frag = ((1024-6-4)/32)*32 = 992 bytes.
static constexpr uint16_t EOE_FRAG_CAPACITY = 992;
static constexpr uint16_t MAX_CHUNK_PAYLOAD =
    EOE_FRAG_CAPACITY - static_cast<uint16_t>(sizeof(ChunkHeader)); // 980 bytes

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
static void cameraThread(int cam_idx, int width, int height, int quality,
                         FrameBuffer& fb, std::atomic<bool> const& running)
{
    cv::VideoCapture cap(cam_idx);
    if (!cap.isOpened())
    {
        printf("[Camera] Cannot open camera %d\n", cam_idx);
        return;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH,  width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS,          30);

    std::vector<int>     params = {cv::IMWRITE_JPEG_QUALITY, quality};
    cv::Mat              frame;
    std::vector<uint8_t> encoded;

    printf("[Camera] Started: %dx%d JPEG quality=%d\n", width, height, quality);

    while (running.load(std::memory_order_relaxed))
    {
        cap >> frame;
        if (frame.empty())
            continue;

        cv::imencode(".jpg", frame, encoded, params);

        {
            std::lock_guard<std::mutex> lock(fb.mtx);
            fb.jpeg = encoded;   // overwrite — always keep the latest frame
            fb.frame_id++;
            fb.ready = true;
        }
    }

    cap.release();
    printf("[Camera] Stopped\n");
}

// ---------------------------------------------------------------------------
// Simple CLI parser
// ---------------------------------------------------------------------------
static void usage(char const* prog)
{
    printf("Usage: %s [-c cam] [-q quality] [-w width] [-h height] [-f fps]\n", prog);
    printf("  -c  camera index        (default 0)\n");
    printf("  -q  JPEG quality 0-100  (default 30)\n");
    printf("  -w  frame width px      (default 320)\n");
    printf("  -h  frame height px     (default 240)\n");
    printf("  -f  target fps          (default 30)\n");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    int cam_idx     = 0;
    int jpeg_quality = 30;
    int frame_width  = 320;
    int frame_height = 240;
    int target_fps   = 30;

    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "-c") == 0 && i + 1 < argc) { cam_idx      = std::atoi(argv[++i]); }
        else if (std::strcmp(argv[i], "-q") == 0 && i + 1 < argc) { jpeg_quality = std::atoi(argv[++i]); }
        else if (std::strcmp(argv[i], "-w") == 0 && i + 1 < argc) { frame_width  = std::atoi(argv[++i]); }
        else if (std::strcmp(argv[i], "-h") == 0 && i + 1 < argc) { frame_height = std::atoi(argv[++i]); }
        else if (std::strcmp(argv[i], "-f") == 0 && i + 1 < argc) { target_fps   = std::atoi(argv[++i]); }
        else { usage(argv[0]); return 1; }
    }

    // Inter-frame budget: we do not queue a new frame more often than this
    nanoseconds const frame_budget_ns{1'000'000'000LL / target_fps};

    printf("KickCAT EoE Camera Slave (RPi/LAN9252)  cam=%d  %dx%d  q=%d  fps=%d\n",
           cam_idx, frame_width, frame_height, jpeg_quality, target_fps);

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
    // PDO setup  (minimal — camera data flows over EoE mailbox)
    // ------------------------------------------------------------------
    PDO pdo(&esc);
    slave::Slave slave_dev(&esc, &pdo);

    constexpr uint32_t PDO_SIZE = 4;
    uint8_t buffer_in [PDO_SIZE]{};
    uint8_t buffer_out[PDO_SIZE]{0xFF, 0xFF, 0xFF, 0xFF};

    // ------------------------------------------------------------------
    // Mailbox: CoE + EoE
    // ------------------------------------------------------------------
    mailbox::response::Mailbox mbx(&esc, 1024);

    auto dictionary = CoE::createOD();
    mbx.enableCoE(std::move(dictionary));

    EoE::IpParam ip{};
    ip.ip_set     = true;
    ip.ip[0] = 192; ip.ip[1] = 168; ip.ip[2] = 10; ip.ip[3] = 2;
    ip.subnet_set = true;
    ip.subnet[0] = 255; ip.subnet[1] = 255; ip.subnet[2] = 255; ip.subnet[3] = 0;
    ip.mac_set    = true;
    ip.mac[0] = 0x02; ip.mac[5] = 0x02;

    // One-way stream: discard any frames the master sends
    mbx.enableEoE(ip, [](uint8_t const*, uint16_t) {});

    slave_dev.setMailbox(&mbx);
    pdo.setInput(buffer_in,   PDO_SIZE);
    pdo.setOutput(buffer_out, PDO_SIZE);

    slave_dev.start();

    // ------------------------------------------------------------------
    // Camera thread
    // ------------------------------------------------------------------
    FrameBuffer       fb;
    std::atomic<bool> running{true};
    std::thread       cam_thread(cameraThread,
                                  cam_idx, frame_width, frame_height, jpeg_quality,
                                  std::ref(fb), std::cref(running));

    // ------------------------------------------------------------------
    // Main EtherCAT loop
    // ------------------------------------------------------------------
    uint32_t    last_sent_frame_id = UINT32_MAX;
    nanoseconds last_queue_t{0};  // timestamp of the last chunk-queue operation

    while (true)
    {
        slave_dev.routine();

        State state = slave_dev.state();

        if (state == State::SAFE_OP)
        {
            // Let the slave advance to OPERATIONAL once the master zeros outputs
            if (buffer_out[0] != 0xFF)
                slave_dev.validateOutputData();
        }
        else if (state == State::OPERATIONAL)
        {
            // ── Rate gate: do not queue faster than target_fps ──
            nanoseconds now = since_epoch();
            if ((now - last_queue_t) < frame_budget_ns)
            {
                sleep(100us);
                continue;
            }

            // ── Grab latest frame (non-blocking) ──
            bool                 has_frame = false;
            std::vector<uint8_t> jpeg;
            uint32_t             fid = 0;

            {
                std::lock_guard<std::mutex> lock(fb.mtx);
                if (fb.ready)
                {
                    jpeg      = fb.jpeg;
                    fid       = fb.frame_id;
                    fb.ready  = false;
                    has_frame = true;
                }
            }

            if (!has_frame || fid == last_sent_frame_id)
            {
                sleep(100us);
                continue;
            }

            // ── Fragment JPEG into chunks and queue each as one EoE frame ──
            last_sent_frame_id = fid;
            last_queue_t       = now;

            size_t   total_bytes  = jpeg.size();
            uint16_t total_chunks = static_cast<uint16_t>(
                (total_bytes + MAX_CHUNK_PAYLOAD - 1) / MAX_CHUNK_PAYLOAD);

            uint8_t pkt[EOE_FRAG_CAPACITY];

            for (uint16_t c = 0; c < total_chunks; ++c)
            {
                auto* hdr         = reinterpret_cast<ChunkHeader*>(pkt);
                hdr->frame_id     = fid;
                hdr->chunk_idx    = c;
                hdr->total_chunks = total_chunks;

                size_t offset        = static_cast<size_t>(c) * MAX_CHUNK_PAYLOAD;
                size_t plen          = std::min(static_cast<size_t>(MAX_CHUNK_PAYLOAD),
                                                total_bytes - offset);
                hdr->payload_len     = static_cast<uint32_t>(plen);

                std::memcpy(pkt + sizeof(ChunkHeader), jpeg.data() + offset, plen);

                mbx.queueEoEFrame(0, pkt,
                    static_cast<uint16_t>(sizeof(ChunkHeader) + plen));
            }

            printf("[EoE TX] frame #%u  %zu B  %u chunks\n",
                   fid, total_bytes, total_chunks);
        }

        sleep(100us);
    }

    running.store(false, std::memory_order_relaxed);
    cam_thread.join();
    return 0;
}
