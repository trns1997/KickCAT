
#include "kickcat/ESC/Lan9252.h"
#include "kickcat/OS/Time.h"
#include "kickcat/PDO.h"
#include "kickcat/nuttx/SPI.h"
#include "kickcat/protocol.h"
#include "kickcat/slave/Slave.h"

#include <arch/board/board.h>
#include <nuttx/board.h>
#include <nuttx/sensors/fxos8700cq.h>
#include <nuttx/leds/userled.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstdio>

using namespace kickcat;

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    std::shared_ptr<SPI> spi_driver = std::make_shared<SPI>();
    spi_driver->open("/dev/spi0", 0, 0, 10000000);

    Lan9252 esc = Lan9252(spi_driver);
    int32_t rc = esc.init();
    if (rc < 0)
    {
        printf("error init %ld - %s\n", rc, strerror(-rc));
    }
    PDO pdo(&esc);
    slave::Slave slave(&esc, &pdo);

    constexpr uint32_t PDO_MAX_SIZE = 16;

    uint8_t buffer_in[PDO_MAX_SIZE];
    uint8_t buffer_out[PDO_MAX_SIZE];

    for (uint32_t i = 0; i < PDO_MAX_SIZE; ++i)
    {
        buffer_in[i]  = i;
        buffer_out[i] = 0xFF;
    }

    mailbox::response::Mailbox mbx(&esc, 1024);

    // Enable CoE with the standard object dictionary
    auto dictionary = CoE::createOD();
    mbx.enableCoE(std::move(dictionary));

    // Enable EoE: initial IP config advertised to the master
    EoE::IpParam initial_ip{};
    initial_ip.ip_set = true;
    initial_ip.ip[0] = 192; initial_ip.ip[1] = 168;
    initial_ip.ip[2] = 1;   initial_ip.ip[3] = 200;

    initial_ip.subnet_set = true;
    initial_ip.subnet[0] = 255; initial_ip.subnet[1] = 255;
    initial_ip.subnet[2] = 255; initial_ip.subnet[3] = 0;

    initial_ip.gateway_set = true;
    initial_ip.gateway[0] = 192; initial_ip.gateway[1] = 168;
    initial_ip.gateway[2] = 1;   initial_ip.gateway[3] = 1;

    initial_ip.mac_set = true;
    initial_ip.mac[0] = 0x02; initial_ip.mac[1] = 0x00; initial_ip.mac[2] = 0x00;
    initial_ip.mac[3] = 0x00; initial_ip.mac[4] = 0x00; initial_ip.mac[5] = 0x02;

    uint32_t frames_received = 0;
    mbx.enableEoE(initial_ip,
        [&](uint8_t const* data, uint16_t size)
        {
            ++frames_received;
            printf("[EoE RX] %d-byte frame (total: %u)\n", size, frames_received);
            printf("         dst=%02x:%02x:%02x:%02x:%02x:%02x "
                   "src=%02x:%02x:%02x:%02x:%02x:%02x "
                   "ethertype=%02x%02x\n",
                   data[0], data[1], data[2], data[3], data[4], data[5],
                   data[6], data[7], data[8], data[9], data[10], data[11],
                   data[12], data[13]);

            // Echo the frame back to the master
            mbx.queueEoEFrame(0, data, size);
        });

    slave.setMailbox(&mbx);
    pdo.setInput(buffer_in, PDO_MAX_SIZE);
    pdo.setOutput(buffer_out, PDO_MAX_SIZE);

    uint8_t esc_config;
    esc.read(reg::ESC_CONFIG, &esc_config, sizeof(esc_config));

    bool is_emulated = esc_config & PDI_EMULATION;
    printf("esc config 0x%x, is emulated %i \n", esc_config, is_emulated);

    slave.start();

    // Init sensor — O_NONBLOCK so read() returns EAGAIN instead of blocking
    // when no new sample is ready.  Without this, slave.routine() (mailbox
    // service) is only called when the sensor produces data, causing EoE
    // mailbox timeouts on the master side.
    int sensor_fd = open("/dev/accel0", O_RDONLY | O_NONBLOCK);
    if (sensor_fd < 0)
    {
        printf("Failed to open sensor device\n");
        return -1;
    }

    fxos8700cq_data sensor_data;

    // Init userleds
    int led_fd = open("/dev/userleds", O_WRONLY);
    if (led_fd < 0)
    {
        printf("Failed to open LED driver\n");
        return -1;
    }

    constexpr uint8_t LED_R_BIT = 1 << 0;
    constexpr uint8_t LED_G_BIT = 1 << 1;
    constexpr uint8_t LED_B_BIT = 1 << 2;

    int16_t *ax = nullptr;
    int16_t *ay = nullptr;
    int16_t *az = nullptr;
    int16_t *mx = nullptr;
    int16_t *my = nullptr;
    int16_t *mz = nullptr;

    uint8_t *led_r = nullptr;
    uint8_t *led_g = nullptr;
    uint8_t *led_b = nullptr;

    while (true)
    {
        slave.routine();

        const State state = slave.state();

        if (state == State::SAFE_OP)
        {
            slave.bind(0x6000, ax);
            slave.bind(0x6001, ay);
            slave.bind(0x6002, az);
            slave.bind(0x6003, mx);
            slave.bind(0x6004, my);
            slave.bind(0x6005, mz);
            slave.bind(0x7000, led_r);
            slave.bind(0x7001, led_g);
            slave.bind(0x7002, led_b);

            if (buffer_out[1] != 0xFF)
            {
                slave.validateOutputData();
            }
        }
        else if (state == State::OPERATIONAL)
        {
            // Non-blocking read: only update if a full sample is available.
            // EAGAIN means no new data yet — keep the previous values.
            ssize_t n = read(sensor_fd, &sensor_data, sizeof(sensor_data));
            if (n == sizeof(sensor_data))
            {
                *ax = sensor_data.accel.x;
                *ay = sensor_data.accel.y;
                *az = sensor_data.accel.z;
                *mx = sensor_data.magn.x;
                *my = sensor_data.magn.y;
                *mz = sensor_data.magn.z;
            }

            userled_set_t led_set = 0;
            if (*led_r) { led_set |= LED_R_BIT; }
            if (*led_g) { led_set |= LED_G_BIT; }
            if (*led_b) { led_set |= LED_B_BIT; }

            if (ioctl(led_fd, ULEDIOC_SETALL, led_set) < 0)
            {
                printf("ERROR: ioctl(ULEDIOC_SETALL) failed: %s\n", strerror(errno));
            }
        }
    }

    close(sensor_fd);
    close(led_fd);

    return 0;
}
