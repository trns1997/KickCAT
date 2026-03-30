#ifndef KICKCAT_EOE_MAILBOX_REQUEST_H
#define KICKCAT_EOE_MAILBOX_REQUEST_H

#include <functional>
#include <vector>

#include "kickcat/Mailbox.h"
#include "kickcat/EoE/protocol.h"

namespace kickcat::mailbox::request
{
    /// \brief EoE SET IP parameter request - sends IP configuration to slave and waits for response
    class EoESetIpMessage final : public AbstractMessage
    {
    public:
        EoESetIpMessage(uint16_t mailbox_size, uint8_t port, EoE::IpParam const& params, nanoseconds timeout);
        ~EoESetIpMessage() override = default;

        ProcessingResult process(uint8_t const* received) override;
    };

    /// \brief EoE GET IP parameter request - queries IP configuration from slave and waits for response
    class EoEGetIpMessage final : public AbstractMessage
    {
    public:
        EoEGetIpMessage(uint16_t mailbox_size, uint8_t port, EoE::IpParam* params, nanoseconds timeout);
        ~EoEGetIpMessage() override = default;

        ProcessingResult process(uint8_t const* received) override;

    private:
        EoE::IpParam* params_;
    };

    /// \brief Single EoE Ethernet frame fragment - fire-and-forget, no response expected
    class EoEFrameFragment final : public AbstractMessage
    {
    public:
        EoEFrameFragment(uint16_t mailbox_size, uint8_t port, uint8_t fragment_no,
                         uint8_t frame_no, uint16_t offset_32, bool is_last,
                         uint8_t const* payload, uint16_t payload_size);
        ~EoEFrameFragment() override = default;

        ProcessingResult process(uint8_t const* received) override;
    };

    /// \brief Persistent handler for incoming EoE Ethernet frame fragments from slave
    /// \details Reassembles multi-fragment frames and invokes a callback on completion.
    ///          Designed to live in to_process indefinitely (returns FINALIZE_AND_KEEP).
    class EoEReceiveMessage final : public AbstractMessage
    {
    public:
        EoEReceiveMessage(Mailbox& mbx, std::function<void(uint8_t const*, uint16_t)> callback);
        ~EoEReceiveMessage() override = default;

        ProcessingResult process(uint8_t const* received) override;

    private:
        std::function<void(uint8_t const*, uint16_t)> callback_;
        std::vector<uint8_t> frame_buffer_;

        uint8_t  rx_fragment_no_{0};
        uint16_t rx_frame_size_{0};
        uint16_t rx_frame_offset_{0};
        uint8_t  rx_frame_no_{0xFF};
    };
}

#endif
