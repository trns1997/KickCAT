#ifndef KICKCAT_EOE_MAILBOX_RESPONSE_H
#define KICKCAT_EOE_MAILBOX_RESPONSE_H

#include <vector>
#include <memory>

#include "kickcat/Mailbox.h"
#include "kickcat/EoE/protocol.h"

namespace kickcat::mailbox::response
{
    std::shared_ptr<AbstractMessage> createEoEMessage(
            Mailbox* mbx,
            std::vector<uint8_t>&& raw_message);

    /// Handles an incoming EoE SET IP request from the master
    class EoESetIpHandler final : public AbstractMessage
    {
    public:
        EoESetIpHandler(Mailbox* mbx, std::vector<uint8_t>&& raw_message);
        virtual ~EoESetIpHandler() = default;

        ProcessingResult process() override;
        ProcessingResult process(std::vector<uint8_t> const& raw_message) override;

    private:
        EoE::IpParam     parsed_params_{};
        mailbox::Header* header_;
        EoE::Header*     eoe_;
    };

    /// Handles an incoming EoE GET IP request from the master
    class EoEGetIpHandler final : public AbstractMessage
    {
    public:
        EoEGetIpHandler(Mailbox* mbx, std::vector<uint8_t>&& raw_message);
        virtual ~EoEGetIpHandler() = default;

        ProcessingResult process() override;
        ProcessingResult process(std::vector<uint8_t> const& raw_message) override;

    private:
        mailbox::Header* header_;
        EoE::Header*     eoe_;
    };

    /// Handles incoming EoE Ethernet frame fragments from the master
    /// Reassembles multi-fragment frames and delivers them to the user callback.
    class EoEFrameDataHandler final : public AbstractMessage
    {
    public:
        EoEFrameDataHandler(Mailbox* mbx, std::vector<uint8_t>&& raw_message);
        virtual ~EoEFrameDataHandler() = default;

        ProcessingResult process() override;
        ProcessingResult process(std::vector<uint8_t> const& raw_message) override;

    private:
        ProcessingResult processFragment(EoE::Header const* eoe, uint8_t const* data, uint16_t size);

        std::vector<uint8_t> frame_buffer_;
        uint8_t  rx_fragment_no_{0};
        uint16_t rx_frame_size_{0};
        uint16_t rx_frame_offset_{0};
        uint8_t  rx_frame_no_{0xFF};
    };
}

#endif
