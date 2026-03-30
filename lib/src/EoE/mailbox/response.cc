#include <cstring>

#include "debug.h"
#include "kickcat/EoE/mailbox/response.h"
#include "protocol.h"

namespace kickcat::mailbox::response
{
    // Bit positions in the EoE parameter flags byte (ETG1000.6 chapter 5.7.4)
    static constexpr uint8_t FLAG_MAC      = (1 << 0);
    static constexpr uint8_t FLAG_IP       = (1 << 1);
    static constexpr uint8_t FLAG_SUBNET   = (1 << 2);
    static constexpr uint8_t FLAG_GATEWAY  = (1 << 3);
    static constexpr uint8_t FLAG_DNS_IP   = (1 << 4);
    static constexpr uint8_t FLAG_DNS_NAME = (1 << 5);

    // Layout of the EoE IP parameter payload: [flags(1), reserved(3), params...]
    static constexpr uint16_t PARAM_FLAGS_OFFSET = 0;
    static constexpr uint16_t PARAM_DATA_OFFSET  = 4;

    static constexpr uint16_t MAX_ETH_FRAME = 1518;


    // -------------------------------------------------------------------------
    // Factory
    // -------------------------------------------------------------------------

    std::shared_ptr<AbstractMessage> createEoEMessage(Mailbox* mbx, std::vector<uint8_t>&& raw_message)
    {
        auto const* header = pointData<mailbox::Header>(raw_message.data());
        if (header->type != mailbox::Type::EoE)
        {
            return nullptr;
        }

        auto const* eoe = pointData<EoE::Header>(header);
        switch (eoe->type)
        {
            case EoE::request::SET_IP:
            {
                return std::make_shared<EoESetIpHandler>(mbx, std::move(raw_message));
            }
            case EoE::request::GET_IP:
            {
                return std::make_shared<EoEGetIpHandler>(mbx, std::move(raw_message));
            }
            case 0:  // FRAG_DATA
            {
                if (eoe->fragment_number != 0)
                {
                    // Orphan continuation fragment with no matching handler — drop it
                    eoe_warning("EoE slave: orphan fragment (no=%d), dropping\n", eoe->fragment_number);
                    return nullptr;
                }
                return std::make_shared<EoEFrameDataHandler>(mbx, std::move(raw_message));
            }
            default:
            {
                return nullptr;
            }
        }
    }


    // -------------------------------------------------------------------------
    // EoESetIpHandler
    // -------------------------------------------------------------------------

    EoESetIpHandler::EoESetIpHandler(Mailbox* mbx, std::vector<uint8_t>&& raw_message)
        : AbstractMessage{mbx}
    {
        data_   = std::move(raw_message);
        header_ = pointData<mailbox::Header>(data_.data());
        eoe_    = pointData<EoE::Header>(header_);

        // Parse parameter payload
        auto const* data        = pointData<uint8_t>(eoe_);
        uint16_t    data_size   = static_cast<uint16_t>(header_->len - sizeof(EoE::Header));

        if (data_size < PARAM_DATA_OFFSET)
        {
            return;
        }

        uint8_t  flags  = data[PARAM_FLAGS_OFFSET];
        uint16_t offset = PARAM_DATA_OFFSET;

        if ((flags & FLAG_MAC) && (offset + sizeof(EoE::MAC) <= data_size))
        {
            std::memcpy(parsed_params_.mac, data + offset, sizeof(EoE::MAC));
            parsed_params_.mac_set = true;
            offset += sizeof(EoE::MAC);
        }
        if ((flags & FLAG_IP) && (offset + sizeof(EoE::IP) <= data_size))
        {
            std::memcpy(parsed_params_.ip, data + offset, sizeof(EoE::IP));
            parsed_params_.ip_set = true;
            offset += sizeof(EoE::IP);
        }
        if ((flags & FLAG_SUBNET) && (offset + sizeof(EoE::SUBNET_MASK) <= data_size))
        {
            std::memcpy(parsed_params_.subnet, data + offset, sizeof(EoE::SUBNET_MASK));
            parsed_params_.subnet_set = true;
            offset += sizeof(EoE::SUBNET_MASK);
        }
        if ((flags & FLAG_GATEWAY) && (offset + sizeof(EoE::DEFAULT_GATEWAY) <= data_size))
        {
            std::memcpy(parsed_params_.gateway, data + offset, sizeof(EoE::DEFAULT_GATEWAY));
            parsed_params_.gateway_set = true;
            offset += sizeof(EoE::DEFAULT_GATEWAY);
        }
        if ((flags & FLAG_DNS_IP) && (offset + sizeof(EoE::DNS_SERVER_IP) <= data_size))
        {
            std::memcpy(parsed_params_.dns_ip, data + offset, sizeof(EoE::DNS_SERVER_IP));
            parsed_params_.dns_ip_set = true;
            offset += sizeof(EoE::DNS_SERVER_IP);
        }
        if ((flags & FLAG_DNS_NAME) && (offset + sizeof(EoE::DNS_NAME) <= data_size))
        {
            std::memcpy(parsed_params_.dns_name, data + offset, sizeof(EoE::DNS_NAME));
            parsed_params_.dns_name_set = true;
        }
    }

    ProcessingResult EoESetIpHandler::process()
    {
        // Apply the new IP configuration
        mailbox_->eoeIpParam() = parsed_params_;

        // Build SET_IP response (type = 0x03), result = SUCCESS
        eoe_->type          = EoE::response::SET_IP;
        eoe_->last_fragment = 1;
        eoe_->time_appended = 0;
        eoe_->time_request  = 0;

        // Result code occupies bytes 2-3 of the EoE header (same location as
        // fragment_number / offset / frame_number bitfields).
        uint16_t result = EoE::result::SUCCESS;
        std::memcpy(reinterpret_cast<uint8_t*>(eoe_) + 2, &result, sizeof(uint16_t));

        header_->type = mailbox::Type::EoE;
        header_->len  = static_cast<uint16_t>(sizeof(EoE::Header));

        reply(std::move(data_));
        return ProcessingResult::FINALIZE;
    }

    ProcessingResult EoESetIpHandler::process(std::vector<uint8_t> const&)
    {
        return ProcessingResult::NOOP;
    }


    // -------------------------------------------------------------------------
    // EoEGetIpHandler
    // -------------------------------------------------------------------------

    EoEGetIpHandler::EoEGetIpHandler(Mailbox* mbx, std::vector<uint8_t>&& raw_message)
        : AbstractMessage{mbx}
    {
        data_   = std::move(raw_message);
        header_ = pointData<mailbox::Header>(data_.data());
        eoe_    = pointData<EoE::Header>(header_);
    }

    ProcessingResult EoEGetIpHandler::process()
    {
        EoE::IpParam const& ip = mailbox_->eoeIpParam();

        // Build GET_IP response (type = 0x07)
        eoe_->type          = EoE::response::GET_IP;
        eoe_->last_fragment = 1;
        eoe_->time_appended = 0;
        eoe_->time_request  = 0;

        // Result code at bytes 2-3 of the EoE header
        uint16_t result = EoE::result::SUCCESS;
        std::memcpy(reinterpret_cast<uint8_t*>(eoe_) + 2, &result, sizeof(uint16_t));

        header_->type = mailbox::Type::EoE;

        // Build parameter payload
        auto*    data   = pointData<uint8_t>(eoe_);
        uint8_t  flags  = 0;
        uint16_t offset = PARAM_DATA_OFFSET;

        // Clear reserved bytes
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;

        if (ip.mac_set)
        {
            flags |= FLAG_MAC;
            std::memcpy(data + offset, ip.mac, sizeof(EoE::MAC));
            offset += sizeof(EoE::MAC);
        }
        if (ip.ip_set)
        {
            flags |= FLAG_IP;
            std::memcpy(data + offset, ip.ip, sizeof(EoE::IP));
            offset += sizeof(EoE::IP);
        }
        if (ip.subnet_set)
        {
            flags |= FLAG_SUBNET;
            std::memcpy(data + offset, ip.subnet, sizeof(EoE::SUBNET_MASK));
            offset += sizeof(EoE::SUBNET_MASK);
        }
        if (ip.gateway_set)
        {
            flags |= FLAG_GATEWAY;
            std::memcpy(data + offset, ip.gateway, sizeof(EoE::DEFAULT_GATEWAY));
            offset += sizeof(EoE::DEFAULT_GATEWAY);
        }
        if (ip.dns_ip_set)
        {
            flags |= FLAG_DNS_IP;
            std::memcpy(data + offset, ip.dns_ip, sizeof(EoE::DNS_SERVER_IP));
            offset += sizeof(EoE::DNS_SERVER_IP);
        }
        if (ip.dns_name_set)
        {
            flags |= FLAG_DNS_NAME;
            std::memcpy(data + offset, ip.dns_name, sizeof(EoE::DNS_NAME));
            offset += sizeof(EoE::DNS_NAME);
        }

        data[PARAM_FLAGS_OFFSET] = flags;
        header_->len = static_cast<uint16_t>(sizeof(EoE::Header) + offset);

        reply(std::move(data_));
        return ProcessingResult::FINALIZE;
    }

    ProcessingResult EoEGetIpHandler::process(std::vector<uint8_t> const&)
    {
        return ProcessingResult::NOOP;
    }


    // -------------------------------------------------------------------------
    // EoEFrameDataHandler
    // -------------------------------------------------------------------------

    EoEFrameDataHandler::EoEFrameDataHandler(Mailbox* mbx, std::vector<uint8_t>&& raw_message)
        : AbstractMessage{mbx}
    {
        data_ = std::move(raw_message);
        frame_buffer_.resize(MAX_ETH_FRAME);

        // Initialise reassembly state from the first fragment header
        auto const* header  = pointData<mailbox::Header>(data_.data());
        auto const* eoe     = pointData<EoE::Header>(header);

        rx_fragment_no_  = 0;
        rx_frame_no_     = static_cast<uint8_t>(eoe->frame_number);
        // First fragment: offset field encodes total frame size in 32-byte units
        rx_frame_size_   = static_cast<uint16_t>(eoe->offset << 5);
        rx_frame_offset_ = 0;

        if (rx_frame_size_ > MAX_ETH_FRAME)
        {
            rx_frame_size_ = 0;  // Will cause an early drop in process()
        }
    }

    ProcessingResult EoEFrameDataHandler::process()
    {
        auto const* header      = pointData<mailbox::Header>(data_.data());
        auto const* eoe         = pointData<EoE::Header>(header);
        auto const* frag_data   = pointData<uint8_t>(eoe);
        uint16_t    frag_size   = static_cast<uint16_t>(header->len - sizeof(EoE::Header));

        return processFragment(eoe, frag_data, frag_size);
    }

    ProcessingResult EoEFrameDataHandler::process(std::vector<uint8_t> const& raw_message)
    {
        auto const* header = pointData<mailbox::Header>(raw_message.data());
        if (header->type != mailbox::Type::EoE)
        {
            return ProcessingResult::NOOP;
        }

        auto const* eoe = pointData<EoE::Header>(header);
        if (eoe->type != 0)  // Not FRAG_DATA
        {
            return ProcessingResult::NOOP;
        }

        if (static_cast<uint8_t>(eoe->frame_number) != rx_frame_no_)
        {
            return ProcessingResult::NOOP;  // Belongs to a different frame — ignore
        }

        auto const* frag_data = pointData<uint8_t>(eoe);
        uint16_t    frag_size = static_cast<uint16_t>(header->len - sizeof(EoE::Header));

        return processFragment(eoe, frag_data, frag_size);
    }

    ProcessingResult EoEFrameDataHandler::processFragment(EoE::Header const* eoe,
                                                          uint8_t const* data,
                                                          uint16_t size)
    {
        if (rx_frame_size_ == 0)
        {
            eoe_warning("EoE slave: frame too large, dropping\n");
            return ProcessingResult::FINALIZE;
        }

        if (eoe->fragment_number != rx_fragment_no_)
        {
            eoe_warning("EoE slave: fragment mismatch (expected %d, got %d), dropping\n",
                        rx_fragment_no_, eoe->fragment_number);
            return ProcessingResult::FINALIZE;
        }

        // Validate cumulative offset for continuation fragments
        if (rx_fragment_no_ > 0)
        {
            uint16_t expected_offset = static_cast<uint16_t>(eoe->offset << 5);
            if (rx_frame_offset_ != expected_offset)
            {
                eoe_warning("EoE slave: offset mismatch (expected %d, got %d), dropping\n",
                            rx_frame_offset_, expected_offset);
                return ProcessingResult::FINALIZE;
            }
        }

        // Copy fragment data into reassembly buffer
        if (rx_frame_offset_ + size > static_cast<uint16_t>(frame_buffer_.size()))
        {
            eoe_warning("EoE slave: fragment overflow, dropping\n");
            return ProcessingResult::FINALIZE;
        }

        std::memcpy(frame_buffer_.data() + rx_frame_offset_, data, size);
        rx_frame_offset_ += size;
        rx_fragment_no_++;

        if (eoe->last_fragment)
        {
            uint16_t frame_size = rx_frame_offset_;
            if (eoe->time_appended && frame_size >= 4)
            {
                frame_size -= 4;
            }

            auto const& callback = mailbox_->eoeOnFrame();
            if (callback)
            {
                callback(frame_buffer_.data(), frame_size);
            }

            return ProcessingResult::FINALIZE;
        }

        return ProcessingResult::FINALIZE_AND_KEEP;
    }
}
