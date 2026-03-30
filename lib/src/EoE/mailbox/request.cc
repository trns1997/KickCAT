#include <cstring>

#include "debug.h"
#include "kickcat/EoE/mailbox/request.h"

namespace kickcat::mailbox::request
{
    // Offset of the flags byte within the EoE parameter data area.
    // data[0] = flags, data[1..3] = reserved, data[4..] = parameter fields.
    static constexpr uint16_t EOE_PARAM_FLAGS_OFFSET  = 0;
    static constexpr uint16_t EOE_PARAM_DATA_OFFSET   = 4;

    // Bit positions in the flags byte (ETG1000.6 chapter 5.7.4)
    static constexpr uint8_t FLAG_MAC     = (1 << 0);
    static constexpr uint8_t FLAG_IP      = (1 << 1);
    static constexpr uint8_t FLAG_SUBNET  = (1 << 2);
    static constexpr uint8_t FLAG_GATEWAY = (1 << 3);
    static constexpr uint8_t FLAG_DNS_IP  = (1 << 4);
    static constexpr uint8_t FLAG_DNS_NAME= (1 << 5);

    // Maximum standard Ethernet frame size (excluding VLAN tagging)
    static constexpr uint16_t MAX_ETH_FRAME = 1518;


    // -------------------------------------------------------------------------
    // EoESetIpMessage
    // -------------------------------------------------------------------------

    EoESetIpMessage::EoESetIpMessage(uint16_t mailbox_size, uint8_t port,
                                     EoE::IpParam const& params, nanoseconds timeout)
        : AbstractMessage(mailbox_size, timeout)
    {
        auto* eoe  = pointData<EoE::Header>(header_);
        auto* data = pointData<uint8_t>(eoe);

        header_->priority = 0;
        header_->channel  = 0;
        header_->type     = mailbox::Type::EoE;

        eoe->type            = EoE::request::SET_IP;
        eoe->port            = port;
        eoe->last_fragment   = 1;
        eoe->time_appended   = 0;
        eoe->time_request    = 0;
        eoe->fragment_number = 0;
        eoe->offset          = 0;
        eoe->frame_number    = 0;

        // Build parameter flags and payload (data[0]=flags, data[1-3]=reserved, data[4+]=params)
        uint8_t flags = 0;
        uint16_t param_offset = EOE_PARAM_DATA_OFFSET;

        if (params.mac_set)
        {
            flags |= FLAG_MAC;
            std::memcpy(data + param_offset, params.mac, sizeof(EoE::MAC));
            param_offset += sizeof(EoE::MAC);
        }
        if (params.ip_set)
        {
            flags |= FLAG_IP;
            std::memcpy(data + param_offset, params.ip, sizeof(EoE::IP));
            param_offset += sizeof(EoE::IP);
        }
        if (params.subnet_set)
        {
            flags |= FLAG_SUBNET;
            std::memcpy(data + param_offset, params.subnet, sizeof(EoE::SUBNET_MASK));
            param_offset += sizeof(EoE::SUBNET_MASK);
        }
        if (params.gateway_set)
        {
            flags |= FLAG_GATEWAY;
            std::memcpy(data + param_offset, params.gateway, sizeof(EoE::DEFAULT_GATEWAY));
            param_offset += sizeof(EoE::DEFAULT_GATEWAY);
        }
        if (params.dns_ip_set)
        {
            flags |= FLAG_DNS_IP;
            std::memcpy(data + param_offset, params.dns_ip, sizeof(EoE::DNS_SERVER_IP));
            param_offset += sizeof(EoE::DNS_SERVER_IP);
        }
        if (params.dns_name_set)
        {
            flags |= FLAG_DNS_NAME;
            std::memcpy(data + param_offset, params.dns_name, sizeof(EoE::DNS_NAME));
            param_offset += sizeof(EoE::DNS_NAME);
        }

        data[EOE_PARAM_FLAGS_OFFSET] = flags;
        // data[1..3] are reserved and already zero-initialized

        // len = EoE header (4) + flags+reserved (4) + parameters
        header_->len = static_cast<uint16_t>(sizeof(EoE::Header) + param_offset);
    }


    ProcessingResult EoESetIpMessage::process(uint8_t const* received)
    {
        auto const* header = pointData<mailbox::Header>(received);

        if ((header->address & mailbox::GATEWAY_MESSAGE_MASK) != 0)
        {
            return ProcessingResult::NOOP;
        }

        if (header->type != mailbox::Type::EoE)
        {
            return ProcessingResult::NOOP;
        }

        auto const* eoe = pointData<EoE::Header>(header);

        if (eoe->type != EoE::response::SET_IP)
        {
            return ProcessingResult::NOOP;
        }

        // Result code occupies bytes 2-3 of the EoE header (same location as frameinfo2)
        uint16_t result;
        std::memcpy(&result, reinterpret_cast<uint8_t const*>(eoe) + 2, sizeof(uint16_t));

        if (result != EoE::result::SUCCESS)
        {
            eoe_warning("EoE SET IP failed: %s (0x%04x)\n", EoE::result::toString(result), result);
            status_ = result;
        }
        else
        {
            status_ = MessageStatus::SUCCESS;
        }

        return ProcessingResult::FINALIZE;
    }


    // -------------------------------------------------------------------------
    // EoEGetIpMessage
    // -------------------------------------------------------------------------

    EoEGetIpMessage::EoEGetIpMessage(uint16_t mailbox_size, uint8_t port,
                                     EoE::IpParam* params, nanoseconds timeout)
        : AbstractMessage(mailbox_size, timeout)
        , params_(params)
    {
        auto* eoe = pointData<EoE::Header>(header_);

        header_->priority = 0;
        header_->channel  = 0;
        header_->type     = mailbox::Type::EoE;

        eoe->type            = EoE::request::GET_IP;
        eoe->port            = port;
        eoe->last_fragment   = 1;
        eoe->time_appended   = 0;
        eoe->time_request    = 0;
        eoe->fragment_number = 0;
        eoe->offset          = 0;
        eoe->frame_number    = 0;

        // GET IP request has no data payload - just the EoE header
        header_->len = static_cast<uint16_t>(sizeof(EoE::Header));
    }


    ProcessingResult EoEGetIpMessage::process(uint8_t const* received)
    {
        auto const* header = pointData<mailbox::Header>(received);

        if ((header->address & mailbox::GATEWAY_MESSAGE_MASK) != 0)
        {
            return ProcessingResult::NOOP;
        }

        if (header->type != mailbox::Type::EoE)
        {
            return ProcessingResult::NOOP;
        }

        auto const* eoe = pointData<EoE::Header>(header);

        if (eoe->type != EoE::response::GET_IP)
        {
            return ProcessingResult::NOOP;
        }

        // Check result code (bytes 2-3 of the EoE header)
        uint16_t result;
        std::memcpy(&result, reinterpret_cast<uint8_t const*>(eoe) + 2, sizeof(uint16_t));

        if (result != EoE::result::SUCCESS)
        {
            eoe_warning("EoE GET IP failed: %s (0x%04x)\n", EoE::result::toString(result), result);
            status_ = result;
            return ProcessingResult::FINALIZE;
        }

        uint16_t eoe_data_size = header->len - static_cast<uint16_t>(sizeof(EoE::Header));
        if (eoe_data_size < EOE_PARAM_DATA_OFFSET)
        {
            eoe_warning("EoE GET IP response too short (%d bytes)\n", eoe_data_size);
            status_ = EoE::result::UNSPECIFIED_ERROR;
            return ProcessingResult::FINALIZE;
        }

        auto const* data = pointData<uint8_t>(eoe);
        uint8_t  flags  = data[EOE_PARAM_FLAGS_OFFSET];
        uint16_t offset = EOE_PARAM_DATA_OFFSET;

        if ((flags & FLAG_MAC) && (offset + sizeof(EoE::MAC) <= eoe_data_size))
        {
            std::memcpy(params_->mac, data + offset, sizeof(EoE::MAC));
            params_->mac_set = true;
            offset += sizeof(EoE::MAC);
        }
        if ((flags & FLAG_IP) && (offset + sizeof(EoE::IP) <= eoe_data_size))
        {
            std::memcpy(params_->ip, data + offset, sizeof(EoE::IP));
            params_->ip_set = true;
            offset += sizeof(EoE::IP);
        }
        if ((flags & FLAG_SUBNET) && (offset + sizeof(EoE::SUBNET_MASK) <= eoe_data_size))
        {
            std::memcpy(params_->subnet, data + offset, sizeof(EoE::SUBNET_MASK));
            params_->subnet_set = true;
            offset += sizeof(EoE::SUBNET_MASK);
        }
        if ((flags & FLAG_GATEWAY) && (offset + sizeof(EoE::DEFAULT_GATEWAY) <= eoe_data_size))
        {
            std::memcpy(params_->gateway, data + offset, sizeof(EoE::DEFAULT_GATEWAY));
            params_->gateway_set = true;
            offset += sizeof(EoE::DEFAULT_GATEWAY);
        }
        if ((flags & FLAG_DNS_IP) && (offset + sizeof(EoE::DNS_SERVER_IP) <= eoe_data_size))
        {
            std::memcpy(params_->dns_ip, data + offset, sizeof(EoE::DNS_SERVER_IP));
            params_->dns_ip_set = true;
            offset += sizeof(EoE::DNS_SERVER_IP);
        }
        if ((flags & FLAG_DNS_NAME) && (offset + sizeof(EoE::DNS_NAME) <= eoe_data_size))
        {
            std::memcpy(params_->dns_name, data + offset, sizeof(EoE::DNS_NAME));
            params_->dns_name_set = true;
        }

        status_ = MessageStatus::SUCCESS;
        return ProcessingResult::FINALIZE;
    }


    // -------------------------------------------------------------------------
    // EoEFrameFragment
    // -------------------------------------------------------------------------

    EoEFrameFragment::EoEFrameFragment(uint16_t mailbox_size, uint8_t port,
                                       uint8_t fragment_no, uint8_t frame_no,
                                       uint16_t offset_32, bool is_last,
                                       uint8_t const* payload, uint16_t payload_size)
        : AbstractMessage(mailbox_size, 0ns)
    {
        auto* eoe  = pointData<EoE::Header>(header_);
        auto* data = pointData<uint8_t>(eoe);

        header_->priority = 0;
        header_->channel  = 0;
        header_->type     = mailbox::Type::EoE;
        header_->len      = static_cast<uint16_t>(sizeof(EoE::Header) + payload_size);

        eoe->type            = 0;  // FRAG_DATA
        eoe->port            = port;
        eoe->last_fragment   = is_last ? 1 : 0;
        eoe->time_appended   = 0;
        eoe->time_request    = 0;
        eoe->fragment_number = fragment_no & 0x3F;
        eoe->offset          = offset_32  & 0x3F;
        eoe->frame_number    = frame_no   & 0xF;

        std::memcpy(data, payload, payload_size);

        // No response expected: set SUCCESS so send() does not add to to_process
        status_ = MessageStatus::SUCCESS;
    }


    ProcessingResult EoEFrameFragment::process(uint8_t const* /*received*/)
    {
        return ProcessingResult::NOOP;
    }


    // -------------------------------------------------------------------------
    // EoEReceiveMessage
    // -------------------------------------------------------------------------

    EoEReceiveMessage::EoEReceiveMessage(Mailbox& mbx,
                                         std::function<void(uint8_t const*, uint16_t)> callback)
        : AbstractMessage(mbx.recv_size, 0ns)
        , callback_(std::move(callback))
    {
        frame_buffer_.resize(MAX_ETH_FRAME);
    }


    ProcessingResult EoEReceiveMessage::process(uint8_t const* received)
    {
        auto const* header = pointData<mailbox::Header>(received);

        if (header->type != mailbox::Type::EoE)
        {
            return ProcessingResult::NOOP;
        }

        auto const* eoe = pointData<EoE::Header>(header);

        if (eoe->type != 0)  // Not FRAG_DATA
        {
            return ProcessingResult::NOOP;
        }

        uint16_t frag_data_size = header->len - static_cast<uint16_t>(sizeof(EoE::Header));
        auto const* frag_data   = pointData<uint8_t>(eoe);

        uint8_t incoming_frag_no  = eoe->fragment_number;
        uint8_t incoming_frame_no = static_cast<uint8_t>(eoe->frame_number);

        // Validate fragment sequence
        if (rx_fragment_no_ != incoming_frag_no)
        {
            if (rx_fragment_no_ != 0)
            {
                // Mid-sequence mismatch: reset and attempt recovery
                rx_fragment_no_ = 0;
                rx_frame_size_   = 0;
                rx_frame_offset_ = 0;
                rx_frame_no_     = 0xFF;
            }

            if (incoming_frag_no != 0)
            {
                // Cannot start mid-sequence
                eoe_warning("EoE recv: unexpected fragment %d (expected 0) - dropping\n", incoming_frag_no);
                return ProcessingResult::FINALIZE_AND_KEEP;
            }
        }

        if (rx_fragment_no_ == 0)
        {
            // First fragment: offset field encodes total frame size in 32-byte units
            rx_frame_size_   = static_cast<uint16_t>(eoe->offset << 5);
            rx_frame_offset_ = 0;
            rx_frame_no_     = incoming_frame_no;

            if (rx_frame_size_ > static_cast<uint16_t>(frame_buffer_.size()))
            {
                eoe_warning("EoE recv: frame size %d exceeds buffer - dropping\n", rx_frame_size_);
                rx_fragment_no_ = 0;
                rx_frame_size_  = 0;
                return ProcessingResult::FINALIZE_AND_KEEP;
            }
        }
        else
        {
            // Continuation fragment: validate frame number and cumulative offset
            if (rx_frame_no_ != incoming_frame_no)
            {
                eoe_warning("EoE recv: frame number mismatch (expected %d, got %d) - dropping\n",
                            rx_frame_no_, incoming_frame_no);
                rx_fragment_no_ = 0; rx_frame_size_ = 0;
                rx_frame_offset_ = 0; rx_frame_no_ = 0xFF;
                return ProcessingResult::FINALIZE_AND_KEEP;
            }

            uint16_t expected_offset = static_cast<uint16_t>(eoe->offset << 5);
            if (rx_frame_offset_ != expected_offset)
            {
                eoe_warning("EoE recv: offset mismatch (expected %d, got %d) - dropping\n",
                            rx_frame_offset_, expected_offset);
                rx_fragment_no_ = 0; rx_frame_size_ = 0;
                rx_frame_offset_ = 0; rx_frame_no_ = 0xFF;
                return ProcessingResult::FINALIZE_AND_KEEP;
            }
        }

        // Copy fragment data into reassembly buffer
        if (rx_frame_offset_ + frag_data_size <= rx_frame_size_ &&
            rx_frame_offset_ + frag_data_size <= static_cast<uint16_t>(frame_buffer_.size()))
        {
            std::memcpy(frame_buffer_.data() + rx_frame_offset_, frag_data, frag_data_size);
            rx_frame_offset_ += frag_data_size;
            rx_fragment_no_++;
        }

        if (eoe->last_fragment)
        {
            uint16_t frame_size = rx_frame_offset_;

            // Strip appended timestamp (4 bytes) if present
            if (eoe->time_appended && frame_size >= 4)
            {
                frame_size -= 4;
            }

            callback_(frame_buffer_.data(), frame_size);

            // Reset reassembly state
            rx_fragment_no_ = 0;
            rx_frame_size_   = 0;
            rx_frame_offset_ = 0;
            rx_frame_no_     = 0xFF;
        }

        return ProcessingResult::FINALIZE_AND_KEEP;
    }
}
