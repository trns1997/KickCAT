#include "Bus.h"
#include "EoE/mailbox/request.h"

namespace kickcat
{
    using namespace mailbox::request;

    void Bus::setEoEIp(Slave& slave, uint8_t port, EoE::IpParam const& params, nanoseconds timeout)
    {
        auto msg = slave.mailbox.createEoESetIp(port, params, timeout);
        waitForMessage(msg);
        if (msg->status() != MessageStatus::SUCCESS)
        {
            THROW_ERROR_CODE("EoE SET IP failed", error::category::EoE, msg->status());
        }
    }


    void Bus::getEoEIp(Slave& slave, uint8_t port, EoE::IpParam& params, nanoseconds timeout)
    {
        auto msg = slave.mailbox.createEoEGetIp(port, params, timeout);
        waitForMessage(msg);
        if (msg->status() != MessageStatus::SUCCESS)
        {
            THROW_ERROR_CODE("EoE GET IP failed", error::category::EoE, msg->status());
        }
    }


    void Bus::queueEoEFrame(Slave& slave, uint8_t port, uint8_t const* frame, uint16_t size)
    {
        slave.mailbox.queueEoEFrame(port, frame, size);
    }


    void Bus::enableEoEReceive(Slave& slave, std::function<void(uint8_t const*, uint16_t)> callback)
    {
        auto handler = std::make_shared<EoEReceiveMessage>(slave.mailbox, std::move(callback));
        slave.mailbox.to_process.push_back(handler);
    }
}
