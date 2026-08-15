#include "CenturyVSPump.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <memory>

namespace esphome
{
    namespace century_vs_pump
    {

        //////////////////////////////////////////////////////////////////////////////////////////////
        //
        //  CenturyVSPump implementation
        //
        /////////////////////////////////////////////////////////////////////////////////////////////

        static const char *const TAG = "century_vs_pump";

        /////////////////////////////////////////////////////////////////////////////////////////////
        void CenturyVSPump::setup()
        {
#ifdef MODBUS_ENABLE_SWITCH
            enabled_switch_ = new CenturyPumpEnabledSwitch();
            enabled_switch_->set_name(name_ + " MODBUS enabled");
            App.register_switch(enabled_switch_);
#endif
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        void CenturyVSPump::update()
        {
            // Request status & pump RPM
#ifdef MODBUS_ENABLE_SWITCH
            if (enabled_switch_ == nullptr)
                return;
            if (enabled_switch_->state == 0)
                return;
#endif
            ESP_LOGV(TAG, "Updating pump component");
            for (auto item : items_)
                queue_command_(item->create_command());
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        void CenturyVSPump::dump_config()
        {
            ESP_LOGCONFIG(TAG, "CenturyVSPump:");
            ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        void CenturyVSPump::queue_command_(const CenturyPumpCommand &command)
        {
#ifdef MODBUS_ENABLE_SWITCH
            if (enabled_switch_ == nullptr)
                return;
            if (enabled_switch_->state == 0)
                return;
#endif
            auto pending = std::make_unique<CenturyPumpCommand>(command);
            const std::vector<uint8_t> pdu = pending->build_pdu();

            // queue_pdu() returns false only when the request never entered the machine at all
            // (oversize PDU, full queue, over-cap duplicate) - no callback will follow, so don't track it.
            if (!this->queue_pdu(pdu))
            {
                ESP_LOGD(TAG, "Command %02X not accepted by modbus hub, dropping", pending->function_);
                return;
            }

            ESP_LOGV(TAG, "Queued command with function %02X", pending->function_);
            this->pending_.push_back(std::move(pending));
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        /// Match a callback back to the command that produced it. The hub hands us the exact request
        /// PDU we queued, so compare the whole thing - function code alone is ambiguous, since the
        /// RPM and demand sensors both send function 0x45 with different page/address payloads.
        std::vector<std::unique_ptr<CenturyPumpCommand>>::iterator
        CenturyVSPump::find_pending_(std::span<const uint8_t> request_pdu)
        {
            for (auto it = this->pending_.begin(); it != this->pending_.end(); ++it)
            {
                const std::vector<uint8_t> pdu = (*it)->build_pdu();
                if (pdu.size() == request_pdu.size() && std::equal(pdu.begin(), pdu.end(), request_pdu.begin()))
                    return it;
            }
            return this->pending_.end();
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        /// Every Century function code (0x41-0x45) is in the user-defined range, so all of this pump's
        /// traffic arrives here. response_pdu is the whole PDU: [function][ACK][data...].
        void CenturyVSPump::on_custom_response(std::span<const uint8_t> request_pdu,
                                               std::span<const uint8_t> response_pdu,
                                               modbus::ResponseStatus status)
        {
            auto it = this->find_pending_(request_pdu);
            if (it == this->pending_.end())
            {
                ESP_LOGW(TAG, "Response for an untracked request, ignoring");
                return;
            }

            std::unique_ptr<CenturyPumpCommand> command = std::move(*it);
            this->pending_.erase(it);

            // Modbus-level failure: response_pdu is empty and the exception code is in status.
            if (!modbus::succeeded(status))
            {
                ESP_LOGW(TAG, "Function %02X returned modbus exception %02X", command->function_,
                         static_cast<uint8_t>(status.value()));
                return;
            }

            if (response_pdu.size() < 2)
            {
                ESP_LOGW(TAG, "Short response for function %02X, ignoring", command->function_);
                return;
            }

            // Application-level ACK from the pump itself (not a modbus exception).
            if (response_pdu[1] != CENTURY_ACK)
            {
                ESP_LOGW(TAG, "Function %02X NACK with %02X, ignoring", command->function_, response_pdu[1]);
                return;
            }

            // Drop the function code and ACK byte; hand the rest to the command-specific handler.
            std::vector<uint8_t> data(response_pdu.begin() + 2, response_pdu.end());
            command->on_data_func_(this, data);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        /// No matching reply arrived. Returning true asks the hub to re-queue the same frame; the hub
        /// does not bound retries, so the countdown here is what stops them.
        bool CenturyVSPump::on_no_response(std::span<const uint8_t> request_pdu)
        {
            auto it = this->find_pending_(request_pdu);
            if (it == this->pending_.end())
                return false;

            if ((*it)->send_countdown > 1)
            {
                (*it)->send_countdown--;
                ESP_LOGD(TAG, "Pump command %02X no response - retrying", (*it)->function_);
                return true;
            }

            ESP_LOGD(TAG, "Pump command %02X no response received - removed from send queue", (*it)->function_);
            this->pending_.erase(it);
            return false;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        /// Accepted, then dropped before it ever reached the wire.
        void CenturyVSPump::on_not_sent(std::span<const uint8_t> request_pdu)
        {
            auto it = this->find_pending_(request_pdu);
            if (it == this->pending_.end())
                return;

            ESP_LOGD(TAG, "Pump command %02X was never sent - removed from send queue", (*it)->function_);
            this->pending_.erase(it);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        std::vector<uint8_t> CenturyPumpCommand::build_pdu() const
        {
            std::vector<uint8_t> pdu;
            pdu.reserve(2 + this->payload_.size());
            pdu.push_back(this->function_);
            pdu.push_back(this->ack_);
            pdu.insert(pdu.end(), this->payload_.begin(), this->payload_.end());
            return pdu;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        CenturyPumpCommand CenturyPumpCommand::create_status_command(CenturyVSPump *pump, std::function<void(CenturyVSPump *pump, bool running)> on_status_func)
        {
            CenturyPumpCommand cmd = {};
            cmd.pump_ = pump;
            cmd.function_ = 0x43; // Pump status
            cmd.on_data_func_ = [=](CenturyVSPump *pump, const std::vector<uint8_t> data)
            {
                ESP_LOGD(TAG, "Got status command reply %02X", data[0]);

                if (data[0] == 0x00)
                    on_status_func(pump, false);
                else if (data[0] == 0x0B)
                    on_status_func(pump, true);
            };
            return cmd;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        CenturyPumpCommand CenturyPumpCommand::create_read_sensor_command(CenturyVSPump *pump, uint8_t page, uint8_t address, uint16_t scale, std::function<void(CenturyVSPump *pump, uint16_t value)> on_value_func)
        {
            CenturyPumpCommand cmd = {};
            cmd.pump_ = pump;
            cmd.function_ = 0x45; // Read sensor
            cmd.payload_.push_back(page);
            cmd.payload_.push_back(address);
            cmd.on_data_func_ = [=](CenturyVSPump *pump, const std::vector<uint8_t> data)
            {
                // Always going to have at least 1 byte of sensor data
                uint16_t value = (uint16_t)data[2];
                if (data.size() == 4)
                {
                    // But sometimes, we get two bytes
                    value |= (uint16_t)data[3] << 8;
                }
                // Scale the value
                value /= scale;
                ESP_LOGD(TAG, "Read value %d from page %d, addr %d", value, page, address);
                on_value_func(pump, value);
            };
            return cmd;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        CenturyPumpCommand CenturyPumpCommand::create_run_command(CenturyVSPump *pump, std::function<void(CenturyVSPump *pump)> on_confirmation_func)
        {
            CenturyPumpCommand cmd = {};
            cmd.pump_ = pump;
            cmd.function_ = 0x41; // Go
            cmd.on_data_func_ = [=](CenturyVSPump *pump, const std::vector<uint8_t> data)
            {
                ESP_LOGD(TAG, "Confirmed pump running");
                on_confirmation_func(pump);
            };
            return cmd;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        CenturyPumpCommand CenturyPumpCommand::create_stop_command(CenturyVSPump *pump, std::function<void(CenturyVSPump *pump)> on_confirmation_func)
        {
            CenturyPumpCommand cmd = {};
            cmd.pump_ = pump;
            cmd.function_ = 0x42; // Stop
            cmd.on_data_func_ = [=](CenturyVSPump *pump, const std::vector<uint8_t> data)
            {
                ESP_LOGD(TAG, "Confirmed pump stopped");
                on_confirmation_func(pump);
            };
            return cmd;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        CenturyPumpCommand CenturyPumpCommand::create_set_demand_command(CenturyVSPump *pump, uint16_t demand, std::function<void(CenturyVSPump *pump)> on_confirmation_func)
        {
            CenturyPumpCommand cmd = {};
            cmd.pump_ = pump;
            cmd.function_ = 0x44;      // Set demand
            cmd.payload_.push_back(0); // Mode (0=Speed, 1=Torque, 2=Reserved, 3=Reserved)
            demand *= 4;               // Scaling
            cmd.payload_.push_back(demand & 0xff);
            cmd.payload_.push_back(demand >> 8);
            cmd.on_data_func_ = [=](CenturyVSPump *pump, const std::vector<uint8_t> data)
            {
                ESP_LOGD(TAG, "Set demand comfirmed");
                on_confirmation_func(pump);
            };
            return cmd;
        }
    }
}
