#include "CenturyVSPump.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

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
        void CenturyVSPump::loop()
        {
            // Incoming data to process?
            if (!response_queue_.empty())
            {
                auto &message = response_queue_.front();
                if (message != nullptr)
                    process_modbus_data_(message.get());
                response_queue_.pop();
            }
            else
            {
                // all messages processed send pending commmands
                send_next_command_();
            }
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
        /// called when a modbus response was parsed without errors
        void CenturyVSPump::on_modbus_data(const std::vector<uint8_t> &data)
        {
            ESP_LOGV(TAG, "Pump got data");
            if (this->command_queue_.empty())
            {
                ESP_LOGW(TAG, "Modbus response received with no pending command");
                return;
            }

            auto &current_command = this->command_queue_.front();
            if (current_command != nullptr)
            {
                current_command->payload_ = data;
                this->response_queue_.push(std::move(current_command));
                ESP_LOGV(TAG, "Pump response queued");
                command_queue_.pop_front();
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        /// called when a modbus error response was received
        void CenturyVSPump::on_modbus_error(uint8_t function_code, uint8_t exception_code)
        {
            ESP_LOGV(TAG, "Received modbus error");
            if (this->command_queue_.empty())
            {
                ESP_LOGW(TAG, "Modbus error received with no pending command");
                return;
            }

            auto &current_command = this->command_queue_.front();
            if (current_command != nullptr)
            {
                ESP_LOGD(TAG, "Modbus error, so removing current command (%02X) from queue", current_command->function_);
                command_queue_.pop_front();
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        void CenturyVSPump::dump_config()
        {
            ESP_LOGCONFIG(TAG, "CenturyVSPump:");
            ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        bool CenturyVSPump::has_pending_command_(const CenturyPumpCommand &command) const
        {
            for (const auto &queued_command : command_queue_)
            {
                if (queued_command == nullptr)
                    continue;
                if (queued_command->function_ == command.function_ && queued_command->payload_ == command.payload_)
                    return true;
            }
            return false;
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
            // The same sensor/status request should not be queued repeatedly while a previous one is
            // still pending. Re-adding the same custom function/PDU simply keeps the Modbus hub busy
            // and causes the repeated "Frame already active" rejections seen in logs.
            if (this->has_pending_command_(command))
            {
                ESP_LOGV(TAG, "Skipping duplicate queued command %02X", command.function_);
                return;
            }
            command_queue_.push_back(make_unique<CenturyPumpCommand>(command));
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        void CenturyVSPump::process_modbus_data_(const CenturyPumpCommand *response)
{
    if (response == nullptr)
    {
        ESP_LOGW(TAG, "Null response received");
        return;
    }

    if (response->payload_.empty())
    {
        ESP_LOGW(TAG, "Empty payload for function %02X", response->function_);
        return;
    }

    // ESPHome 2026.7's modbus rewrite strips the function-code echo before
    // calling on_modbus_data() — the first byte is now the pump ACK. Some
    // older/newer modbus layers may still include the echoed function-code
    // as the first byte. Be tolerant: if the first byte equals the function
    // code, use the next byte as the ACK and drop two leading bytes.
    uint8_t ack = response->payload_[0];
    size_t data_start = 1;
    if (ack == response->function_)
    {
        if (response->payload_.size() < 2)
        {
            ESP_LOGW(TAG, "Function %02X unexpected echo-only payload", response->function_);
            return;
        }
        ack = response->payload_[1];
        data_start = 2;
        ESP_LOGD(TAG, "Detected echoed function code for %02X, using next byte as ACK", response->function_);
    }

    if (ack != 0x10)
    {
        ESP_LOGW(TAG, "Function %02X NACK with %02X, ignoring", response->function_, ack);
        return;
    }

    // Drop the ACK (and optional echoed function-code) byte(s) and pass the rest
    // to the command-specific handler.
    std::vector<uint8_t> data(response->payload_.begin() + data_start, response->payload_.end());
    response->on_data_func_(this, data);
}
        /////////////////////////////////////////////////////////////////////////////////////////////
        bool CenturyVSPump::send_next_command_()
        {
            uint32_t last_send = millis() - this->last_command_timestamp_;
            if ((last_send > this->command_throttle_) && this->ready_for_immediate_send() && !command_queue_.empty())
            {
                auto &command = command_queue_.front();

                if (command->send_countdown < 1)
                {
                    ESP_LOGD(TAG, "Pump command %02X no response received - removed from send queue", command->function_);
                    command_queue_.pop_front();
                }
                else
                {
                    ESP_LOGV(TAG, "Sending command with function %02X", command->function_);
                    command->send();
                    this->last_command_timestamp_ = millis();
                }
            }
            return true;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        bool CenturyPumpCommand::send()
        {
            std::vector<uint8_t> cmd;
            // `queue_pdu()` will prepend the device address for us; provide the PDU only.
            cmd.push_back(function_);
            cmd.push_back(0x20);
            cmd.insert(cmd.end(), payload_.begin(), payload_.end());
            pump_->queue_pdu(cmd);
            this->send_countdown--;
            return true;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        CenturyPumpCommand CenturyPumpCommand::create_status_command(CenturyVSPump *pump, std::function<void(CenturyVSPump *pump, bool running)> on_status_func)
        {
            CenturyPumpCommand cmd = {};
            cmd.pump_ = pump;
            cmd.function_ = 0x43; // Pump status
            cmd.on_data_func_ = [on_status_func](CenturyVSPump *pump, const std::vector<uint8_t> data)
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
            cmd.on_data_func_ = [page, address, scale, on_value_func](CenturyVSPump *pump, const std::vector<uint8_t> data)
            {
                if (data.size() < 3)
                {
                    ESP_LOGW(TAG, "Sensor response too short for page %d addr %d: %zu bytes", page, address, data.size());
                    return;
                }

                uint16_t value = static_cast<uint16_t>(data[2]);
                if (data.size() >= 4)
                {
                    value |= static_cast<uint16_t>(data[3]) << 8;
                }
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
            cmd.on_data_func_ = [on_confirmation_func](CenturyVSPump *pump, const std::vector<uint8_t> data)
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
            cmd.on_data_func_ = [on_confirmation_func](CenturyVSPump *pump, const std::vector<uint8_t> data)
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
            cmd.on_data_func_ = [on_confirmation_func](CenturyVSPump *pump, const std::vector<uint8_t> data)
            {
                ESP_LOGD(TAG, "Set demand comfirmed");
                on_confirmation_func(pump);
            };
            return cmd;
        }
    }
}
