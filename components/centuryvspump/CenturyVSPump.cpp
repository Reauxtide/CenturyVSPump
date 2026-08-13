#include "CenturyVSPump.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome
{
    namespace century_vs_pump
    {
        //////////////////////////////////////////////////////////////////////////////////////////////
        //
        // CenturyVSPump implementation
        //
        //////////////////////////////////////////////////////////////////////////////////////////////

        static const char *const TAG = "century_vs_pump";

 *      ////////////////////////////*//////////////////////////////////*//////////////////////////////
   *    void CenturyVSPump::setup()
  *     {
#ifdef MODBUS_ENABLE_SWITCH*            enabled_switch_ = new *enturyPumpEnabledSwitch();
       *    enabled_switch_->set_name(name* + " MODBUS enabled");
           *App.register_switch(enabled_switch*);
#endif
        }

        /////*//////////////////////////////////*//////////////////////////////////*//////////////////
        void Ce*turyVSPump::loop()
        {
     *      // Incoming data to process?*            if (!response_queue_.e*pty())
            {
             *  auto &message = response_queue_.*ront();

                if (messa*e != nullptr)
                {
  *                 process_modbus_da*a_(message.get());
               *}

                response_queue_*pop();
            }
            e*se
            {
                /* All messages processed, send pend*ng commands.
                send_*ext_command_();
            }
    *   }

        ////////////////////*//////////////////////////////////*//////////////////////////////////*///
        void CenturyVSPump::up*ate()
        {
            // Req*est status and pump RPM.
#ifdef MO*BUS_ENABLE_SWITCH
            if (*nabled_switch_ == nullptr)
       *    {
                return;
    *       }

            if (enabled_*witch_->state == 0)
            {
*               return;
           *}
#endif

            ESP_LOGV(TAG* "Updating pump component");

    *       for (auto item : items_)
  *         {
                queue_c*mmand_(item->create_command());
  *         }
        }

        ////*//////////////////////////////////*//////////////////////////////////*///////////////////
        /// Ca*led when a Modbus response was par*ed without errors.
        void Ce*turyVSPump::on_modbus_data(
      *     const std::vector<uint8_t> &d*ta)
        {
            ESP_LOGV*TAG, "Pump got data");

          * if (command_queue_.empty())
     *      {
                ESP_LOGW(
*                   TAG,
          *         "Received pump data with *o command waiting");
             *  return;
            }

         *  auto &current_command = this->co*mand_queue_.front();

            *f (current_command != nullptr)
   *        {
                current_*ommand->payload_ = data;

        *       this->response_queue_.push(*                    std::move(curr*nt_command));

                ESP*LOGV(TAG, "Pump response queued");*
                command_queue_.po*_front();
            }
        }
*        //////////////////////////*//////////////////////////////////*////////////////////////////////
 *      /// Called when a Modbus err*r response was received.
        v*id CenturyVSPump::on_modbus_error(*            uint8_t function_code,*            uint8_t exception_code*
        {
            ESP_LOGV(
 *              TAG,
               *"Received Modbus error for functio* %02X, exception %02X",
          *     function_code,
              * exception_code);

            if *command_queue_.empty())
          * {
                ESP_LOGW(
     *              TAG,
               *    "Received Modbus error with no*command waiting");
               *return;
            }

           *auto &current_command = this->comm*nd_queue_.front();

            if*(current_command != nullptr)
     *      {
                ESP_LOGD(
*                   TAG,
          *         "Modbus error, removing c*rrent command (%02X) from queue",
*                   current_command*>function_);

                comm*nd_queue_.pop_front();
           *}
        }

        /////////////*//////////////////////////////////*//////////////////////////////////*//////////
        void CenturyVSP*mp::dump_config()
        {
      *     ESP_LOGCONFIG(TAG, "CenturyVS*ump:");
            ESP_LOGCONFIG(*                TAG,
             *  "  Address: 0x%02X",
           *
