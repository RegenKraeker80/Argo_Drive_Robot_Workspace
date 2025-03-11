#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include "ethercat.h"  // EtherCAT-Funktionen einbinden
#include "pluginlib/class_list_macros.hpp"

namespace argo_drive_hardware {
using namespace hardware_interface;

class ArgoDriveHardware : public hardware_interface::SystemInterface
{
public:
    CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
        RCLCPP_INFO(rclcpp::get_logger("ArgoDriveHardware"), "Hardware-Aktivierung wird übersprungen.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override
    {
        RCLCPP_WARN(rclcpp::get_logger("ArgoDriveHardware"), "Deaktiviere Hardware...");

        // Deaktiviere Motorsteuerung (z.B. Drive-Enable Bit löschen)
        int32_t disable_drive = 0;
        if (!writeSDO(1, 0x511C, 0x01, 0x01)) {
            RCLCPP_ERROR(rclcpp::get_logger("ArgoDriveHardware"), "Fehler: Konnte Antrieb nicht deaktivieren!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_WARN(rclcpp::get_logger("ArgoDriveHardware"), "Hardware erfolgreich deaktiviert.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override
    {
        RCLCPP_INFO(rclcpp::get_logger("ArgoDriveHardware"), "Initializing Argo Drive Hardware Interface");

        // Initialwerte setzen
        speed_ = 0.0;
        steering_angle_ = M_PI/2;
        speed_command_ = 0.0;
        steering_command_ = 0.0;

        // EtherCAT-Initialisierung
        if (ec_init("eno1")) {
            RCLCPP_INFO(rclcpp::get_logger("ArgoDriveHardware"), "EtherCAT erfolgreich initialisiert.");
            if (ec_config_init(FALSE) > 0) {
                RCLCPP_INFO(rclcpp::get_logger("ArgoDriveHardware"), "%d Slaves gefunden.", ec_slavecount);
                uint8 IOmap[4096];
                if (ec_config_map(&IOmap) > 0) {
                    RCLCPP_INFO(rclcpp::get_logger("ArgoDriveHardware"), "PDO-Mapping erfolgreich.");
                }
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("ArgoDriveHardware"), "EtherCAT-Initialisierung fehlgeschlagen!");
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back("traction_joint", hardware_interface::HW_IF_VELOCITY, &speed_);  // Geschwindigkeit (m/s)
        state_interfaces.emplace_back("steering_joint", hardware_interface::HW_IF_POSITION, &steering_angle_);  // Lenkwinkel (rad)
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back("traction_joint", hardware_interface::HW_IF_VELOCITY, &speed_command_);  // Antriebsbefehl (m/s)
        command_interfaces.emplace_back("steering_joint", hardware_interface::HW_IF_POSITION, &steering_command_);  // Lenkwinkelbefehl (rad)
        return command_interfaces;
    }

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
    {
        // Lese Daten von EtherCAT (gleiche Register wie in deiner EtherCATNode)
        readSDO(1, 0x511B, 0x02, current_velocity_);  // Geschwindigkeit (m/s)
        readSDO(1, 0x511B, 0x01, current_angle_);  // Lenkwinkel (rad)

        // Umrechnung falls nötig
        speed_ = (current_velocity_ / 60) * 0.145 * M_PI;  // Umwandlung in m/s
        steering_angle_ = ((current_angle_-1024) / 4096.0) * 2 * M_PI;  // Umwandlung in Radiant

        RCLCPP_INFO(rclcpp::get_logger("ArgoDriveHardware"), "READ: Speed = %f m/s, Steering Angle = %f rad", speed_, steering_angle_);
        return return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
    {
        // Setze Befehle an EtherCAT-Node
        int velocity_cmd = static_cast<int32_t>((speed_command_ * 60)/(0.145 * M_PI));  // m/s → Integer-Wert
        int steering_cmd = static_cast<int32_t>((steering_command_ / (2 * M_PI)) * 4096 + 1024);  // Rad → Registerwert

        writeSDO(1, 0x511C, 0x01, 0x04);
        writeSDO(1, 0x511A, 0x02, velocity_cmd);  // Geschwindigkeit senden
        writeSDO(1, 0x511A, 0x01, steering_cmd);  // Lenkwinkel senden

        RCLCPP_INFO(rclcpp::get_logger("ArgoDriveHardware"), "WRITE: Speed Cmd = %f m/s, Steering Cmd = %f rad", speed_command_, steering_command_);
        return return_type::OK;
    }

private:
    double speed_ = 0.0;
    double steering_angle_ = 0.0;
    double speed_command_ = 0.0;
    double steering_command_ = 0.0;

    int current_velocity_ = 0;
    int current_angle_ = 0;

    // EtherCAT Kommunikation
    bool readSDO(uint16 slave, uint16 index, uint8 subindex, int32_t &value) {
        int size = sizeof(value);
        int result = ec_SDOread(slave, index, subindex, FALSE, &size, &value, EC_TIMEOUTSAFE);

        if (result > 0) {
            return true;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("ArgoDriveHardware"), "Fehler beim Lesen von Index 0x%X, Subindex 0x%X", index, subindex);
            return false;
        }
    }

    bool writeSDO(uint16 slave, uint16 index, uint8 subindex, int32_t value) {
        int result = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTSAFE);
        if (result > 0) {
            return true;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("ArgoDriveHardware"), "Fehler beim Schreiben von Index 0x%X, Subindex 0x%X", index, subindex);
            return false;
        }
    }
};

}
PLUGINLIB_EXPORT_CLASS(argo_drive_hardware::ArgoDriveHardware, hardware_interface::SystemInterface)