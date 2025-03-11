#include "rclcpp/rclcpp.hpp"
#include "ethercat.h"
#include <iostream>
#include <iomanip>

class EtherCATScanner : public rclcpp::Node {
public:
    EtherCATScanner() : Node("ethercat_scan") {
        RCLCPP_INFO(this->get_logger(), "Starte EtherCAT Scan...");

        if (ec_init("enp1s0")) {
            RCLCPP_INFO(this->get_logger(), "EtherCAT-Initialisierung erfolgreich.");

            if (ec_config_init(FALSE) > 0) {
                RCLCPP_INFO(this->get_logger(), "%d Slaves gefunden.", ec_slavecount);
                
                for (int i = 1; i <= ec_slavecount; ++i) {
                    if (strncmp(ec_slave[i].name, "EL2008", 6) == 0) {
                        RCLCPP_INFO(this->get_logger(), "Setze PDO-Mapping für EL2008 (Slave %d)", i);
                        
                        if (ec_slave[i].Obits == 0) {
                            RCLCPP_WARN(this->get_logger(), "EL2008 Slave %d hat keine Output-Bits! Setze lokales Mapping...", i);
                            ec_slave[i].Obits = 8;  // Erzwinge 8 Output-Bits
                            ec_slave[i].outputs = (uint8_t*) malloc(1); // Speicher für 8 Bits (1 Byte)
                            memset(ec_slave[i].outputs, 0, 1);
                        }
                    }
                }

                for (int i = 1; i <= ec_slavecount; ++i) {
                    printSlaveInfo(i);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Keine Slaves gefunden.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "EtherCAT-Initialisierung fehlgeschlagen.");
        }
    }

private:
    void printSlaveInfo(int slave) {
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Slave " << slave << " - Name: " << ec_slave[slave].name << std::endl;
        std::cout << "State: " << ec_slave[slave].state << std::endl;
        std::cout << "Output Size: " << ec_slave[slave].Obits << " bits" << std::endl;
        std::cout << "Input Size: " << ec_slave[slave].Ibits << " bits" << std::endl;
        std::cout << "Has DC: " << (ec_slave[slave].hasdc ? "Yes" : "No") << std::endl;
        std::cout << "Configured Address: " << std::hex << ec_slave[slave].configadr << std::dec << std::endl;
        // Entfernt: std::cout << "Ebus Current: " << ec_slave[slave].Icurrent << " mA" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EtherCATScanner>();
    rclcpp::shutdown();
    return 0;
}
