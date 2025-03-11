#include "rclcpp/rclcpp.hpp"
#include "ethercat.h"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <vector>
#include <bitset>

class EtherCATNode : public rclcpp::Node {
public:
    EtherCATNode() : Node("ethercat_node_slave2_10"), timer_interval_ms(10) {
        RCLCPP_INFO(this->get_logger(), "EtherCAT Node gestartet.");

        uint8 IOmap[4096];

        

        if (ec_init("enp1s0")) {
            RCLCPP_INFO(this->get_logger(), "EtherCAT-Initialisierung erfolgreich.");

            if (ec_config_init(FALSE) > 0) {
                RCLCPP_INFO(this->get_logger(), "%d Slaves gefunden.", ec_slavecount);

                if (ec_config_map(&IOmap) > 0) {
                    RCLCPP_INFO(this->get_logger(), "PDO-Mapping erfolgreich konfiguriert.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "PDO-Mapping fehlgeschlagen.");
                    return;
                }
                
                // Manuelles PDO-Mapping für EL2008
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
                
                ec_slave[2].state = EC_STATE_SAFE_OP;
                ec_writestate(2);
                if (ec_statecheck(2, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) == EC_STATE_SAFE_OP) {
                    RCLCPP_INFO(this->get_logger(), "Slave ist im SAFE-OP-Modus.");
                    usleep(10000);
                    
                    ec_slave[2].state = EC_STATE_OPERATIONAL;
                    ec_writestate(2);
                    usleep(5000);
                    if (ec_statecheck(2, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) == EC_STATE_OPERATIONAL) {
                        RCLCPP_INFO(this->get_logger(), "Slave ist im OPERATIONAL-Modus.");

                        output_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
                            "/ethercat_out", 10,
                            std::bind(&EtherCATNode::handle_output, this, std::placeholders::_1));

                        input_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/ethercat_in", 10);
                        
                        timer_ = this->create_wall_timer(
                            std::chrono::milliseconds(timer_interval_ms),
                            std::bind(&EtherCATNode::update, this));
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Slave konnte nicht in den OP-Modus wechseln.");
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Slave konnte nicht in den SAFE-OP-Modus wechseln.");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Keine Slaves gefunden.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "EtherCAT-Initialisierung fehlgeschlagen.");
        }
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr input_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr output_sub_;
    const int timer_interval_ms;

    void handle_output(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Ungültige Nachricht für EtherCAT-Ausgänge!");
            return;
        }

        uint16 slave = msg->data[0];
        uint8 output_index = msg->data[1];  // Welcher Ausgang gesetzt werden soll
        bool state = msg->data[2];  // 1 = HIGH, 0 = LOW

        if (slave < 1 || slave > ec_slavecount) {
            RCLCPP_ERROR(this->get_logger(), "Fehler: Ungültiger Slave %d", slave);
            return;
        }

        if (ec_slave[slave].outputs == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Fehler: Slave %d hat keine Outputs!", slave);
            return;
        }

        uint8 *data_ptr = (uint8 *)ec_slave[slave].outputs;

        if (state) {
            *data_ptr |= (1 << output_index);  // Setzt das gewünschte Bit auf 1
        } else {
            *data_ptr &= ~(1 << output_index); // Setzt das gewünschte Bit auf 0
        }

        RCLCPP_INFO(this->get_logger(), "Slave %d - Ausgang %d auf %d gesetzt", slave, output_index, state);
    }


    void update() {
        std_msgs::msg::Int32MultiArray msg;
        for (int i = 2; i <= ec_slavecount; ++i) {
            if (ec_slave[i].inputs != nullptr) {
                msg.data.push_back(*(uint8 *)ec_slave[i].inputs);
            } else {
                msg.data.push_back(0);
                RCLCPP_WARN(this->get_logger(), "Slave %d hat keine Eingänge!", i);
            }
        }
        input_pub_->publish(msg);
    }

    void setDigitalOutput(uint16 slave, uint8 output_index, bool state) {
        if (ec_slave[slave].outputs == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Fehler: Slave %d hat keine Outputs!", slave);
            return;
        }

        uint8 *data_ptr = (uint8 *)ec_slave[slave].outputs;
        uint8 current_value = *data_ptr;
        if (state) {
            current_value |= (1 << output_index);
        } else {
            current_value &= ~(1 << output_index);
        }
        *data_ptr = current_value;

        RCLCPP_INFO(this->get_logger(), "Slave %d - Ausgang %d auf %d gesetzt", slave, output_index, state);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EtherCATNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
