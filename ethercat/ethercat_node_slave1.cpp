#include <rclcpp/rclcpp.hpp>
#include "ethercat.h"
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <geometry_msgs/msg/twist.hpp>  // Für ros2_tricycle_controller

class EtherCATNode : public rclcpp::Node {
public:
    explicit EtherCATNode() : rclcpp::Node("ethercat_node_slave1"), target_velocity(0), target_angle(0),
                              current_velocity(0), current_angle(0), motor_running(false), timer_interval_ms(10) {
        RCLCPP_INFO(this->get_logger(), "EtherCAT Node gestartet.");
        
        uint8 IOmap[4096];

        if (ec_init("eno1")) {
            RCLCPP_INFO(this->get_logger(), "EtherCAT-Initialisierung erfolgreich.");
            
            if (ec_config_init(FALSE) > 0) {
                RCLCPP_INFO(this->get_logger(), "%d Slaves gefunden.", ec_slavecount);

                if (ec_config_map(&IOmap) > 0) {
                    RCLCPP_INFO(this->get_logger(), "PDO-Mapping erfolgreich konfiguriert.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "PDO-Mapping fehlgeschlagen.");
                    return;
                }

                // Motor beim Start ausschalten (Sicherheit)
                writeSDO(slave_number_mcDSA_E4, 0x511C, 0x01, 0x01); // Motor AUS
                
                ec_slave[slave_number_mcDSA_E4].state = EC_STATE_SAFE_OP;
                ec_writestate(slave_number_mcDSA_E4);
                if (ec_statecheck(slave_number_mcDSA_E4, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) == EC_STATE_SAFE_OP) {
                    RCLCPP_INFO(this->get_logger(), "Slave ist im SAFE-OP-Modus.");
                    usleep(10000);
                    
                    timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(timer_interval_ms),
                        std::bind(&EtherCATNode::update, this));

                    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
                        "/tricycle_controller/cmd_vel", 10, std::bind(&EtherCATNode::cmdVelCallback, this, std::placeholders::_1));
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
    int slave_number_mcDSA_E4 = 1;
    int target_velocity;
    int target_angle;
    int current_velocity;
    int current_angle;
    bool motor_running;  // Motorstatus
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    const int timer_interval_ms;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Steuerung des Lenkwinkels (linear.x -> Geschwindigkeit, angular.z -> Winkel)
        target_velocity = static_cast<int>(((msg->linear.x  * 60)/(0.145 * M_PI)/198)*4000); // Geschwindigkeit in Prozent
        target_angle = static_cast<int>((msg->angular.z / (2 * M_PI)) * 4096); // Umrechnung von rad/s auf 0-4096
    }

    void update() {
        // Geschwindigkeit schrittweise anpassen
        if (current_velocity < target_velocity) {
            current_velocity = std::min(current_velocity + 30, target_velocity);
        } else if (current_velocity > target_velocity) {
            current_velocity = std::max(current_velocity - 30, target_velocity);
        }

        // Lenkwinkel schrittweise anpassen
        if (current_angle < target_angle) {
            current_angle = std::min(current_angle + 10, target_angle);
        } else if (current_angle > target_angle) {
            current_angle = std::max(current_angle - 10, target_angle);
        }


        // Motorstatus aktualisieren
        if ((current_velocity != 0 || current_angle != 0) && !motor_running) {
            startMotor();
        } else if (current_velocity == 0 && current_angle == 0 && motor_running) {
            stopMotor();
        }

        // SDO-Werte schreiben
        writeSDO(slave_number_mcDSA_E4, 0x511A, 0x01, current_angle+1024);
        writeSDO(slave_number_mcDSA_E4, 0x511A, 0x02, current_velocity);
        RCLCPP_INFO(this->get_logger(), "Sende -> Geschwindigkeit: %d, Winkel: %d", current_velocity, current_angle);
    }

    // Motor einschalten
    void startMotor() {
        if (writeSDO(slave_number_mcDSA_E4, 0x511C, 0x01, 0x07)) {
            motor_running = true;
            RCLCPP_INFO(this->get_logger(), "Motor eingeschaltet.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Einschalten des Motors.");
        }
    }

    // Motor ausschalten
    void stopMotor() {
        if (writeSDO(slave_number_mcDSA_E4, 0x511C, 0x01, 0x01)) {
            motor_running = false;
            RCLCPP_INFO(this->get_logger(), "Motor ausgeschaltet.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Ausschalten des Motors.");
        }
    }

    // SDO-Schreiben
    bool writeSDO(uint16 slave, uint16 index, uint8 subindex, int32_t value) {
        int result = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTSAFE);
        return result > 0;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EtherCATNode>());
    rclcpp::shutdown();
    return 0;
}
