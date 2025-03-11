#include <rclcpp/rclcpp.hpp>
#include <ethercat.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

class EtherCATNode : public rclcpp::Node {
public:
    explicit EtherCATNode() : rclcpp::Node("ethercat_node_slave1"), current_velocity(0), current_angle(0),
                              pos_x(0.0), pos_y(0.0), theta(0.0), timer_interval_ms(10) {
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

                // Setze initialen Zustand des Motors auf AUS
                writeSDO(slave_number_mcDSA_E4, 0x511C, 0x01, 0x01);

                ec_slave[slave_number_mcDSA_E4].state = EC_STATE_SAFE_OP;
                ec_writestate(slave_number_mcDSA_E4);
                if (ec_statecheck(slave_number_mcDSA_E4, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) == EC_STATE_SAFE_OP) {
                    RCLCPP_INFO(this->get_logger(), "Slave ist im SAFE-OP-Modus.");
                    usleep(10000);

                    timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(timer_interval_ms),
                        std::bind(&EtherCATNode::update, this));

                    // Erstelle Publisher für Odometrie
                    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
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
    int current_velocity;
    int current_angle;

    double pos_x, pos_y, theta;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    const int timer_interval_ms;

    void update() {
        // Lese aktuelle Werte
        readSDO(slave_number_mcDSA_E4, 0x511B, 0x01, current_angle);
        readSDO(slave_number_mcDSA_E4, 0x511B, 0x02, current_velocity);

        RCLCPP_INFO(this->get_logger(), "Gelesen -> Winkel: %d, Geschwindigkeit: %d", current_angle, current_velocity);

        // Berechne Odometrie
        double dt = timer_interval_ms / 1000.0;  // Zeitintervall in Sekunden
        double linear_velocity = (current_velocity / 60) * 0.145 * M_PI; // Geschwindigkeit in m/s
        double angular_velocity = ((current_angle-1024) / 4096.0) * 2 * M_PI;  // In Rad/s

        // Positionsupdate
        pos_x += linear_velocity * cos(theta) * dt;
        pos_y += linear_velocity * sin(theta) * dt;
        theta += angular_velocity * dt;

        // Sende Motorbefehl, wenn Geschwindigkeit > 0
        if (current_velocity > 0) {
            writeSDO(slave_number_mcDSA_E4, 0x511C, 0x01, 0x07);  // Motor an
        } else {
            writeSDO(slave_number_mcDSA_E4, 0x511C, 0x01, 0x01);  // Motor aus
        }

        // Veröffentliche Odometrie-Daten
        publishOdom(linear_velocity, angular_velocity);
    }

    // Funktion zum Veröffentlichen der Odometrie
    void publishOdom(double linear_velocity, double angular_velocity) {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = pos_x;
        odom_msg.pose.pose.position.y = pos_y;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = linear_velocity;
        odom_msg.twist.twist.angular.z = angular_velocity;

        odom_publisher_->publish(odom_msg);
    }

    // SDO-Lesen
    bool readSDO(uint16 slave, uint16 index, uint8 subindex, int32_t &value) {
        int size = sizeof(value);
        int result = ec_SDOread(slave, index, subindex, FALSE, &size, &value, EC_TIMEOUTSAFE);

        if (result > 0) {
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Lesen von Index 0x%X, Subindex 0x%X", index, subindex);
            return false;
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
