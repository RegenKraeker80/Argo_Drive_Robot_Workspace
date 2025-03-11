#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class EtherCATLogicNode : public rclcpp::Node {
public:
    EtherCATLogicNode() : Node("ethercat_logic_node") {
        RCLCPP_INFO(this->get_logger(), "EtherCAT Logik-Node gestartet.");
        
        // Subscriber für EtherCAT Eingänge
        input_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/ethercat_in", 10,
            std::bind(&EtherCATLogicNode::process_inputs, this, std::placeholders::_1));
        
        // Publisher für EtherCAT Ausgänge
        output_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/ethercat_out", 10);
        
        // Timer für zyklisches Setzen des ersten Ausgangs
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&EtherCATLogicNode::set_output, this));
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr input_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr output_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void process_inputs(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        // Eingangsverarbeitung kann hier ergänzt werden
    }

    void set_output() {
        std_msgs::msg::Int32MultiArray output_msg;
        output_msg.data.push_back(3); // Slave 3
        output_msg.data.push_back(1); // Ausgang auf HIGH setzen
        output_pub_->publish(output_msg);
        RCLCPP_INFO(this->get_logger(), "Setze Ausgang 1 auf Slave 3HIGH");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EtherCATLogicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
