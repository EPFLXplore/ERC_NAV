#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <libserial/SerialStream.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>

using namespace LibSerial;

constexpr const char* SERIAL_PORT = "/dev/ttyACM0";

class ImuPublisher : public rclcpp::Node {
public:
    ImuPublisher() : Node("imu_publisher") {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data_raw", 10);

        try {
            serial_stream_.Open(SERIAL_PORT);
        }
        catch (const OpenFailed&) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the IMU's serial port.");
            rclcpp::shutdown();
            return;
        }

        serial_stream_.SetBaudRate(BaudRate::BAUD_115200);
        serial_stream_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serial_stream_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        serial_stream_.SetParity(Parity::PARITY_NONE);
        serial_stream_.SetStopBits(StopBits::STOP_BITS_1);

        // Wait for data to become available on the serial port
        while (serial_stream_.rdbuf()->in_avail() == 0) {
            usleep(30000);  // microseconds
        }

        mainLoop();
    }

private:
    void mainLoop() {
        while (rclcpp::ok()) {
            if (serial_stream_.IsDataAvailable()) {
                std::string data;
                std::getline(serial_stream_, data);

                std::istringstream iss(data);
                //std::cout << data << std::endl;
                double acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, mag_x, mag_y, mag_z;

                iss >> acc_x >> acc_y >> acc_z >> gyr_x >> gyr_y >> gyr_z >> mag_x >> mag_y >> mag_z;

                auto imu_msg = sensor_msgs::msg::Imu();
                imu_msg.header.stamp = this->get_clock()->now();
                imu_msg.header.frame_id = "imu_link";

                imu_msg.linear_acceleration.x = acc_x;
                imu_msg.linear_acceleration.y = acc_y;
                imu_msg.linear_acceleration.z = acc_z;

                //convert from dps to rad/s
                imu_msg.angular_velocity.x = gyr_x * M_PI / 180.0;
                imu_msg.angular_velocity.y = gyr_y * M_PI / 180.0;
                imu_msg.angular_velocity.z = gyr_z * M_PI / 180.0;

                imu_pub_->publish(imu_msg);
            } else {
                // If no data is available, wait before retrying
                usleep(1000); //in microseconds
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    SerialStream serial_stream_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
