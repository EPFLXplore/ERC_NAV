#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <libserial/SerialStream.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <sstream>

using namespace LibSerial;

//TODO: add those commands to docker
// sudo apt-get install ros-humble-imu-tools
// sudo apt-get install libserial-dev

constexpr const char* SERIAL_PORT = "/dev/ttyACM1";

class ImuPublisher : public rclcpp::Node {
public:
    ImuPublisher() : Node("imu_publisher") {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
        imu_mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", rclcpp::SensorDataQoS());

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
            usleep(30000);  //30  microseconds
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

                sensor_msgs::msg::Imu imu_msg;
                rclcpp::Time time_now = this->get_clock()->now();
                imu_msg.header.stamp = time_now;
                imu_msg.header.frame_id = "imu_link";

                imu_msg.linear_acceleration.x = (-1.0) * acc_x;
                imu_msg.linear_acceleration.y = (-1.0) * acc_y;
                imu_msg.linear_acceleration.z = acc_z;

                //convert from dps to rad/s
                double dps_to_rads = M_PI / 180.0;
                imu_msg.angular_velocity.x = gyr_x * dps_to_rads;
                imu_msg.angular_velocity.y = (-1.0) * gyr_y * dps_to_rads;
                imu_msg.angular_velocity.z = (-1.0) * gyr_z * dps_to_rads;

                //imu orientation in the form of quaternions
                //need Magdwick filter to get orientation
                sensor_msgs::msg::MagneticField imu_mag_msg;
                imu_mag_msg.header.stamp = time_now;
                imu_mag_msg.header.frame_id = "imu_link";

                imu_mag_msg.magnetic_field.x = mag_y;
                imu_mag_msg.magnetic_field.y = mag_x;
                imu_mag_msg.magnetic_field.z = (-1.0) * mag_z;


                imu_mag_pub_->publish(imu_mag_msg);
                imu_pub_->publish(imu_msg);
            } else {
                // If no data is available, wait before retrying
                usleep(1000); //in microseconds
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr imu_mag_pub_;
    SerialStream serial_stream_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
