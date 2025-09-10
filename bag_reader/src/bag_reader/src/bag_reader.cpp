#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <iomanip>

namespace fs = std::filesystem;

class RosbagParser {
public:
    RosbagParser(const std::string& bag_path, const std::string& output_dir)
        : bag_path_(bag_path), output_dir_(output_dir) {
        
        // 创建输出目录
        fs::create_directories(output_dir_);
        fs::create_directories(output_dir_ + "/images");
        fs::create_directories(output_dir_ + "/pointclouds");
        fs::create_directories(output_dir_ + "/odometry");
        fs::create_directories(output_dir_ + "/imu");
    }

    void parse() {
        rosbag2_cpp::Reader reader;
        try {
            reader.open(bag_path_);
            
            // 获取所有话题信息
            auto topics = reader.get_all_topics_and_types();
            std::cout << "Available topics:" << std::endl;
            for (const auto& topic : topics) {
                std::cout << "  " << topic.name << " [" << topic.type << "]" << std::endl;
            }

            // 处理每个消息
            while (reader.has_next()) {
                auto message = reader.read_next();
                processMessage(message);
            }

            reader.close();

        } catch (const std::exception& e) {
            std::cerr << "Error reading bag file: " << e.what() << std::endl;
        }
    }

private:
    void processMessage(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) {
        const std::string& topic_name = message->topic_name;
        
        if (topic_name.find("surround_view") != std::string::npos) {
            processCompressedImage(message);
        }
        else if (topic_name.find("imu") != std::string::npos) {
            processImu(message);
        }
        else if (topic_name.find("odom") != std::string::npos) {
            processOdometry(message);
        }
        // 可以添加其他话题的处理逻辑
    }

    void processCompressedImage(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) {
        try {
            rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*message->serialized_data);
            sensor_msgs::msg::CompressedImage::SharedPtr img_msg = 
                std::make_shared<sensor_msgs::msg::CompressedImage>();
            
            serialization.deserialize_message(&extracted_serialized_msg, img_msg.get());

            // 转换为OpenCV格式
            cv::Mat image = cv::imdecode(cv::Mat(img_msg->data), cv::IMREAD_COLOR);
            
            if (image.empty()) {
                std::cerr << "Failed to decode image from topic: " << message->topic_name << std::endl;
                return;
            }

            // 生成文件名（使用时间戳）
            auto nanoseconds = std::chrono::nanoseconds(message->time_stamp);
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(nanoseconds);
            auto fractional = nanoseconds - seconds;

            std::string topic_dir = message->topic_name.substr(1);
            size_t slash_pos = topic_dir.find('/');
            if (slash_pos != std::string::npos) {
                topic_dir = topic_dir.substr(0, slash_pos);
            }
            std::string image_dir = output_dir_ + "/images/" + topic_dir;
            fs::create_directories(image_dir);
            
            std::stringstream filename;
            filename << output_dir_ << "/images/"
                    << message->topic_name.substr(1) << "_"  // 移除开头的'/'
                    << seconds.count() << "_"
                    << std::setw(9) << std::setfill('0') << fractional.count()
                    << ".jpg";

            // 保存图片
            if (cv::imwrite(filename.str(), image)) {
                std::cout << "Saved image: " << filename.str() << std::endl;
            } else {
                std::cerr << "Failed to save image: " << filename.str() << std::endl;
            }

        } catch (const std::exception& e) {
            std::cerr << "Error processing image: " << e.what() << std::endl;
        }
    }

    void processImu(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) {
        try {
            rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*message->serialized_data);
            sensor_msgs::msg::Imu::SharedPtr imu_msg = 
                std::make_shared<sensor_msgs::msg::Imu>();
            
            serialization.deserialize_message(&extracted_serialized_msg, imu_msg.get());

            // 可以在这里保存IMU数据到文件
            // 例如：保存为CSV格式

        } catch (const std::exception& e) {
            std::cerr << "Error processing IMU: " << e.what() << std::endl;
        }
    }

    void processOdometry(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) {
        try {
            rclcpp::Serialization<nav_msgs::msg::Odometry> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*message->serialized_data);
            nav_msgs::msg::Odometry::SharedPtr odom_msg = 
                std::make_shared<nav_msgs::msg::Odometry>();
            
            serialization.deserialize_message(&extracted_serialized_msg, odom_msg.get());

            // 可以在这里保存里程计数据到文件

        } catch (const std::exception& e) {
            std::cerr << "Error processing odometry: " << e.what() << std::endl;
        }
    }

private:
    std::string bag_path_;
    std::string output_dir_;
};

// 如果需要处理点云数据的函数（虽然您的bag中没有点云话题）
void processPointCloud(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) {
    try {
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
        rclcpp::SerializedMessage extracted_serialized_msg(*message->serialized_data);
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = 
            std::make_shared<sensor_msgs::msg::PointCloud2>();
        
        serialization.deserialize_message(&extracted_serialized_msg, cloud_msg.get());

        // 处理点云数据，可以保存为PCD格式
        // 需要pcl库支持

    } catch (const std::exception& e) {
        std::cerr << "Error processing point cloud: " << e.what() << std::endl;
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <path_to_db3_folder> <output_directory>" << std::endl;
        return 1;
    }

    std::string bag_path = argv[1];
    std::string output_dir = argv[2];
    // std::cout << "Reading bag file: " << bag_path << std::endl;
    // std::cout << "Output directory: " << output_dir << std::endl;
    // std::string innn;
    // std::cin >> innn;

    RosbagParser parser(bag_path, output_dir);
    parser.parse();

    return 0;
}