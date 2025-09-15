#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <example_interfaces/msg/string.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <map>

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

        this->initCSVFiles();
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

    void initCSVFiles() {
        // 初始化里程计CSV文件
        std::ofstream odom_csv(output_dir_ + "/odometry/odometry_data.csv");
        odom_csv << "timestamp,position_x,position_y,position_z,"
                << "orientation_x,orientation_y,orientation_z,orientation_w,"
                << "linear_velocity_x,linear_velocity_y,linear_velocity_z,"
                << "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
                << "topic_name" << std::endl;
        odom_csv.close();

        // 初始化TF数据CSV文件
        // std::ofstream tf_csv(output_dir_ + "/tf/tf_data.csv");
        // tf_csv << "timestamp,parent_frame,child_frame,"
        //        << "translation_x,translation_y,translation_z,"
        //        << "rotation_x,rotation_y,rotation_z,rotation_w" << std::endl;
        // tf_csv.close();
    }
    void processMessage(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) {
        const std::string& topic_name = message->topic_name;
        
        if (topic_name.find("surround_view") != std::string::npos) {
            processCompressedImage(message);
        }
        else if (topic_name == "/livox/lidar") {
            processPointCloud(message);
        }
        else if (topic_name.find("odom") != std::string::npos) {
            processOdometry(message);
        }
        else if (topic_name == "/tf") {
            processTF(message);
        }
        else if (topic_name == "/sensor_sim/Collision_Event") {
            processCollisionEvent(message);
        }
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
                topic_dir = topic_dir.substr(slash_pos+1);
            }
            std::string image_dir = output_dir_ + "/images/" + topic_dir;
            fs::create_directories(image_dir);
            
            std::stringstream filename;
            filename << output_dir_ << "/images/"
                    << topic_dir << "/"
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

    void processPointCloud(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) {
        try {
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*message->serialized_data);
            sensor_msgs::msg::PointCloud2 cloud_msg;
            
            serialization.deserialize_message(&extracted_serialized_msg, &cloud_msg);

            // 转换为PCL点云
            pcl::PointCloud<pcl::PointXYZI> cloud;
            pcl::fromROSMsg(cloud_msg, cloud);

            if (cloud.empty()) {
                std::cerr << "Empty point cloud from topic: " << message->topic_name << std::endl;
                return;
            }

            // 生成文件名
            std::string filename = generateFilename(message->topic_name, message->time_stamp, "pcd");
            std::string full_path = output_dir_ + "/pointclouds/" + filename;

            // 保存为PCD文件
            if (pcl::io::savePCDFileBinary(full_path, cloud) == 0) {
                std::cout << "Saved point cloud: " << filename 
                         << " with " << cloud.size() << " points" << std::endl;
            } else {
                std::cerr << "Failed to save point cloud: " << filename << std::endl;
            }

        } catch (const std::exception& e) {
            std::cerr << "Error processing point cloud: " << e.what() << std::endl;
        }
    }

    void processOdometry(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) {
        try {
            rclcpp::Serialization<nav_msgs::msg::Odometry> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*message->serialized_data);
            nav_msgs::msg::Odometry odom_msg;
            
            serialization.deserialize_message(&extracted_serialized_msg, &odom_msg);

            // 保存到CSV文件
            std::ofstream csv_file(output_dir_ + "/odometry/odometry_data.csv", std::ios::app);
            if (csv_file.is_open()) {
                csv_file << message->time_stamp << ","
                        << odom_msg.pose.pose.position.x << ","
                        << odom_msg.pose.pose.position.y << ","
                        << odom_msg.pose.pose.position.z << ","
                        << odom_msg.pose.pose.orientation.x << ","
                        << odom_msg.pose.pose.orientation.y << ","
                        << odom_msg.pose.pose.orientation.z << ","
                        << odom_msg.pose.pose.orientation.w << ","
                        << odom_msg.twist.twist.linear.x << ","
                        << odom_msg.twist.twist.linear.y << ","
                        << odom_msg.twist.twist.linear.z << ","
                        << odom_msg.twist.twist.angular.x << ","
                        << odom_msg.twist.twist.angular.y << ","
                        << odom_msg.twist.twist.angular.z << ","
                        << message->topic_name << std::endl;
                csv_file.close();
                
                std::cout << "Saved odometry data from: " << message->topic_name << std::endl;
            }

        } catch (const std::exception& e) {
            std::cerr << "Error processing odometry: " << e.what() << std::endl;
        }
    }

    void processTF(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) {
        try {
            rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*message->serialized_data);
            tf2_msgs::msg::TFMessage tf_msg;
            
            serialization.deserialize_message(&extracted_serialized_msg, &tf_msg);

            // 处理每个TF变换
            for (const auto& transform : tf_msg.transforms) {
                std::ofstream csv_file(output_dir_ + "/tf/tf_data.csv", std::ios::app);
                if (csv_file.is_open()) {
                    csv_file << message->time_stamp << ","
                            << transform.header.frame_id << ","
                            << transform.child_frame_id << ","
                            << transform.transform.translation.x << ","
                            << transform.transform.translation.y << ","
                            << transform.transform.translation.z << ","
                            << transform.transform.rotation.x << ","
                            << transform.transform.rotation.y << ","
                            << transform.transform.rotation.z << ","
                            << transform.transform.rotation.w << std::endl;
                    csv_file.close();
                    
                    std::cout << "Saved TF data: " << transform.header.frame_id 
                             << " -> " << transform.child_frame_id << std::endl;
                }
            }

        } catch (const std::exception& e) {
            std::cerr << "Error processing TF: " << e.what() << std::endl;
        }
    }

    void processCollisionEvent(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) {
        try {
            rclcpp::Serialization<example_interfaces::msg::String> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*message->serialized_data);
            example_interfaces::msg::String collision_msg;
            
            serialization.deserialize_message(&extracted_serialized_msg, &collision_msg);

            // 保存碰撞事件
            std::ofstream event_file(output_dir_ + "/collision_events/collision_events.txt", std::ios::app);
            if (event_file.is_open()) {
                event_file << "Timestamp: " << message->time_stamp 
                          << ", Data: " << collision_msg.data << std::endl;
                event_file.close();
                
                std::cout << "Saved collision event: " << collision_msg.data << std::endl;
            }

        } catch (const std::exception& e) {
            std::cerr << "Error processing collision event: " << e.what() << std::endl;
        }
    }

    std::string generateFilename(const std::string& topic, uint64_t timestamp, const std::string& extension) {
        // 清理topic名称（移除特殊字符）
        // std::string clean_topic = topic;
        // std::replace(clean_topic.begin(), clean_topic.end(), '/', '_');
        // if (clean_topic[0] == '_') clean_topic = clean_topic.substr(1);
        
        // return clean_topic + "_" + std::to_string(timestamp) + "." + extension;
        auto nanoseconds = std::chrono::nanoseconds(timestamp);
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(nanoseconds);
        auto fractional = nanoseconds - seconds;
        std::stringstream filename;
        filename << seconds.count() << "_"
                << std::setw(9) << std::setfill('0') << fractional.count()
                << "." << extension;
        return filename.str();
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