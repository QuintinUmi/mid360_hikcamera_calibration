#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include <string>

// 全局点集存储
std::vector<std::string> points;

// 显示菜单
void showMenu() {
    std::cout << "\n--- Point Collection Menu ---\n";
    std::cout << "1. Capture current pointset\n";
    std::cout << "2. List all pointsets\n";
    std::cout << "3. Delete a pointset\n";
    std::cout << "4. Exit\n";
    std::cout << "Select an option: ";
}

// 捕获点集
void capturePointset(ros::Publisher &pub) {
    // 此处只是模拟，实际应由ROS节点动态接收
    std::string pointset = "Pointset_" + std::to_string(points.size() + 1);
    points.push_back(pointset);
    std_msgs::String msg;
    msg.data = pointset;
    pub.publish(msg);
    std::cout << "Captured and published: " << pointset << std::endl;
}

// 列出所有点集
void listPointsets() {
    std::cout << "Stored pointsets:\n";
    for (size_t i = 0; i < points.size(); ++i) {
        std::cout << i + 1 << ". " << points[i] << std::endl;
    }
}

// 删除点集
void deletePointset() {
    if (points.empty()) {
        std::cout << "No pointsets available to delete." << std::endl;
        return;
    }
    int index;
    std::cout << "Enter index of pointset to delete (1 to " << points.size() << "): ";
    std::cin >> index;
    if (index > 0 && index <= points.size()) {
        points.erase(points.begin() + index - 1);
        std::cout << "Pointset " << index << " deleted." << std::endl;
    } else {
        std::cout << "Invalid index, try again." << std::endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "point_manager");
    ros::NodeHandle nh;
    ros::Publisher point_pub = nh.advertise<std_msgs::String>("point_topic", 10);

    int choice;
    while (ros::ok()) {
        showMenu();
        std::cin >> choice;
        switch (choice) {
            case 1:
                capturePointset(point_pub);
                break;
            case 2:
                listPointsets();
                break;
            case 3:
                deletePointset();
                break;
            case 4:
                std::cout << "Exiting..." << std::endl;
                return 0;
            default:
                std::cout << "Invalid choice, try again." << std::endl;
        }
    }

    return 0;
}