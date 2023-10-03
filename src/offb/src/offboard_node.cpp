#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <cmath>

#define LASER_MIN 0.3
#define LASER_INF 20.0
#define P_POS 0.9
#define MAX_VEL 0.25

double r, p, y, x, yaw, z;
double target_x, target_y;
int number = -1;
double fly_height = 0.5, drop_height = 0.4;
bool is_detect = 0, got_plan = 0;

mavros_msgs::State current_state;  // 创建全局变量
geometry_msgs::PoseStamped pos_drone;
sensor_msgs::LaserScan scan;

enum FLY_MOD {
    READY,
    TAKEOFF,
    HOVER,
    LANDING,
    DROP,
    END,
    GOTO_DROP,
    HOME,
    SCAN_QR,
    FIND_QR_MID,
    AROUND,
} state;

struct point {
    double x;
    double y;
    double z;
} waypoint[10], left_land, right_land, qrcode;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

point next_point;

void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    ROS_INFO("Received path with %zu poses.", path_msg->poses.size());
    got_plan = true;
    for (size_t i = 0; i < path_msg->poses.size(); ++i) {
        double tx = path_msg->poses[i].pose.position.x;
        double ty = path_msg->poses[i].pose.position.y;
        if ((tx - x) * (tx - x) + (ty - y) * (ty - y) > 0.07) {
            next_point.x = tx;
            next_point.y = ty;
            break;
        }
        // ROS_INFO("Pose %zu: [%f, %f, %f].", i,
        //          path_msg->poses[i].pose.position.x,
        //          path_msg->poses[i].pose.position.y,
        //          path_msg->poses[i].pose.position.z);
    }
}

// void target_cb(const geometry_msgs::PoseStamped::ConstPtr& marker_pose){
//     target_x = marker_pose->pose.position.x;
//     target_y = marker_pose->pose.position.y;
//     is_detect=1;
// }

// void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
// {
//     int count = scan_in->ranges.size(),min_index=0;
//     scan.ranges.clear();
//     float degree = scan_in->angle_min;
//     float min = LASER_INF;
//     for(int i = 0; i < count; i++){
//         if(isnan(scan_in->ranges[i])||isinf(scan_in->ranges[i])||scan_in->ranges[i]<LASER_MIN)
//             scan.ranges.push_back(LASER_INF);
//         else scan.ranges.push_back(scan_in->ranges[i]);
//         if(scan.ranges[i]<min) min=scan.ranges[i],min_index=i;
//     }
//     ROS_INFO("count = [%d]  min_distance = [%f]  , index =  [%d] \n",count,
//     min, min_index);
//     // for(int i = 0; i < count; i++){
//     //     std::cout << isinf(scan_in->ranges[i]) << " \n";
//     //     // scan.ranges.push_back(scan_in->ranges[i]);
//     // }
// }

void pos_cb(const nav_msgs::Odometry::ConstPtr& odom_3d) {
    pos_drone.pose.position.x = odom_3d->pose.pose.position.x;
    pos_drone.pose.position.y = odom_3d->pose.pose.position.y;
    pos_drone.pose.position.z = odom_3d->pose.pose.position.z;
    pos_drone.pose.orientation = odom_3d->pose.pose.orientation;
    x = pos_drone.pose.position.x;
    y = pos_drone.pose.position.y;
    z = pos_drone.pose.position.z;
    tf::Quaternion RQ2;
    tf::quaternionMsgToTF(pos_drone.pose.orientation, RQ2);
    tf::Matrix3x3(RQ2).getRPY(r, p, yaw);
}

void target_cb(const geometry_msgs::PoseStamped::ConstPtr& marker_pose) {
    target_x = marker_pose->pose.position.x * z + 0.07;
    target_y = marker_pose->pose.position.y * z + 0.02;
    number = (int)marker_pose->pose.position.z;
    if (number != -1)
        is_detect = 1;
}

bool isget(point a, double dis = 0.1) {
    if (fabs(a.x - x) < dis && fabs(a.y - y) < dis)
        return true;
    else
        return false;
}

void pos_vel(geometry_msgs::Twist& vel,
             double target_x,
             double target_y,
             double target_z = fly_height) {
    vel.linear.x = (target_x - x) * P_POS;
    vel.linear.y = (target_y - y) * P_POS;
    vel.linear.z = (target_z - z) * P_POS;
    if (fabs(vel.linear.x) > MAX_VEL)
        vel.linear.x = vel.linear.x < 0 ? -MAX_VEL : MAX_VEL;
    if (fabs(vel.linear.y) > MAX_VEL)
        vel.linear.y = vel.linear.y < 0 ? -MAX_VEL : MAX_VEL;
    if (fabs(vel.linear.z) > MAX_VEL)
        vel.linear.z = vel.linear.z < 0 ? -MAX_VEL : MAX_VEL;
}

bool isget_qr = 0;
std::string qr_str;

void qr_cb(const std_msgs::String::ConstPtr& qr) {
    isget_qr = 1;
    qr_str = qr->data;
    // std::cout << qr_str << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber path_sub =
        nh.subscribe("/move_base/DWAPlannerROS/global_plan", 1, pathCallback);
    // ROS_INFO("offb_node started");
    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>(
        "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client =
        nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", 100, pos_cb);
    ros::Subscriber target_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/target", 10, target_cb);
    ros::Publisher drop_pub =
        nh.advertise<std_msgs::String>("/drop", 10);  // 发布投放指令
    ros::Subscriber qr_sub = nh.subscribe<std_msgs::String>("/qr", 10, qr_cb);

    int camera_channel = 0;

    for (int i = 1; i <= 6; i++) {
        nh.param<double>("x" + std::to_string(i), waypoint[i].x, 0.0);
        nh.param<double>("y" + std::to_string(i), waypoint[i].y, 0.0);
    }
    nh.param<double>("fly_height", fly_height, 1.4);
    nh.param<double>("drop_height", drop_height, 0.4);
    nh.param<double>("left_land_x", left_land.x, 0.0);
    nh.param<double>("left_land_y", left_land.y, 0.0);
    nh.param<double>("right_land_x", right_land.x, 0.0);
    nh.param<double>("right_land_y", right_land.y, 0.0);
    nh.param<double>("qrcode_x", qrcode.x, 0.0);
    nh.param<double>("qrcode_y", qrcode.y, 0.0);
    nh.param<int>("camera_channel", camera_channel, 0);

    // ros print waypoint
    for (int i = 1; i <= 6; i++)
        ROS_INFO("x%d = %f, y%d = %f", i, waypoint[i].x, i, waypoint[i].y);
    // ros print param above
    ROS_INFO("fly_height = %f", fly_height);
    ROS_INFO("drop_height = %f", drop_height);
    ROS_INFO("left_land_x = %f, left_land_y = %f", left_land.x, left_land.y);
    ROS_INFO("right_land_x = %f, right_land_y = %f", right_land.x,
             right_land.y);
    ROS_INFO("qrcode_x = %f, qrcode_y = %f", qrcode.x, qrcode.y);
    ROS_INFO("camera_channel = %d", camera_channel);

    // cv::VideoCapture cap(camera_channel);
    // cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    // cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    // create a subscriber to subscribe the topic /scan
    // ros::Subscriber sub_scan = nh.subscribe("/scan", 1, scanCallback);

    // ros::Subscriber target_sub =
    //     nh.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 100,
    //     target_cb);

    // the setpoint publishing rate MUST be faster than 2Hz

    ros::Rate rate(20.0);

    int mid_point[7][7] = {{0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0}};

    mid_point[1][5] = mid_point[1][6] = 2;
    mid_point[2][4] = 1;
    mid_point[3][4] = 5;
    mid_point[4][2] = mid_point[4][3] = 5;
    mid_point[5][1] = 2;
    mid_point[6][1] = 2;

    // wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("offb_node started");
    double px[5], py[5];

    geometry_msgs::PoseStamped pose;
    geometry_msgs::Twist vel;

    int order[5] = {1, 2, 3, 4, 5};
    int where[20] = {0};
    char destination;
    int now_pos = 0;

    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;

    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = fly_height;

    px[0] = x, py[0] = y;

    pose.pose.orientation.w = 1;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;

    // send a few setpoints before starting
    // 在切换到offboard模式之前，你必须先发送一些期望点信息到飞控中。不然飞控会拒绝切换到offboard模式。
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    // open camera

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    state = READY;

    while (ros::ok()) {
        switch (state) {
            case READY:
                if (current_state.mode != "OFFBOARD" &&
                    (ros::Time::now() - last_request > ros::Duration(0.5))) {
                    if (set_mode_client.call(offb_set_mode) &&
                        offb_set_mode.response.mode_sent) {
                        ROS_INFO("Offboard enabled");
                    }
                    last_request = ros::Time::now();
                } else {
                    if (!current_state.armed &&
                        (ros::Time::now() - last_request >
                         ros::Duration(0.5))) {
                        if (arming_client.call(arm_cmd) &&
                            arm_cmd.response.success) {
                            ROS_INFO("Vehicle armed");
                            state = TAKEOFF;  // 替换模式
                        }
                        last_request = ros::Time::now();
                    }
                }
                local_pos_pub.publish(pose);
                break;
            case TAKEOFF:
                ROS_INFO("TAKEOFF NOW");
                local_pos_pub.publish(pose);
                if (std::fabs(z - fly_height) < 0.15 && got_plan) {
                    state = FIND_QR_MID;
                    last_request = ros::Time::now();
                }
                break;
            case FIND_QR_MID:
                ROS_INFO("GO 1 NOW");
                // pose.pose.position.x = waypoint[1].x;
                // pose.pose.position.y = waypoint[1].y;
                pos_vel(vel, next_point.x, next_point.y);
                ROS_INFO("vel_x%f  vel_y%f", vel.linear.x, vel.linear.y);
                local_vel_pub.publish(vel);

                // if (isget(waypoint[1], 0.1)) {
                //     state = SCAN_QR;
                // }
                break;
            case SCAN_QR:
                ROS_INFO("GO 2 NOW");
                pos_vel(vel, waypoint[2].x, waypoint[2].y);
                // ROS_INFO("GO 2 NOW");
                local_vel_pub.publish(vel);

                if (isget(waypoint[2], 0.1)) {
                    state = AROUND;
                }
                break;
            case AROUND:
                ROS_INFO("GO 3 NOW");
                pos_vel(vel, waypoint[3].x, waypoint[3].y);
                local_vel_pub.publish(vel);
                if (isget(waypoint[3], 0.1)) {
                    ROS_INFO("GET FINAL");
                }
                break;
            case GOTO_DROP:
                static int now_drop = 1;
                static bool mid_flag = false;
                if (now_drop == 4) {
                    state = HOME;
                    break;
                }
                ROS_INFO("GOTO_DROP %d NOW", order[now_drop]);
                if (mid_point[now_pos][where[order[now_drop]]] == 0)
                    mid_flag = true;

                if (mid_flag) {  // 已经走过中间点，直达目的地
                    pose.pose.position.x = waypoint[where[order[now_drop]]].x;
                    pose.pose.position.y = waypoint[where[order[now_drop]]].y;
                    pose.pose.position.z = fly_height;
                    local_pos_pub.publish(pose);
                    if (isget(waypoint[where[order[now_drop]]])) {
                        now_pos = where[order[now_drop]];
                        mid_flag = false;
                        state = DROP;
                    }
                } else {  // 还没走过中间点，先走中间点
                    pose.pose.position.x =
                        waypoint[mid_point[now_pos][where[order[now_drop]]]].x;
                    pose.pose.position.y =
                        waypoint[mid_point[now_pos][where[order[now_drop]]]].y;
                    pose.pose.position.z = fly_height;
                    local_pos_pub.publish(pose);
                    if (isget(waypoint[mid_point[now_pos]
                                                [where[order[now_drop]]]])) {
                        mid_flag = true;  // 已经走到中间点
                    }
                }
                break;

            case DROP:
                static bool get_precise = false;
                static double precise_x, precise_y;
                ROS_INFO("DROP %d NOW", order[now_drop]);
                if (!get_precise) {
                    precise_x = x + target_x;
                    precise_y = y + target_y;
                    get_precise = true;
                }
                pose.pose.position.x = precise_x;
                pose.pose.position.y = precise_y;
                pose.pose.position.z = drop_height;
                local_pos_pub.publish(pose);

                if (z < drop_height + 0.1) {  // 投放，投放完毕后，进入下一阶段
                    get_precise = 0;
                    now_drop++;
                    mid_flag = false;
                    std_msgs::String message;
                    message.data = "drop";
                    drop_pub.publish(message);
                    state = GOTO_DROP;
                }
                break;
            case HOME:
                pose.pose.position.z = fly_height;
                if (destination == 'l') {
                    pose.pose.position.x = left_land.x;
                    pose.pose.position.y = left_land.y;
                    local_pos_pub.publish(pose);
                    if (isget(left_land)) {
                        state = LANDING;
                    }
                } else {
                    pose.pose.position.x = right_land.x;
                    pose.pose.position.y = right_land.y;
                    local_pos_pub.publish(pose);
                    if (isget(right_land)) {
                        state = LANDING;
                    }
                }
                break;
            case LANDING:
                ROS_INFO("LANDING NOW");
                static point prescise_land{x + target_x, y + target_y};
                static bool get_prescise_land = false;
                if (!get_prescise_land) {
                    prescise_land.x = x + target_x;
                    prescise_land.y = y + target_y;
                    get_prescise_land = true;
                }
                pose.pose.position.x = prescise_land.x;
                pose.pose.position.y = prescise_land.y;
                pose.pose.position.z = 0;
                local_pos_pub.publish(pose);
                if (z < 0.2) {
                    // 切manul模式,上锁停桨
                    offb_set_mode.request.custom_mode = "MANNUL";
                    set_mode_client.call(offb_set_mode);
                    arm_cmd.request.value = false;
                    arming_client.call(arm_cmd);
                    state = END;
                }
                break;
            case END:
                ROS_INFO("MISSION COMPLETE!");
                break;
            default:
                break;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
