#include <cfloat>
#include <iostream>
#include <vector>
#include <math.h>
#include <limits.h>
#include <tf/transform_datatypes.h>
#include "ros/ros.h"
#include "src/pure_pursuit/geometry.hpp"
#include "MPC.hpp"
#include "src/pid/PID.hpp"
#include "src/brake_feedforward/brakeFF.hpp"
#include "src/kalman/lkf.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "driverless_msgs/PoseStamped.h"
#include "driverless_msgs/curvature_throttle.h"
// #include "driverless_msgs/can.h"
#include "driverless_msgs/brake_values.h"
#include <can_msgs/Frame.h>
#include "driverless_msgs/steering.h"
#include "driverless_msgs/velocity.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float64.h"



// TODO: make controls aware of mission finished
class DPP {
public:
    DPP(ros::NodeHandle &nh, std::string steering_angle_topic,std::string brake_topic,std::string throttle_topic, PIDController& brake_pid, PIDController& throttle_pid, MPC& mpc):
            brake_pid(brake_pid),
            throttle_pid(throttle_pid),
            controller(controller),
            is_started(false){
        // Correct message types and publishers
        steering_angle_pub = nh.advertise<driverless_msgs::steering>(steering_angle_topic, 10);
        brake_pub = nh.advertise<can_msgs::Frame>(brake_topic, 10);
        throttle_pub = nh.advertise<can_msgs::Frame>(brake_topic, 10);
        lookahead_publisher = nh.advertise<std_msgs::Float64>("/controls/lookahead", 10);
        to_data_logger = nh.advertise<driverless_msgs::brake_values>("/data_logger/brake_values_raw", 10);
    }



    void callback(const driverless_msgs::curvature_throttle::ConstPtr &curve_msg,
                  const driverless_msgs::PoseStamped::ConstPtr &pose_msg,
                  const driverless_msgs::velocity::ConstPtr &velocity_msg) {

        // Extract position and yaw from pose message
        std::cout << "Entrato in Callback" << std::endl;
        this->x = pose_msg->pose.position.x;
        this->y = pose_msg->pose.position.y;
        this->z = pose_msg->pose.position.z;
        this->yaw = tf::getYaw(pose_msg->pose.orientation);  // Use tf::getYaw to extract yaw from quaternion

        // Get velocity from phonic wheel (velocity message)
        this->velocity = velocity_msg->velocity;

        // Extract values from trajectory node output
        this->curvature = curve_msg->curvatures;
        this->velocities = curve_msg->velocities;
        this->points = curve_msg->trajectory;
        this->reference_velocity = velocities[0]; // First point in velocities

        // Calculate current position and direction
        Point position(x - 0.8 * cos(yaw), y - 0.8 * sin(yaw)); // 0.8 is the rear axle distance
        Point direction(cos(yaw) + x, sin(yaw) + y); // Absolute direction vector

        // Variables to store brake pid and ff to send it to telemetry
        double brake_pid_value;
        double brake_ff_value;

        // Convert the trajectory points into a usable format
        std::vector<Point> trajectory;
        for (int i = 0; i < points.size(); i++) {
            trajectory.push_back(Point(points[i].x, points[i].y));
        }

        Eigen::VectorXd x0 << x, y, yaw, lateral_velocity, yaw_rate;
        mpc.updateDiscretization(velocity, yaw, acceleration);
        double steering = mpc.compute(x0, trajectory);
        ROS_INFO_STREAM("reference velocity "<<reference_velocity);
        ROS_INFO_STREAM("lateral distance from pat"<<lateral_distance());
    

        steering = steering * 180 / M_PI; // Convert to degrees
        ROS_INFO_STREAM("MPC steering: "<<steering);

        double throttle = 0; // percentage of throttle
        double brake = 0; // front brake pressure

        unsigned long int current_time = nanos();
        double dt = (current_time - previous_time) / 1e9;
        double deceleration =  (reference_velocity - this->velocity)/dt;

        double dead_zone = 1; // m/s

        // pid per decellerazione
        if (reference_velocity - this->velocity < -dead_zone) {
            ROS_INFO_STREAM ("Calcolo Brake");

            brake_pid_value = brake_pid.calculate(reference_velocity, this->velocity, dt)*10;
            brake_ff_value = brake_ff.DecToPress(deceleration)/10000;
            brake = brake_pid_value + brake_ff_value + 20;

            ROS_INFO_STREAM(" decelleration " << deceleration << "." );
            //brake = brake_pid.calculate(reference_velocity, this->velocity, dt);
        }
            // pid per accelerazione
        else if (reference_velocity - this->velocity > dead_zone) {
            if(is_started)
                //throttle = 8;
                throttle = 8.0 + throttle_pid.calculate(reference_velocity, this->velocity, dt);

            else
                throttle = 14;
            if (this->velocity > 3.0){
                is_started = true;
            }
        }
        else {
            ROS_INFO_STREAM("Dead zone");
            throttle_pid.set_integral_to_zero();
            brake_pid.set_integral_to_zero();
        }

        previous_time = current_time;
        ROS_INFO_STREAM("brake " << brake << " throttle " << throttle);
        ROS_INFO_STREAM(" with r velocity " << reference_velocity << " and velocity " << velocity);


        //kalman.Linear({this->x, this->y, this->yaw});

        // Create messages for throttle, brake, and steering
        driverless_msgs::steering steering_msg;
        can_msgs::Frame throttle_msg;
        can_msgs::Frame brake_msg;
        ros::Time timestamp = ros::Time::now();


        // Populate the messages

        steering_msg.position = static_cast<float>(steering);

        brake_msg.id = 100;
        brake_msg.dlc = 2;
        uint16_t brake_data = static_cast<uint16_t>(brake);
        brake_msg.data = {
                static_cast<uint8_t>(brake_data >> 8),  // MSB
                static_cast<uint8_t>(brake_data & 0xFF) // LSB
        };

        throttle_msg.id = 354;
        throttle_msg.dlc = 1;
        throttle_msg.data = {static_cast<uint8_t>(throttle)}; //Assuming throttlecurve_subscriber is a percentage, it easily fits in a byte

        driverless_msgs::brake_values brake_values_msg;
        brake_values_msg.header.stamp = timestamp;
        brake_values_msg.brakePID = brake_pid_value;
        brake_values_msg.brakeFF = brake_ff_value;

        // Publish the messages
        throttle_msg.header.stamp = timestamp;
        brake_msg.header.stamp = timestamp;
        steering_angle_pub.publish(steering_msg);
        throttle_pub.publish(throttle_msg);
        brake_pub.publish(brake_msg);
        to_data_logger.publish(brake_values_msg);
    }
    //Generate an error that tracks the evolution of the lateral error
    float lateral_distance() {
        if (this->points.size() > 0){
            Point p(this->points.at(0).x, this->points.at(0).y);
            float distance = std::sqrt(
                    ((p.x - this->x) * (p.x - this->x)) +
                    ((p.y - this->y) * (p.y - this->y))
            );
            return distance;
        }
        return 0.0;
    }

private:
    ros::Publisher steering_angle_pub;
    ros::Publisher brake_pub;
    ros::Publisher throttle_pub;
    ros::Publisher to_data_logger;
    ros::Publisher lookahead_publisher;
    bool is_started;

    unsigned long int previous_time = nanos();


    // State variables
    double x = 0, y = 0, z = 0, yaw = 0;
    double velocity = 0;
    double reference_velocity = 0;
    std::vector<float> curvature;
    std::vector<float> velocities;
    std::vector<geometry_msgs::Point> points;

    MPC mpc;

    PIDController throttle_pid;
    PIDController brake_pid;
    BrakeFF brake_ff{0.66};
    //KalmanFilter kalman {};
};

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    ROS_INFO("Started Control Node");

    // Topic names
    std::string brake_topic = "/can_handler/to_can";
    std::string throttle_topic = "/can_handler/throttle";
    std::string steering_topic = "/can_handler/to_steering";

    //Getting pid parameter from launch file
    double ki_throttle, kp_throttle, kd_throttle;
    double kp_brake, ki_brake, kd_brake;
    double min_brake, max_brake, low_pass_brake;
    double min_throttle, max_throttle, low_pass_throttle;
    double lookahead_a, lookahead_k;
    ros::param::get("~kp_throttle", kp_throttle);
    ros::param::get("~ki_throttle", ki_throttle);
    ros::param::get("~kd_throttle", kd_throttle);
    ros::param::get("~kp_brake", kp_brake);
    ros::param::get("~ki_brake", ki_brake);
    ros::param::get("~kd_brake", kd_brake);
    ros::param::get("~min_brake", min_brake);
    ros::param::get("~max_brake", max_brake);
    ros::param::get("~low_pass_brake", low_pass_brake);
    ros::param::get("~min_throttle", min_throttle);
    ros::param::get("~max_throttle", max_throttle);
    ros::param::get("~low_pass_throttle", low_pass_throttle);
    ros::param::get("~lookahead_k", lookahead_k);
    ros::param::get("~lookahead_a", lookahead_a);

    PIDController brake_pid(kp_brake, ki_brake, kd_brake, min_brake, max_brake, low_pass_brake);
    PIDController throttle_pid(kp_throttle, ki_throttle, kd_throttle, min_throttle, max_throttle, low_pass_throttle);
    MPC mpc();

    // Create DPP controller instance
    DPP controller(nh, steering_topic, brake_topic, throttle_topic, brake_pid, throttle_pid, mpc);

    // Set up subscribers for pose, curvature/throttle, and velocity topics
    message_filters::Subscriber<driverless_msgs::curvature_throttle> curve_subscriber(nh, "/trajectory/output", 10);
    message_filters::Subscriber<driverless_msgs::PoseStamped> pose_subscriber(nh, "/orb_slam3/camera_pose", 10);
    message_filters::Subscriber<driverless_msgs::velocity> velocity_subscriber(nh, "/data_logger/speed_actual", 10);
    ROS_INFO("Creati subscriber");
    // Subscribe per il log della media delle velocità delle ruote foniche anteriori che viene letta dal CAN
    // cambiato da Float32 visto che Float32 non ha un header, ergo non ha un timestamp, ergo ApproximateTime non funziona

    // message_filters::Subscriber<std_msgs::Float32> velocity_subscriber(nh, // "/data_logger/speed_actual", 10);
    // Subscribe per il log della media delle velocità delle ruote foniche anteriori che viene letta dal CAN
    // message_filters::Subscriber<std_msgs::Float32> pressure_subscriber(nh, "/data_logger/brake_actual", 10);
    // Subscriber to read front and rear pressure in case we would like to calculate repartition

    // Synchronize the message filters

    // TODO, sostituire a velocity_subscriber, reference_velocity per ridurre oscillazioni sterzo a basse velocità.
    // NB: La velocità di riferimento dovrebbe già avere lo stesso time stamp di curve_subscriber.
    typedef message_filters::sync_policies::ApproximateTime<driverless_msgs::curvature_throttle,
    driverless_msgs::PoseStamped,
    driverless_msgs::velocity> sync_policy;
    message_filters::Synchronizer<sync_policy> sync(sync_policy(10), curve_subscriber, pose_subscriber, velocity_subscriber);
    ROS_INFO("Creata time policy");
    sync.registerCallback(boost::bind(&DPP::callback, &controller, _1, _2, _3));
    ROS_INFO("Registrata callback");


    ros::spin();
    return 0;
}