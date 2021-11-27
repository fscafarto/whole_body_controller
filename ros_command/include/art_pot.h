#ifndef ART_POT_H
#define ART_POT_H


#include <ros/ros.h>
//#include "boost/thread.hpp"
#include <iostream>
//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include "sensor_msgs/LaserScan.h"
//#include "tf/tf.h"
#include "for_dyn.h"



class GlobalPlanner{

    public:
        GlobalPlanner(QUADRUPED &quadruped);

        void set_goal(double, double, double);
        //void odom_cb(const nav_msgs::Odometry&);
        void laser_cb(const sensor_msgs::LaserScan& laser);

        void compute_attractive();
        void compute_repulsive();
        void compute_total_force();

        bool check_local_min();

        //Eigen::Vector2d get_position();

        //void run();

        Eigen::Matrix<double,6,1> get_total_force();
        

    private:

        QUADRUPED*  dogbot;

        //ros::NodeHandle _nh;

        //ros::Subscriber _sub_odom;

        ros::Subscriber _sub_scan;

        //ros::Publisher _pub_vel;

        double _x_goal, _y_goal, _yaw_goal;
        //nav_msgs::Odometry _pose;
        Eigen::Matrix<double,6,1> _e=Eigen::MatrixXd::Zero(6,1);
        Eigen::Matrix<double,6,1> _gradient=Eigen::MatrixXd::Zero(6,1);
        Eigen::Matrix<double,6,1> _attractive_pot =Eigen::MatrixXd::Zero(6,1);
        Eigen::Matrix<double,6,1> _repulsive_pot =Eigen::MatrixXd::Zero(6,1);
        Eigen::Matrix<double,6,1> _total_force=Eigen::MatrixXd::Zero(6,1);

        double _ka=0.0;
        double _kr=0.0;
        double _k_damp=0.0;
        bool _obs_found;
        double _obs_distance;
        //void loop();

        double temp_min_old=0.0;

        double temp_x=0.0;
        double temp_x_old=0.0;

        double temp_y=0.0;
        double temp_y_old=0.0;

        double temp_yaw=0.0;
        double temp_yaw_old=0.0;
};

#endif