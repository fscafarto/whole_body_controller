#ifndef FOR_DYN_H
#define FOR_DYN_H

// C headers
#include <cstdlib>

// Eigen headers
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR> 

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// Helpers function to convert between


// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>

//Libraries for ros subscriber
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>


#include <gazebo_msgs/ModelStates.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ContactsState.h>

#include "boost/thread.hpp"


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "alglib/optimization.h"

#include <tf/transform_broadcaster.h>

#include "optimal.h"

#include "planning.h"

//#include "art_pot.h"

#include "scanner.h"

#include <chrono>

//gazebo libraries for the configuration
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>

#include <gazebo/common/Time.hh>
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/GetLinkProperties.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "gazebo_msgs/SetModelState.h"
#include <std_srvs/Empty.h>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "gazebo_msgs/ContactsState.h"
#include <random>

#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"

//for relative path
#include <ros/package.h> 


//graph
#include <iostream>
#include <sstream>




using namespace std;

struct EigenRobotState
{
    void resize(int nrOfInternalDOFs)
    {
        jointPos.resize(nrOfInternalDOFs); //resize del vettore, noto il numero dei giunti
        jointVel.resize(nrOfInternalDOFs);
    }

    void random()
    {
        world_H_base.setIdentity();
        jointPos.setRandom();
        baseVel.setRandom();
        jointVel.setRandom();
        gravity[0] = 0;
        gravity[1] = 0;
        gravity[2] = -9.8;
    }

    Eigen::Matrix4d world_H_base;
    //Eigen::VectorXd jointPos;
    Eigen::Matrix<double,12,1> jointPos;
    Eigen::Matrix<double,6,1> baseVel;
    //Eigen::VectorXd jointVel;
    Eigen::Matrix<double,12,1> jointVel;
    Eigen::Vector3d gravity;
    
    //Eigen::MatrixXd v; //full velocity vector
    //Eigen::VectorXd v_new;

    // See https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};





class QUADRUPED{

	public:
        QUADRUPED();

        void run();

		void joint_pos_cb( const sensor_msgs::JointState::ConstPtr& data );

        void box_states_callback( const gazebo_msgs::ModelStates &box_state_current );

        void f_gr_bl_cb( const gazebo_msgs::ContactsState::ConstPtr& box_state_current );

        void f_gr_br_cb( const gazebo_msgs::ContactsState::ConstPtr& box_state_current );

        void f_gr_fl_cb( const gazebo_msgs::ContactsState::ConstPtr& box_state_current );

        void f_gr_fr_cb( const gazebo_msgs::ContactsState::ConstPtr& box_state_current );

        int stand_phase();

        int init_model();

        int compute_j_st(int flag);

        void compute_m22_bar();

        void compute_c2_bar();

        void compute_j_st_j_bar(int flag);

        //void compute_q();  

        int swing_phase();

        int swing_phase2();

        void loop();

        int update(int flag);

        void camera_scan();

        //void publish_values();

        //void artpot();

        //void compute_estimation();

        //void distxbr_cb(std_msgs::Float64ConstPtr dist_x);

        //void distxbl_cb(std_msgs::Float64ConstPtr dist_x);

        //void distxfl_cb(std_msgs::Float64ConstPtr dist_x);

        //void distxfr_cb(std_msgs::Float64ConstPtr dist_x);

        //void distxbase_cb(std_msgs::Float64ConstPtr dist_x);




        //get function for optimal

        //void set_pc(Eigen::Vector3d data);

        Eigen::Vector3d get_pc();

        //void set_r_c(Eigen::Matrix<double,6,1> data);

        //void set_r_c_dot(Eigen::Matrix<double,6,1> data);

        void set_w_com_des(Eigen::Matrix<double,6,1> data);

        Eigen::Matrix<double,6,1> get_r_c();

        Eigen::Matrix<double,6,1> get_r_c_dot();

        Eigen::Vector3d get_pc_dot();

        Eigen::Matrix<double,6,1> get_r_c_ref();

        Eigen::Matrix<double,6,1> get_r_c_ref_dot();

        Eigen::Matrix<double,18,18> get_mass_matrix();

        Eigen::Matrix<double,6,1> get_r_c_ref_dot_dot();

        /*Eigen::Matrix<double,3,4> get_L_bar_1_0();

        Eigen::Matrix<double,3,4> get_L_bar_2_0();

        Eigen::Matrix<double,3,4> get_n_bar_0();

        Eigen::Matrix<double,3,2> get_L_bar_1_1();

        Eigen::Matrix<double,3,2> get_L_bar_2_1();

        Eigen::Matrix<double,3,2> get_n_bar_1();*/

        double get_mu();

        //std::array<bool, 4> get_contact();

        //void set_f_st_hat_0(Eigen::Matrix<double,12,1> data);

        //void set_f_st_hat_1(Eigen::Matrix<double,6,1> data);

        //void set_f_sw_hat(Eigen::Matrix<double,6,1> data);

        Eigen::Matrix<double,12,6> get_j_st_c_bar_0();

        Eigen::Matrix<double,6,6> get_j_st_c_bar_1();



        Eigen::Matrix<double,12,18> get_j_st_bar_0();

        Eigen::Matrix<double,6,18> get_j_st_bar_1();

        Eigen::Matrix<double,18,18> get_t_bar();

        Eigen::Matrix<double,18,1> get_h();

        Eigen::Matrix<double,18,18> get_t_inv_der();

        Eigen::Matrix<double,18,1> get_v_c();

        Eigen::Matrix<double,24,18> get_j_bar();

        Eigen::Matrix<double,12,18> get_j_st_dot_0();

        Eigen::Matrix<double,6,1> get_j_st_dot_1();

        Eigen::Matrix<double,12,12> get_j_st_j_bar_0();

        Eigen::Matrix<double,6,12> get_j_st_j_bar_1();

        Eigen::Matrix<double,12,18> get_c2_bar();

        void set_q_dot_dot_des(Eigen::Matrix<double,12,1> data);

        void set_f_gr_star(Eigen::Matrix<double,12,1> data);

        Eigen::Matrix<double,6,12> get_j_sw_j_bar();

        Eigen::Matrix<double,6,6> get_j_sw_c_bar();

        //Eigen::Matrix<double,6,1> get_x_sw_cmd_dot_dot();

        Eigen::Matrix<double,6,1> get_j_sw_dot();

        //Eigen::Matrix<double,12,1> get_gamma_1();

        //Eigen::Matrix<double,12,1> get_gamma_2();

        //Eigen::Matrix<double,12,1> get_gamma_3();

        //void set_gamma_1(Eigen::Matrix<double,12,1> data);
        
        //void set_gamma_2(Eigen::Matrix<double,12,1> data);

        //void set_gamma_3(Eigen::Matrix<double,12,1> data);

        Eigen::Matrix<double,12,1> get_jnt_torques_star();

        Eigen::Matrix<double,12,12> get_j_st_j_bar_1_new();

        Eigen::Matrix<double,6,1> get_w_com_des();

        //Eigen::Matrix<double,12,1> get_f_st_hat_0();

        //Eigen::Matrix<double,6,1> get_f_st_hat_1();

        //Eigen::Matrix<double,6,1> get_f_sw_hat();

        //iDynTree::KinDynComputations get_kindyncomp();

        Eigen::Quaterniond get_quaternion();

        int get_stl1();

        int get_stl2();

        int get_swl1();

        int get_swl2();

        Eigen::Matrix<double,18,1> get_bias_com();

        

        //

        //get functions for planning

        Eigen::Matrix<double,6,1> get_init_pos();

        Eigen::Matrix<double,6,1> get_init_vel();

        Eigen::Matrix<double,6,1> get_fin_pos();

        //void set_r_c_ref(Eigen::Matrix<double,6,1> data);

        //void set_r_c_ref_dot(Eigen::Matrix<double,6,1> data);

        //void set_r_c_ref_dot_dot(Eigen::Matrix<double,6,1> data);

        //Eigen::Matrix<double,12,1> get_f_gr();

        //Eigen::Matrix<double,3,3> get_Tbl();

        //Eigen::Matrix<double,3,3> get_Tbr();

        //Eigen::Matrix<double,3,3> get_Tfl();

        //Eigen::Matrix<double,3,3> get_Tfr();

        Eigen::MatrixXd getBRpos();

        Eigen::MatrixXd getBLpos();

        Eigen::MatrixXd getFLpos();

        Eigen::MatrixXd getFRpos();

        Eigen::MatrixXd getBRvel();

        Eigen::MatrixXd getBLvel();

        Eigen::MatrixXd getFLvel();

        Eigen::MatrixXd getFRvel();

        //void set_x_sw_des(Eigen::Matrix<double,6,1> data);

        //void set_x_sw_des_dot(Eigen::Matrix<double,6,1> data);

        //void set_x_sw_des_dot_dot(Eigen::Matrix<double,6,1> data);

        Eigen::Matrix4d get_world_base();

        Eigen::Matrix<double,6,18> get_j_st_1();

        Eigen::Matrix<double,6,1> get_accd();

        Eigen::Matrix<double,6,1> get_veldelta();

        Eigen::Matrix<double,6,1> get_posdelta();

        Eigen::Matrix<double,6,18> get_j_sw();

        //void set_pc_dot(Eigen::Vector3d data);

        //Eigen::Matrix<double,18,18> get_c();

        //bool get_overlap();

        //void set_overlap(bool data);

        //double get_diff();

        //void set_diff(double data);

        Eigen::Matrix<double,12,1> get_joint_pos();

        double get_roll();

        double get_pitch();

        double get_yaw();

        ros::NodeHandle get_nh();

        //Eigen::Matrix<double,12,1> get_f_ext();

        double get_m();

        bool get_started();

        
                





        //  

        ros::Time _begin;

        double _begin_first;

        Eigen::Matrix<double,6,1> _init_pos;

        Eigen::Matrix<double,6,1> _init_vel;

        Eigen::Matrix<double,6,1> _fin_pos;

        gazebo::transport::NodePtr _node;	

        

        

        
        
	
	private:
		ros::NodeHandle _nh;

        ros::Subscriber _sub_box_state;

        ros::Publisher _pub_joint_torque_blk;

        ros::Publisher _pub_joint_torque_blp;

        ros::Publisher _pub_joint_torque_blr;

        ros::Publisher _pub_joint_torque_brk;

        ros::Publisher _pub_joint_torque_brp;

        ros::Publisher _pub_joint_torque_brr;

        ros::Publisher _pub_joint_torque_flk;

        ros::Publisher _pub_joint_torque_flp;

        ros::Publisher _pub_joint_torque_flr;

        ros::Publisher _pub_joint_torque_frk;

        ros::Publisher _pub_joint_torque_frp;

        ros::Publisher _pub_joint_torque_frr;

        //ros::Publisher _pub_joint_torque; 

		ros::Subscriber _sub_joint_pos;

        //double _joint_pos_temp[12];

        ros::Subscriber _sub_f_gr_bl;

        ros::Subscriber _sub_f_gr_br;

        ros::Subscriber _sub_f_gr_fl;

        ros::Subscriber _sub_f_gr_fr;

        EigenRobotState _eig_robot_state;

        std::string modelFile = "/home/fortunato/ros_ws/src/ros_command/urdf/dogbot.urdf";

        iDynTree::ModelLoader mdlLoader;

        iDynTree::KinDynComputations kinDynComp;

        const iDynTree::Model *model;

        //std::string arbitraryFrameName;

        //iDynTree::FrameIndex arbitraryFrameIndex;

        /*std::string _base_link_frame;
        std::string _body_frame;         
        std::string _bodytext_frame;     
        
        std::string _back_left_hip_frame;
        std::string _back_left_upperleg_frame;
        std::string _back_left_lowerleg_frame;
        std::string _back_left_foot_frame;
    
        std::string _back_right_hip_frame;
        std::string _back_right_upperleg_frame;
        std::string _back_right_lowerleg_frame;
        std::string _back_right_foot_frame;
    
        std::string _front_left_hip_frame;
        std::string _front_left_upperleg_frame;
        std::string _front_left_lowerleg_frame;
        std::string _front_left_foot_frame;
    
        std::string _front_right_hip_frame; 
        std::string _front_right_upperleg_frame; 
        std::string _front_right_lowerleg_frame;
        std::string _front_right_foot_frame;*/ 


        Eigen::Matrix<double,18,18> _eig_mass_matrix;
        Eigen::Matrix<double,6,18> _eig_jacobian;
        Eigen::Matrix<double,3,18> _eig_com_jacobian;

        //Eigen::Matrix<double,12,18> _select_matrix;
        //Eigen::Matrix<double,1,18> _v_pseudo; //definition for the pseudo inverse velocity vector otherwise it doesn't work

        //unsigned long int _n_st=4;

        Eigen::Matrix<double,12,18> _j_st_0;
        Eigen::Matrix<double,12,6> _j_st_b_0;
        Eigen::Matrix<double,12,12> _j_st_j_0;

        Eigen::Matrix<double,6,18> _j_st_1;
        Eigen::Matrix<double,6,6> _j_st_b_1;
        Eigen::Matrix<double,6,12> _j_st_j_1;

        Eigen::Matrix<double,12,18> _j_st_1_new;
        Eigen::Matrix<double,12,12> _j_st_j_1_new;

        Eigen::Matrix<double,12,18> _j_st_bar_1_new;
        Eigen::Matrix<double,12,12> _j_st_j_bar_1_new;

        Eigen::Matrix<double,6,18> _j_t_1;

        Eigen::Matrix<double,6,18> _j_t_2;

        Eigen::Matrix<double,6,18> _j_t_3;

        Eigen::Matrix<double,6,18> _j_t_4;

        Eigen::Vector3d _pc;

        Eigen::Vector3d _pb;
        
        Eigen::Vector3d _pbc;

        Eigen::Matrix3d _s;

        Eigen::Matrix<double,6,6> _x; 

        Eigen::Matrix<double,6,6> _m11; 

        Eigen::Matrix<double,6,12> _m12; 

        Eigen::Matrix<double,6,12> _j_s;

        Eigen::Matrix<double,18,18> _t_bar;

        Eigen::Matrix<double,18,18> _m_bar;

        Eigen::Matrix<double,12,12> _m22_bar;

        Eigen::Matrix<double,18,1> _h;

        //Eigen::Matrix<double,18,1> _g;

        //Eigen::Matrix<double,18,18> _c;

        Eigen::Vector3d _pc_dot;

        Eigen::Vector3d _pb_dot;

        Eigen::Vector3d _pbc_dot;

        Eigen::Vector3d _m_dr;

        Eigen::Matrix3d _s_dot;

        Eigen::Matrix<double,6,6> _dx;

        Eigen::Matrix3d _s_mdr;

        Eigen::Matrix<double,6,6> _dmb;

        Eigen::Matrix<double,6,6> _dmb_1;

        Eigen::Matrix<double,6,6> _dmb_2;

        Eigen::Matrix<double,6,12> _dj_s;

        Eigen::Matrix<double,18,18> _t_inv_der;

        Eigen::Matrix<double,18,18> _c_bar;

        Eigen::Matrix<double,12,18> _c2_bar;

        Eigen::Matrix<double,12,18> _j_st_bar_0;
        Eigen::Matrix<double,12,12> _j_st_j_bar_0;
        Eigen::Matrix<double,12,6> _j_st_c_bar_0;

        Eigen::Matrix<double,6,18> _j_st_bar_1;
        Eigen::Matrix<double,6,12> _j_st_j_bar_1;
        Eigen::Matrix<double,6,6> _j_st_c_bar_1;

        Eigen::Matrix<double,12,1> _f_gr_star;

        Eigen::Matrix<double,12,1> _q_dot_dot_des;

        Eigen::Matrix<double,12,1> _jnt_torques_star;

        Eigen::Matrix<double,12,1> _f_gr;        

        Eigen::Matrix<double,18,1> _v_c;

        //iDynTree::Twist _v_c_twist;


        //compute w_com_des

        Eigen::Matrix<double, 6, 6> _k_p;

        Eigen::Matrix<double, 6, 6> _k_d;        

        Eigen::Matrix<double,6,1> _r_c;

        Eigen::Matrix<double,6,1> _r_c_ref;

        Eigen::Matrix<double,6,1> _r_c_dot;

        Eigen::Matrix<double,6,1> _r_c_ref_dot;

        Eigen::Matrix<double,6,1> _r_c_dot_dot;

        Eigen::Matrix<double,6,1> _r_c_ref_dot_dot;

        Eigen::Matrix<double,6,1> _w_com_des;

        //

        

        //compute_optimization

        //Eigen::MatrixXd _sigma;

        //Eigen::Matrix<double,12,1> _f_st_hat_0;

        //Eigen::Matrix<double,6,1> _f_st_hat_1;

        //Eigen::Matrix<double,6,1> _f_sw_hat;

        double _mu=0.4;

        std::array<bool, 4> _contact;

        Eigen::Matrix<double,6,18> _j_sw;
        
        Eigen::Matrix<double,6,6> _j_sw_b;

        Eigen::Matrix<double,6,12> _j_sw_j;

        Eigen::Matrix<double,6,18> _j_sw_bar; 

        Eigen::Matrix<double,6,12> _j_sw_j_bar;

        Eigen::Matrix<double,6,6> _j_sw_c_bar;

        Eigen::Quaterniond q;
 
        Eigen::Matrix<double,12,18> _j_st_dot_0;

        Eigen::Matrix<double,6,1> _j_st_dot_1;

        Eigen::Matrix<double,24,18> _j_st_dot;

        Eigen::Matrix<double,6,1> _j_sw_dot;

        Eigen::Matrix<double,12,18> _j_st_temp;

        Eigen::Matrix<double,6,1> _j_t_1_dot_temp;

        Eigen::Matrix<double,6,1> _j_t_2_dot_temp;

        Eigen::Matrix<double,6,1> _j_t_3_dot_temp;

        Eigen::Matrix<double,6,1> _j_t_4_dot_temp;

        Eigen::Matrix<double,24,1> _j_st_dot_temp;

        Eigen::Matrix<double,24,18> _j;

        Eigen::Matrix<double,24,18> _j_bar;

        //Eigen::Matrix<double,12,18> _j_bar_red;

        Eigen::Matrix<double,3,3> _Tbl;

        Eigen::Matrix<double,3,3> _Tbr;

        Eigen::Matrix<double,3,3> _Tfl;

        Eigen::Matrix<double,3,3> _Tfr;

        

        //

        //planning

        //iDynTree::Vector6 _p;

        //iDynTree::Vector6 _p_dot;

        //iDynTree::Vector6 _p_dot_dot;

        double _duration=0.0;

        //

        //x_sw

        

        Eigen::Matrix<double,6,1> _x_sw_des_dot_dot;

        Eigen::Matrix<double,6,1> _x_sw_des_dot;

        Eigen::Matrix<double,6,1> _x_sw_dot;

        Eigen::Matrix<double,6,1> _x_sw_des;

        Eigen::Matrix<double,6,1> _x_sw;

       

        

        

        

        bool _already_config=false;

        bool _joint_state_available=false;

        bool _base_state_available=false;

        Eigen::Matrix<double,6,1> _accd;

        Eigen::Matrix<double,6,1> _posdelta;

        Eigen::Matrix<double,6,1> _veldelta;

        

        //Time derivative of jacobian
        

        Eigen::Matrix<double,24,1> _Jdqd;

        Eigen::Matrix<double,24,1> _JdqdCOM;

        Eigen::Matrix<double,12,1> _JdqdCOM_lin;

	    Eigen::Matrix<double,12,24> _B;


        double _roll, _pitch, _yaw;

        int _swl1;
	    int _swl2;
	    int _stl1;
	    int _stl2; 

        gazebo::transport::PublisherPtr _pub;
        gazebo::msgs::WorldControl _stepper;

        //ros::Subscriber _distxbr_sub, _distxbl_sub,  _distxfl_sub,  _distxfr_sub, _distxbase_sub;
        
        //double distx_br,  distx_bl,  distx_fl, distx_fr, distx_base;

        std_msgs::Float64 back_left_knee_position;
	    std_msgs::Float64 back_left_pitch_position;
        std_msgs::Float64 back_left_roll_position;
        std_msgs::Float64 back_right_knee_position;
        std_msgs::Float64 back_right_pitch_position;
        std_msgs::Float64 back_right_roll_position;
        std_msgs::Float64 front_left_knee_position;
	    std_msgs::Float64 front_left_pitch_position;
        std_msgs::Float64 front_left_roll_position;
	    std_msgs::Float64 front_right_knee_position;
	    std_msgs::Float64 front_right_pitch_position;
        std_msgs::Float64 front_right_roll_position;

        //double _base_pos_temp[3];

        Eigen::Matrix<double,18,1> _bias_com;

        //bool _overlap=false;

        //double _diff=0.0;

        //Eigen::MatrixXd Mb_Mj;

        iDynTree::Transform  Tbl, Tbr, Tfl, Tfr;

        //bool _ready=false;
        //bool _ref_ready=false;
        //Eigen::Matrix<double,6,1> _pos=Eigen::MatrixXd::Zero(6,1);
        //Eigen::Matrix<double,6,1> _pos_old=Eigen::MatrixXd::Zero(6,1);
        //Eigen::Matrix<double,6,1> _vel=Eigen::MatrixXd::Zero(6,1);
        //Eigen::Matrix<double,6,1> _vel_old=Eigen::MatrixXd::Zero(6,1);
        //bool _already_set_old=false;

        /////////STIMATORE/////////////////////

        /*Eigen::Matrix<double,12,12> _K_1=10*Eigen::MatrixXd::Identity(12,12);;

        Eigen::Matrix<double,12,12> _K_2;
    
        Eigen::Matrix<double,12,12> _K_3;
    
        Eigen::Matrix<double,12,1> _integral_1=Eigen::MatrixXd::Zero(12,1);;
    
        Eigen::Matrix<double,12,1> _integral_2=Eigen::MatrixXd::Zero(12,1);;
    
        Eigen::Matrix<double,12,1> _integral_3=Eigen::MatrixXd::Zero(12,1);;
    
        Eigen::Matrix<double,12,1> _Fgrf=Eigen::MatrixXd::Zero(12,1);;
    
        Eigen::Matrix<double,12,1> _rho=Eigen::MatrixXd::Zero(12,1);;
    
        Eigen::Matrix<double,12,1> _integral_1_dot=Eigen::MatrixXd::Zero(12,1);;
    
        Eigen::Matrix<double,12,1> _integral_2_dot=Eigen::MatrixXd::Zero(12,1);;
    
        Eigen::Matrix<double,12,1> _integral_3_dot=Eigen::MatrixXd::Zero(12,1);;

        bool _already_set=false;

        Eigen::Matrix<double,12,1> _integral_1_old=Eigen::MatrixXd::Zero(12,1);;

        Eigen::Matrix<double,12,1> _integral_2_old=Eigen::MatrixXd::Zero(12,1);;

        Eigen::Matrix<double,12,1> _integral_3_old=Eigen::MatrixXd::Zero(12,1);;

        bool _ready_est=false;

        Eigen::Matrix<double,12,1> _f_ext=Eigen::MatrixXd::Zero(12,1);

        Eigen::Matrix<double,12,1> _gamma_1;

        Eigen::Matrix<double,12,1> _gamma_2;

        Eigen::Matrix<double,12,1> _gamma_3;*/

        //////////////////////////////////////


        double _robot_mass=0.0;

        int index=0;

        bool _yaw_was_positive=false;

        bool _yaw_was_negative=false;

        bool _ang_was_positive=false;

        bool _ang_was_negative=false;

        double _ang_prec=0.0;

        bool _start=false;

        


       

        



        

        







};

#endif