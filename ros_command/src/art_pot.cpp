#include "art_pot.h"

GlobalPlanner::GlobalPlanner(QUADRUPED &quadruped){
    //_sub_odom = _nh.subscribe("/odom", 1, &GlobalPlanner::odom_cb, this);
    _sub_scan = dogbot->get_nh().subscribe("/laser/scan", 1, &GlobalPlanner::laser_cb, this);
    //_pub_vel = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    _x_goal = 0.0;
    _y_goal = 0.0;

    _ka = 0.01;
    _kr = 1;
    _k_damp=0.3;

    _obs_found = false;

    dogbot = &quadruped;
}

void GlobalPlanner::set_goal(double x, double y, double yaw){
    _x_goal = x;
    _y_goal = y;
    _yaw_goal = yaw;
}

//Eigen::Vector2d GlobalPlanner::get_position(){
//    Eigen::Vector2d posizione_x_y;
//    posizione_x_y << _pose.pose.pose.position.x , _pose.pose.pose.position.y;
//    return posizione_x_y;
//}

//void GlobalPlanner::odom_cb(const nav_msgs::Odometry& od){
//    _pose = od;
//}

void GlobalPlanner::laser_cb(const sensor_msgs::LaserScan& laser){

    int start_ind = int (((90-20)/180.0*M_PI) /laser.angle_increment);
    int end_ind = int (((90+20)/180.0*M_PI) /laser.angle_increment);

    bool found = false;
    int i = start_ind;
    double temp_min=1.0;

    temp_x=dogbot->get_pc()(0);
    temp_y=dogbot->get_pc()(1);
    temp_yaw=dogbot->get_yaw();
    

    //double yaw;
    //double temp1,temp2;
    Eigen::Matrix<double,6,1> gradient_temp;

    //tf::Quaternion quaternione(_pose.pose.pose.orientation.x,_pose.pose.pose.orientation.y,_pose.pose.pose.orientation.z,_pose.pose.pose.orientation.w);
    //tf::Matrix3x3 rot(quaternione);
    //rot.getRPY(temp1,temp2,yaw);



    for(int i=start_ind;i<end_ind;i++){
        if(laser.ranges[i]<0.8){  //questo 0.8 è il range di influenza dell'ostacolo
            found=true;
            if(laser.ranges[i]<temp_min){
                temp_min=laser.ranges[i];
                //gradient_temp << cos(i*laser.angle_increment + dogbot->get_yaw()), sin(i*laser.angle_increment + dogbot->get_yaw()); //come calcolare il gradiente?
                gradient_temp << (temp_min - temp_min_old)/(temp_x - temp_x_old), (temp_min - temp_min_old)/(temp_y - temp_y_old), 0.0, 0.0, 0.0, (temp_min - temp_min_old)/(temp_yaw - temp_yaw_old);
                _gradient << -gradient_temp/gradient_temp.norm();
            }
        }
    }
    
    _obs_found = found;
    _obs_distance = temp_min;

    temp_x_old=temp_x;
    temp_y_old=temp_y;
    temp_yaw_old=temp_yaw;
    temp_min_old=temp_min;

}

bool GlobalPlanner::check_local_min(){
    return (_total_force == Eigen::MatrixXd::Zero(6,1) && _e!= Eigen::MatrixXd::Zero(6,1));
}

void GlobalPlanner::compute_repulsive(){

    if(!_obs_found){_repulsive_pot << 0,0;}
    else{_repulsive_pot = -(_kr/pow(_obs_distance,2))*(1/_obs_distance -1/0.8)*_gradient;}  //questa è la forza non il potenziale

}

void GlobalPlanner::compute_total_force(){
    _total_force = _repulsive_pot+_attractive_pot;
}

void GlobalPlanner::compute_attractive(){
    if (_e.norm()>=1.0){        
        _attractive_pot = _ka * _e / _e.norm();
        _attractive_pot -= _k_damp*dogbot->get_r_c_dot();
    }
    else{ 
        _attractive_pot = _ka * _e;
        _attractive_pot -= _k_damp*dogbot->get_r_c_dot();
    }
}

Eigen::Matrix<double,6,1> GlobalPlanner::get_total_force(){

    Eigen::Matrix<double,6,1> current_pos;
    Eigen::Matrix<double,6,1> desired_pos;

    //desired_pos << _x_goal, _y_goal, dogbot->get_pc()(2), 0.0, 0.0, _yaw_goal;

    desired_pos(0,0) = _x_goal;
    desired_pos(1,0) = _y_goal;
    desired_pos(2,0) = 0.40229;//dogbot->get_pc()(2);
    desired_pos(3,0) = 0.0;
    desired_pos(4,0) = 0.0;
    desired_pos(5,0) = _yaw_goal;


    current_pos(0,0) = dogbot->get_pc()(0);
    current_pos(1,0) = dogbot->get_pc()(1);
    current_pos(2,0) = dogbot->get_pc()(2);
    current_pos(3,0) = dogbot->get_roll();
    current_pos(4,0) = dogbot->get_pitch();
    current_pos(5,0) = dogbot->get_yaw();

    _e = desired_pos - current_pos;

    //Calcola forza attrattiva + forza repulsiva = forza totale
    compute_attractive();
    //compute_repulsive();
    compute_total_force();

    std::cout<<"x goal: "<<desired_pos(0,0)<<endl;
    std::cout<<"y goal: "<<desired_pos(1,0)<<endl;
    std::cout<<"z goal: "<<desired_pos(2,0)<<endl;
    std::cout<<"roll: "<<desired_pos(3,0)<<endl;
    std::cout<<"pitch: "<<desired_pos(4,0)<<endl;
    std::cout<<"yaw: "<<desired_pos(5,0)<<endl;


    std::cout<<"current pos: "<<current_pos<<endl;

    return _total_force;
}

/*void GlobalPlanner::loop(){
	ros::Rate r(250);

    Eigen::Vector2d current_pos;
    Eigen::Vector2d desired_pos;

    geometry_msgs::Twist q_dot;

    while(ros::ok()){

        desired_pos << _x_goal, _y_goal;
        current_pos = get_position();

        _e = desired_pos - current_pos;

        //Calcola forza attrattiva + forza repulsiva = forza totale
        compute_attractive();
        compute_repulsive();
        compute_total_force();


        if(check_local_min() ){
            //Spegni algoritmo e genera rand
        }

        //Publish cmd_vel calcolate con potenziali artificiali
        _pub_vel.publish(q_dot);
        
        
	r.sleep();
	}
}

void GlobalPlanner::run() {
	boost::thread ctrl_loop_t ( &GlobalPlanner::loop, this);
	ros::spin();
    
}



int main(int argc, char** argv){
    ros::init(argc, argv, "globalplanner");
    GlobalPlanner planner;

    planner.run();
    return 0;
}*/




