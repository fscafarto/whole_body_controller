#include "for_dyn.h"

QUADRUPED* quad;

PLANNING* plan;

OPTIMAL* opti;

//GlobalPlanner* glob_plan;

SCANNER* scan;



QUADRUPED::QUADRUPED(){

    _sub_joint_pos = _nh.subscribe<sensor_msgs::JointState>("/dogbot/joint_states", 1,  &QUADRUPED::joint_pos_cb, this);

    _sub_box_state = _nh.subscribe("/gazebo/model_states", 1, &QUADRUPED::box_states_callback, this);

    _sub_f_gr_bl = _nh.subscribe("/dogbot/back_left_contactsensor_state", 1, &QUADRUPED::f_gr_bl_cb, this);

    _sub_f_gr_br = _nh.subscribe("/dogbot/back_right_contactsensor_state", 1, &QUADRUPED::f_gr_br_cb, this);

    _sub_f_gr_fl = _nh.subscribe("/dogbot/front_left_contactsensor_state", 1, &QUADRUPED::f_gr_fl_cb, this);

    _sub_f_gr_fr = _nh.subscribe("/dogbot/front_right_contactsensor_state", 1, &QUADRUPED::f_gr_fr_cb, this);

    _pub_joint_torque_blk = _nh.advertise<std_msgs::Float64>( "/dogbot/back_left_knee_effort_controller/command", 1);
    _pub_joint_torque_blp = _nh.advertise<std_msgs::Float64>("/dogbot/back_left_pitch_effort_controller/command", 1);
    _pub_joint_torque_blr = _nh.advertise<std_msgs::Float64>( "/dogbot/back_left_roll_effort_controller/command", 1);
    _pub_joint_torque_brk = _nh.advertise<std_msgs::Float64>("/dogbot/back_right_knee_effort_controller/command", 1);
    _pub_joint_torque_brp = _nh.advertise<std_msgs::Float64>("/dogbot/back_right_pitch_effort_controller/command", 1);
    _pub_joint_torque_brr = _nh.advertise<std_msgs::Float64>("/dogbot/back_right_roll_effort_controller/command", 1);    
    _pub_joint_torque_flk = _nh.advertise<std_msgs::Float64>("/dogbot/front_left_knee_effort_controller/command", 1);
    _pub_joint_torque_flp = _nh.advertise<std_msgs::Float64>("/dogbot/front_left_pitch_effort_controller/command", 1);
    _pub_joint_torque_flr = _nh.advertise<std_msgs::Float64>("/dogbot/front_left_roll_effort_controller/command", 1);
    _pub_joint_torque_frk = _nh.advertise<std_msgs::Float64>("/dogbot/front_right_knee_effort_controller/command", 1);
    _pub_joint_torque_frp = _nh.advertise<std_msgs::Float64>("/dogbot/front_right_pitch_effort_controller/command", 1);
    _pub_joint_torque_frr = _nh.advertise<std_msgs::Float64>("/dogbot/front_right_roll_effort_controller/command", 1);


    init_model();


    
}

void QUADRUPED::joint_pos_cb( const sensor_msgs::JointState::ConstPtr& data ){



    _eig_robot_state.jointPos(0,0)=data->position[11];
    _eig_robot_state.jointPos(1,0)=data->position[8];
    _eig_robot_state.jointPos(2,0)=data->position[5];
    _eig_robot_state.jointPos(3,0)=data->position[2];
    _eig_robot_state.jointPos(4,0)=data->position[1];
    _eig_robot_state.jointPos(5,0)=data->position[0];
    _eig_robot_state.jointPos(6,0)=data->position[4];
    _eig_robot_state.jointPos(7,0)=data->position[3];
    _eig_robot_state.jointPos(8,0)=data->position[7];
    _eig_robot_state.jointPos(9,0)=data->position[6];
    _eig_robot_state.jointPos(10,0)=data->position[10];
    _eig_robot_state.jointPos(11,0)=data->position[9];

    _eig_robot_state.jointVel(0,0)=data->velocity[11];
    _eig_robot_state.jointVel(1,0)=data->velocity[8];
    _eig_robot_state.jointVel(2,0)=data->velocity[5];
    _eig_robot_state.jointVel(3,0)=data->velocity[2];
    _eig_robot_state.jointVel(4,0)=data->velocity[1];
    _eig_robot_state.jointVel(5,0)=data->velocity[0];
    _eig_robot_state.jointVel(6,0)=data->velocity[4];
    _eig_robot_state.jointVel(7,0)=data->velocity[3];
    _eig_robot_state.jointVel(8,0)=data->velocity[7];
    _eig_robot_state.jointVel(9,0)=data->velocity[6];
    _eig_robot_state.jointVel(10,0)=data->velocity[10];
    _eig_robot_state.jointVel(11,0)=data->velocity[9]; 

    _joint_state_available=true; 

}


void QUADRUPED::box_states_callback( const gazebo_msgs::ModelStates &box_state_current ){

    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    
    
    double _base_orie_temp[4];
    double _base_lin_vel_temp[3];
    double _base_ang_vel_temp[3];


    int box_index = -1;
    std::vector<std::string> model_names = box_state_current.name;

    for(size_t i = 0; i < model_names.size(); i++)
    {
        if(model_names[i] == "dogbot")
            box_index = i;
    }
    
    
    pose = box_state_current.pose[box_index];

    //std::cerr<<box_index<<std::endl;
    
    //Store the position of the base
    _pb[0]=pose.position.x;
    _pb[1]=pose.position.y;
    _pb[2]=pose.position.z;
    //std::cerr<<basePos_temp_new[0]<<std::endl;

    //Store the orientation of the base (quaternion) 
    _base_orie_temp[0]=pose.orientation.x;
    _base_orie_temp[1]=pose.orientation.y;
    _base_orie_temp[2]=pose.orientation.z;
    _base_orie_temp[3]=pose.orientation.w;

    
    twist = box_state_current.twist[box_index];

    //Store the linear velocity of the base
    _eig_robot_state.baseVel[0]=twist.linear.x;
    _eig_robot_state.baseVel[1]=twist.linear.y;
    _eig_robot_state.baseVel[2]=twist.linear.z;

    //Store the angular velocity of the base
    _eig_robot_state.baseVel[3]=twist.angular.x;
    _eig_robot_state.baseVel[4]=twist.angular.y;
    _eig_robot_state.baseVel[5]=twist.angular.z;

    tf::Quaternion q(_base_orie_temp[0], _base_orie_temp[1], _base_orie_temp[2],  _base_orie_temp[3]);

    q.normalize();  
    tf::Matrix3x3 m(q);  

    Eigen::Matrix3d R;
    tf::matrixTFToEigen(m,R);

    if(_yaw<0.0 && std::abs(_yaw)>3.0){
        _yaw_was_negative=true;
    }

    if(_yaw>0.0 && std::abs(_yaw)>3.0){
        _yaw_was_positive=true;
    }

    if(std::abs(_yaw)>=2*3.14){
        _yaw=0.0;
        _yaw_was_negative=false;
        _yaw_was_positive=false;

    }

    tf::Matrix3x3(q).getRPY(_roll, _pitch, _yaw);

    /*std::cerr<<"posa"<<endl;
    std::cerr<<_base_pos_temp[0]<<endl;
    std::cerr<<_base_pos_temp[1]<<endl;
    std::cerr<<_base_pos_temp[2]<<endl;
    std::cerr<<_roll<<endl;
    std::cerr<<_pitch<<endl;
    std::cerr<<_yaw<<endl;*/

    if(_yaw>0.0 && _yaw_was_negative){
        _yaw=_yaw-2*M_PI;
    }

    if(_yaw<0.0 && _yaw_was_positive){
        _yaw=_yaw+2*M_PI;
    }

    if(std::abs(_yaw)>=2*3.14){
        _yaw=0.0;
        _yaw_was_negative=false;
        _yaw_was_positive=false;

    }

    

    

    _eig_robot_state.world_H_base<<R(0,0), R(0,1), R(0,2), _pb[0],
                                   R(1,0), R(1,1), R(1,2), _pb[1],
                                   R(2,0), R(2,1), R(2,2), _pb[2],
                                   0,      0,      0,      1;

    /*_eig_robot_state.world_H_base.block(0,0,3,3)= R;
    _eig_robot_state.world_H_base.block(0,3,3,1)= _base_pos_temp;
    _eig_robot_state.world_H_base.block(3,0,1,3)= Eigen::MatrixXd::Zero(1,3);
    _eig_robot_state.world_H_base.block(3,3,1,1)= 1;*/

    //_eig_robot_state.v.block(0,0,6,1)=_eig_robot_state.baseVel;
    //_eig_robot_state.v.block(6,0,12,1)=_eig_robot_state.jointVel;

    _base_state_available=true;

    //std::cout<<"Matrice di rotazione R: "<<R<<endl;
    //std::cout<<"roll: "<<_roll<<endl;
    //std::cout<<"pitch: "<<_pitch<<endl;
    //std::cout<<"yaw: "<<_yaw<<endl;

    
}

void QUADRUPED::f_gr_fr_cb( const gazebo_msgs::ContactsState::ConstPtr& box_state_current ){

    //geometry_msgs::Wrench wrench;
    
    //wrench=box_state_current.states.total_wrench;


    if(box_state_current->states.size()>0){

        _contact[0]=true;

        _f_gr(0,0)=box_state_current->states[0].total_wrench.force.x;
        _f_gr(1,0)=box_state_current->states[0].total_wrench.force.y;
        _f_gr(2,0)=box_state_current->states[0].total_wrench.force.z;



    }else{

        _contact[0]=false;

        _f_gr(0,0)=0.0;
        _f_gr(1,0)=0.0;
        _f_gr(2,0)=0.0;

    }

}

void QUADRUPED::f_gr_fl_cb( const gazebo_msgs::ContactsState::ConstPtr& box_state_current ){

   if(box_state_current->states.size()>0){

       _contact[1]=true;

        _f_gr(3,0)=box_state_current->states[0].total_wrench.force.x;
        _f_gr(4,0)=box_state_current->states[0].total_wrench.force.y;
        _f_gr(5,0)=box_state_current->states[0].total_wrench.force.z;



    }else{

        _contact[1]=false;

        _f_gr(3,0)=0.0;
        _f_gr(4,0)=0.0;
        _f_gr(5,0)=0.0;

    }

}

void QUADRUPED::f_gr_bl_cb( const gazebo_msgs::ContactsState::ConstPtr& box_state_current ){

    if(box_state_current->states.size()>0){

        _contact[2]=true;

        _f_gr(6,0)=box_state_current->states[0].total_wrench.force.x;
        _f_gr(7,0)=box_state_current->states[0].total_wrench.force.y;
        _f_gr(8,0)=box_state_current->states[0].total_wrench.force.z;

       
    }else{

        _contact[2]=false;

        _f_gr(6,0)=0.0;
        _f_gr(7,0)=0.0;
        _f_gr(8,0)=0.0;
    }

}

void QUADRUPED::f_gr_br_cb( const gazebo_msgs::ContactsState::ConstPtr& box_state_current ){

    if(box_state_current->states.size()>0){

        _contact[3]=true;

        _f_gr(9,0)=box_state_current->states[0].total_wrench.force.x;
        _f_gr(10,0)=box_state_current->states[0].total_wrench.force.y;
        _f_gr(11,0)=box_state_current->states[0].total_wrench.force.z;


    }else{
        _contact[3]=false;
        
        _f_gr(9,0)=0.0;
        _f_gr(10,0)=0.0;
        _f_gr(11,0)=0.0;

    }

}


int QUADRUPED::stand_phase(){   

    //ros::Rate r(1000); 
   

    while((ros::Time::now()-_begin).toSec() < _duration ){   

    double t = (ros::Time::now()-_begin).toSec(); 

    //std::cout<<"prima dei riferimenti"<<endl;

    //std::cout<<"time: "<<t<<endl;

    //if(t>0.8){
    //    t=0.8;
    //}

    _r_c_ref<<plan->get_solution().base_linear_->GetPoint(t).p(), plan->get_solution().base_angular_->GetPoint(t).p();
    //std::cout<<"dopo rcref"<<endl;
    _r_c_ref_dot<<plan->get_solution().base_linear_->GetPoint(t).v(), plan->get_solution().base_angular_->GetPoint(t).v();
    //std::cout<<"dopo rcrefdot"<<endl;
    _r_c_ref_dot_dot<<plan->get_solution().base_linear_->GetPoint(t).a(), plan->get_solution().base_angular_->GetPoint(t).a();
    //std::cout<<"dopo rereefdotdot"<<endl;

    //std::cout<<"riferimenti stand"<<endl;
    //std::cout<<_r_c_ref<<endl;
    //std::cout<<"riferimenti dot stand"<<endl;
    //std::cout<<_r_c_ref_dot<<endl;
    //std::cout<<"riferimenti dot dot stand"<<endl;
    //std::cout<<_r_c_ref_dot_dot<<endl;

    //std::cout<<"prima di update: "<<ros::Time::now().toSec()<<endl;

    update(0);

    //std::cout<<"dopo di update: "<<ros::Time::now().toSec()<<endl;


    

   

    ////////////COMPUTE F_GR_STAR & Q_DOT_DOT_DES//////////  


    //pauseGazebo.call(pauseSrv);

    //std::cout<<"entro in opti"<<ros::Time::now().toSec()<<endl;
    opti->compute_optimization(0);
    //std::cout<<"esco da opti"<<ros::Time::now().toSec()<<endl;

    //unpauseGazebo.call(pauseSrv);

    //std::cout<<"correnti stand"<<endl;
    //std::cout<<_r_c<<endl;
    //std::cout<<"correnti dot stand"<<endl;
    //std::cout<<_r_c_dot<<endl;
    //std::cout<<"correnti dot dot stand"<<endl;
    //std::cout<<_r_c_dot_dot<<endl;
 
    //////////////////////////////////////////////////////


    //////////COMPUTE JNT_TORQUES_STAR///////////////////

    //*****approccio alternativo per calcolare uno dei termini della dinamica inversa*****//

      
    _jnt_torques_star=_m22_bar*_q_dot_dot_des + _bias_com.block(6,0,12,1) - _j_st_j_bar_0.transpose()*_f_gr_star;

    //std::cout<<"coppie calcolate: "<<ros::Time::now().toSec()<<endl;

    //coma calcolavo prima la dinamica inversa
    //_jnt_torques_star=_m22_bar*_q_dot_dot_des + _c2_bar*_v_c - _j_st_j_bar_0.transpose()*_f_gr_star;

    ////////////////////////////////////////////////////

    /*std::cerr<<"Desired joint acceleration:"<<std::endl;
    std::cerr<<_q_dot_dot_des<<std::endl;

    std::cerr<<"Desired ground reaction forces:"<<std::endl;
    std::cerr<<_f_gr_star<<std::endl;

    std::cerr<<"Torques:"<<std::endl;
    std::cerr<<_jnt_torques_star<<std::endl;*/ 


    //std::cout<<"pubblico"<<endl;
    back_left_knee_position.data=_jnt_torques_star(5,0);
    back_left_pitch_position.data=_jnt_torques_star(4,0);
    back_left_roll_position.data=_jnt_torques_star(3,0);
	back_right_knee_position.data=_jnt_torques_star(7,0);
    back_right_pitch_position.data=_jnt_torques_star(6,0);
    back_right_roll_position.data=_jnt_torques_star(2,0);	
	front_left_knee_position.data=_jnt_torques_star(9,0);
    front_left_pitch_position.data=_jnt_torques_star(8,0);
    front_left_roll_position.data=_jnt_torques_star(1,0);
    front_right_knee_position.data=_jnt_torques_star(11,0);
    front_right_pitch_position.data=_jnt_torques_star(10,0);
    front_right_roll_position.data=_jnt_torques_star(0,0);

    //std::cout<<"prima fase del publish "<<ros::Time::now().toSec()<<endl;

    _pub_joint_torque_blk.publish(back_left_knee_position);
	_pub_joint_torque_blp.publish(back_left_pitch_position);
	_pub_joint_torque_blr.publish(back_left_roll_position);
    _pub_joint_torque_brk.publish(back_right_knee_position);
	_pub_joint_torque_brp.publish(back_right_pitch_position);
	_pub_joint_torque_brr.publish(back_right_roll_position);
    _pub_joint_torque_flk.publish(front_left_knee_position);
	_pub_joint_torque_flp.publish(front_left_pitch_position);
	_pub_joint_torque_flr.publish(front_left_roll_position);
    _pub_joint_torque_frk.publish(front_right_knee_position);
	_pub_joint_torque_frp.publish(front_right_pitch_position);
	_pub_joint_torque_frr.publish(front_right_roll_position);
    //std::cout<<"fine stand phase"<<endl;
    
    
    //if(_overlap){
    //    pauseGazebo.call(pauseSrv);
    //    _pub->Publish(_stepper);
    //    ros::spinOnce();
    //    //_overlap=false;
    //    //unpauseGazebo.call(unpauseSrv);
    //}else{
    //    unpauseGazebo.call(unpauseSrv);
    //}

    //_pub->Publish(_stepper);
    
    //ros::spinOnce();

   

    
    
    //r.sleep();

  }

    return 0;
}


int QUADRUPED::swing_phase(){

    bool endswing= false;

    bool takeoff=false;

    //&& !endswing
  

    //ros::Rate r(1000);   


    while((ros::Time::now()-_begin).toSec() < _duration && !endswing ){ 


    double t = (ros::Time::now()-_begin).toSec(); 

    //ros::Duration d(_diff);
    //if(_overlap){
    //    t = (ros::Time::now() - _begin - d).toSec();
    //    _overlap=false;
    //}

    _r_c_ref<<plan->get_solution().base_linear_->GetPoint(t).p(), plan->get_solution().base_angular_->GetPoint(t).p();
    _r_c_ref_dot<<plan->get_solution().base_linear_->GetPoint(t).v(), plan->get_solution().base_angular_->GetPoint(t).v();
    _r_c_ref_dot_dot<<plan->get_solution().base_linear_->GetPoint(t).a(), plan->get_solution().base_angular_->GetPoint(t).a();
    
    //std::cout<<"riferimenti swing1"<<endl;
    //std::cout<<_r_c_ref<<endl;
    //std::cout<<"riferimenti dot swing1"<<endl;
    //std::cout<<_r_c_ref_dot<<endl;
    //std::cout<<"riferimenti dot dot swing1"<<endl;
    //std::cout<<_r_c_ref_dot_dot<<endl;



    
    _accd<< plan->get_solution().ee_motion_.at(1)->GetPoint(t).a(),
           plan->get_solution().ee_motion_.at(2)->GetPoint(t).a();

    
    _posdelta<< plan->get_solution().ee_motion_.at(1)->GetPoint(t).p()-quad->getBRpos(),
               plan->get_solution().ee_motion_.at(2)->GetPoint(t).p()-quad->getFLpos();

    
    _veldelta<< plan->get_solution().ee_motion_.at(1)->GetPoint(t).v()-quad->getBRvel(),
               plan->get_solution().ee_motion_.at(2)->GetPoint(t).v()-quad->getFLvel();

    
    update(1);


    

    /*Tbl=kinDynComp.getWorldTransform(8); //secondo me qui dovrebbe essere 8 e non 11
    _Tbl=toEigen(Tbl.getRotation());

    Tbr=kinDynComp.getWorldTransform(11);
    _Tbr=toEigen(Tbr.getRotation());

    Tfl=kinDynComp.getWorldTransform(14);
    _Tfl=toEigen(Tfl.getRotation());

    Tfr=kinDynComp.getWorldTransform(17);
    _Tfr=toEigen(Tfr.getRotation());*/

    ////////////COMPUTE F_GR_STAR & Q_DOT_DOT_DES//////////

    //pauseGazebo.call(pauseSrv);

    //std::cerr<<"entro in opti da swing"<<ros::Time::now()<<endl;

    opti->compute_optimization(1);

    //std::cerr<<"esco da opti da swing"<<ros::Time::now()<<endl;

    //unpauseGazebo.call(pauseSrv);

    

    //std::cout<<"correnti stand"<<endl;
    //std::cout<<_r_c<<endl;
    //std::cout<<"correnti dot stand"<<endl;
    //std::cout<<_r_c_dot<<endl;
    //std::cout<<"correnti dot dot stand"<<endl;
    //std::cout<<_r_c_dot_dot<<endl;

 
    //////////////////////////////////////////////////////


    //////////COMPUTE JNT_TORQUES_STAR//////

    _jnt_torques_star=_m22_bar*_q_dot_dot_des + _bias_com.block(6,0,12,1) - _j_st_j_bar_1_new.transpose()*_f_gr_star;  


    //////////////////////////////////////

    /*std::cerr<<"Desired joint acceleration:"<<std::endl;
    std::cerr<<_q_dot_dot_des<<std::endl;

    std::cerr<<"Desired ground reaction forces:"<<std::endl;
    std::cerr<<_f_gr_star<<std::endl;*/

    //std::cout<<"pubblico"<<endl;
    back_left_knee_position.data=_jnt_torques_star(5,0);
    back_left_pitch_position.data=_jnt_torques_star(4,0);
    back_left_roll_position.data=_jnt_torques_star(3,0);
	
	back_right_knee_position.data=_jnt_torques_star(7,0);
    back_right_pitch_position.data=_jnt_torques_star(6,0);
    back_right_roll_position.data=_jnt_torques_star(2,0);
		
	front_left_knee_position.data=_jnt_torques_star(9,0);
    front_left_pitch_position.data=_jnt_torques_star(8,0);
    front_left_roll_position.data=_jnt_torques_star(1,0);

    front_right_knee_position.data=_jnt_torques_star(11,0);
    front_right_pitch_position.data=_jnt_torques_star(10,0);
    front_right_roll_position.data=_jnt_torques_star(0,0);

    _pub_joint_torque_blk.publish(back_left_knee_position);
	_pub_joint_torque_blp.publish(back_left_pitch_position);
	_pub_joint_torque_blr.publish(back_left_roll_position);
    _pub_joint_torque_brk.publish(back_right_knee_position);
	_pub_joint_torque_brp.publish(back_right_pitch_position);
	_pub_joint_torque_brr.publish(back_right_roll_position);
    _pub_joint_torque_flk.publish(front_left_knee_position);
	_pub_joint_torque_flp.publish(front_left_pitch_position);
	_pub_joint_torque_flr.publish(front_left_roll_position);
    _pub_joint_torque_frk.publish(front_right_knee_position);
	_pub_joint_torque_frp.publish(front_right_pitch_position);
	_pub_joint_torque_frr.publish(front_right_roll_position);

    

    //std::cout<<"fine swing phase"<<endl;

    //if(_overlap){
    //    pauseGazebo.call(pauseSrv);
    //    _pub->Publish(_stepper);
    //    ros::spinOnce();
    //    _overlap=false;
    //    unpauseGazebo.call(unpauseSrv);
    //}

    //_pub->Publish(_stepper);

    //ros::spinOnce();
    
    if(!_contact[1] && !_contact[3] ){
        takeoff=true;
    }

    if(takeoff && _contact[1] && _contact[3]){
        endswing=true;
    }
    
    //r.sleep();

    

  }

    return 0;
}

int QUADRUPED::swing_phase2(){   

    bool takeoff=false;

    bool endswing=false;

    

    //ros::Rate r(1000);    

    

    while((ros::Time::now()-_begin).toSec() < _duration && !endswing){ 

    
    
    double t = (ros::Time::now()-_begin).toSec();

    _r_c_ref<<plan->get_solution().base_linear_->GetPoint(t).p(), plan->get_solution().base_angular_->GetPoint(t).p();
    _r_c_ref_dot<<plan->get_solution().base_linear_->GetPoint(t).v(), plan->get_solution().base_angular_->GetPoint(t).v();
    _r_c_ref_dot_dot<<plan->get_solution().base_linear_->GetPoint(t).a(), plan->get_solution().base_angular_->GetPoint(t).a();

    //std::cout<<"riferimenti swing1"<<endl;
    //std::cout<<_r_c_ref<<endl;
    //std::cout<<"riferimenti dot swing1"<<endl;
    //std::cout<<_r_c_ref_dot<<endl;
    //std::cout<<"riferimenti dot dot swing1"<<endl;
    //std::cout<<_r_c_ref_dot_dot<<endl;



    
    _accd<< plan->get_solution().ee_motion_.at(0)->GetPoint(t).a(),
           plan->get_solution().ee_motion_.at(3)->GetPoint(t).a();

    
    _posdelta<< plan->get_solution().ee_motion_.at(0)->GetPoint(t).p()-quad->getBLpos(),
               plan->get_solution().ee_motion_.at(3)->GetPoint(t).p()-quad->getFRpos();

    
    _veldelta<< plan->get_solution().ee_motion_.at(0)->GetPoint(t).v()-quad->getBLvel(),
               plan->get_solution().ee_motion_.at(3)->GetPoint(t).v()-quad->getFRvel();



    update(2);


    

    /*Tbl=kinDynComp.getWorldTransform(8); //secondo me qui dovrebbe essere 8 e non 11
    _Tbl=toEigen(Tbl.getRotation());

    Tbr=kinDynComp.getWorldTransform(11);
    _Tbr=toEigen(Tbr.getRotation());

    Tfl=kinDynComp.getWorldTransform(14);
    _Tfl=toEigen(Tfl.getRotation());

    Tfr=kinDynComp.getWorldTransform(17);
    _Tfr=toEigen(Tfr.getRotation());*/

    ////////////COMPUTE F_GR_STAR & Q_DOT_DOT_DES//////////

    //pauseGazebo.call(pauseSrv);

    //std::cerr<<"entro in opti da swing2"<<ros::Time::now()<<endl;

    opti->compute_optimization(2);

    //std::cerr<<"esco da opti da swing2"<<ros::Time::now()<<endl;

    //unpauseGazebo.call(pauseSrv);
    

 
    //////////////////////////////////////////////////////


    //////////COMPUTE JNT_TORQUES_STAR//////

    _jnt_torques_star=_m22_bar*_q_dot_dot_des + _bias_com.block(6,0,12,1) - _j_st_j_bar_1_new.transpose()*_f_gr_star;
    

    //////////////////////////////////////

    /*std::cerr<<"Desired joint acceleration:"<<std::endl;
    std::cerr<<_q_dot_dot_des<<std::endl;

    std::cerr<<"Desired ground reaction forces:"<<std::endl;
    std::cerr<<_f_gr_star<<std::endl;*/

    //std::cout<<"pubblico"<<endl;
    back_left_knee_position.data=_jnt_torques_star(5,0);
    back_left_pitch_position.data=_jnt_torques_star(4,0);
    back_left_roll_position.data=_jnt_torques_star(3,0);
	
	back_right_knee_position.data=_jnt_torques_star(7,0);
    back_right_pitch_position.data=_jnt_torques_star(6,0);
    back_right_roll_position.data=_jnt_torques_star(2,0);
		
	front_left_knee_position.data=_jnt_torques_star(9,0);
    front_left_pitch_position.data=_jnt_torques_star(8,0);
    front_left_roll_position.data=_jnt_torques_star(1,0);

    front_right_knee_position.data=_jnt_torques_star(11,0);
    front_right_pitch_position.data=_jnt_torques_star(10,0);
    front_right_roll_position.data=_jnt_torques_star(0,0);

    _pub_joint_torque_blk.publish(back_left_knee_position);
	_pub_joint_torque_blp.publish(back_left_pitch_position);
	_pub_joint_torque_blr.publish(back_left_roll_position);
    _pub_joint_torque_brk.publish(back_right_knee_position);
	_pub_joint_torque_brp.publish(back_right_pitch_position);
	_pub_joint_torque_brr.publish(back_right_roll_position);
    _pub_joint_torque_flk.publish(front_left_knee_position);
	_pub_joint_torque_flp.publish(front_left_pitch_position);
	_pub_joint_torque_flr.publish(front_left_roll_position);
    _pub_joint_torque_frk.publish(front_right_knee_position);
	_pub_joint_torque_frp.publish(front_right_pitch_position);
	_pub_joint_torque_frr.publish(front_right_roll_position);

    


    //if(_overlap){
    //    pauseGazebo.call(pauseSrv);
    //    _pub->Publish(_stepper);
    //    ros::spinOnce();
    //    _overlap=false;
    //    unpauseGazebo.call(unpauseSrv);
    //}

    //_pub->Publish(_stepper);

    //ros::spinOnce();

    if(!_contact[0] && !_contact[2] ){
        takeoff=true;
    }

    if(takeoff && _contact[0] && _contact[2]){
        endswing=true;
    }

    
    
    //r.sleep();

    


  }

    return 0;
}





int QUADRUPED::compute_j_st(int flag){

    bool ok = kinDynComp.getFrameFreeFloatingJacobian(8, iDynTree::make_matrix_view(_j_t_1));

    if (!ok)
    {
        std::cerr << "Wrong frame index or wrong size passed to KinDynComputations::getFrameFreeFloatingJacobian" << std::endl;
        return EXIT_FAILURE;
    }

    ok = kinDynComp.getFrameFreeFloatingJacobian(11, iDynTree::make_matrix_view(_j_t_2));

    if (!ok)
    {
        std::cerr << "Wrong frame index or wrong size passed to KinDynComputations::getFrameFreeFloatingJacobian" << std::endl;
        return EXIT_FAILURE;
    }

    ok = kinDynComp.getFrameFreeFloatingJacobian(14, iDynTree::make_matrix_view(_j_t_3));

    if (!ok)
    {
        std::cerr << "Wrong frame index or wrong size passed to KinDynComputations::getFrameFreeFloatingJacobian" << std::endl;
        return EXIT_FAILURE;
    }

    ok = kinDynComp.getFrameFreeFloatingJacobian(17, iDynTree::make_matrix_view(_j_t_4));

    if (!ok)
    {
        std::cerr << "Wrong frame index or wrong size passed to KinDynComputations::getFrameFreeFloatingJacobian" << std::endl;
        return EXIT_FAILURE;
    }

    ///////////////////CONTACT JACOBIAN (ONLY THE FIRST 3 COMPONENTS)//////////////////////
    /*_j_st<<_j_t_1.block(0,0,3,6+model->getNrOfDOFs()),
          _j_t_2.block(0,0,3,6+model->getNrOfDOFs()),
          _j_t_3.block(0,0,3,6+model->getNrOfDOFs()),
          _j_t_4.block(0,0,3,6+model->getNrOfDOFs());*/

    _j.block(0,0,6,18)=_j_t_1;
    _j.block(6,0,6,18)=_j_t_2;
    _j.block(12,0,6,18)=_j_t_3;
    _j.block(18,0,6,18)=_j_t_4;

    

    /*_j_st.block(0,0,3,6+12)=_j_t_1.block(0,0,3,6+12);
    _j_st.block(3,0,3,6+12)=_j_t_2.block(0,0,3,6+12);
    _j_st.block(6,0,3,6+12)=_j_t_3.block(0,0,3,6+12);
    _j_st.block(9,0,3,6+12)=_j_t_4.block(0,0,3,6+12);*/

    _j_st_temp.block(0,0,3,6+12)=_j_t_1.block(0,0,3,18);
    _j_st_temp.block(3,0,3,6+12)=_j_t_2.block(0,0,3,18);
    _j_st_temp.block(6,0,3,6+12)=_j_t_3.block(0,0,3,18);
    _j_st_temp.block(9,0,3,6+12)=_j_t_4.block(0,0,3,18);

    _j_st_0=_j_st_temp;

    if(flag==0){  

        _j_st_0=_j_st_temp;      

       
        //_j_st_0.block(0,0,3,18)=_j_st_temp.block(0,0,3,18);
        //_j_st_0.block(3,0,3,18)=_j_st_temp.block(3,0,3,18);
        //_j_st_0.block(6,0,3,18)=_j_st_temp.block(6,0,3,18);
        //_j_st_0.block(9,0,3,18)=_j_st_temp.block(9,0,3,18);
               

    }    

    //std::cerr<<"check1"<<endl; 

    if(flag==1 || flag==2){

        if(flag==1){

            _stl1=0;
            _swl1=3;
            _swl2=6;
            _stl2=9;

        }

        if(flag==2){
    
            _swl1=0;
            _stl1=3;
            _stl2=6;
            _swl2=9;
    
        }

        
         
    _j_st_1.block(0,0,3,18)=_j_st_temp.block(_stl1,0,3,18);

    _j_st_1.block(3,0,3,18)=_j_st_temp.block(_stl2,0,3,18);
            
    //std::cerr<<"check2"<<endl;

        
    _j_sw.block(0,0,3,18)=_j_st_temp.block(_swl1,0,3,18);

    _j_sw.block(3,0,3,18)=_j_st_temp.block(_swl2,0,3,18);

                
    //std::cerr<<"check22"<<endl;

    _j_st_1_new.block(_swl1,0,3,18)=Eigen::MatrixXd::Zero(3,18);

    _j_st_1_new.block(_stl1,0,3,18)=_j_st_temp.block(_stl1,0,3,18);

    _j_st_1_new.block(_swl2,0,3,18)=Eigen::MatrixXd::Zero(3,18);

    _j_st_1_new.block(_stl2,0,3,18)=_j_st_temp.block(_stl2,0,3,18);
            
       
    //std::cerr<<"check3"<<endl;

    }

    

    //_j_st_dot_temp.block(0,0,6,1)=_j_t_1_dot_temp.block(0,0,6,1);
    //_j_st_dot_temp.block(6,0,6,1)=_j_t_2_dot_temp.block(0,0,6,1);
    //_j_st_dot_temp.block(12,0,6,1)=_j_t_3_dot_temp.block(0,0,6,1);
    //_j_st_dot_temp.block(18,0,6,1)=_j_t_4_dot_temp.block(0,0,6,1);

    _j_t_1_dot_temp = iDynTree::toEigen(kinDynComp.getFrameBiasAcc(8));
    _j_t_2_dot_temp = iDynTree::toEigen(kinDynComp.getFrameBiasAcc(11));    
    _j_t_3_dot_temp = iDynTree::toEigen(kinDynComp.getFrameBiasAcc(14));    
    _j_t_4_dot_temp = iDynTree::toEigen(kinDynComp.getFrameBiasAcc(17));  

    /////////////stile vivo////////////////////////////     
    
    _Jdqd.block(0,0,6,1)=_j_t_1_dot_temp;
    _Jdqd.block(6,0,6,1)=_j_t_2_dot_temp;
    _Jdqd.block(12,0,6,1)=_j_t_3_dot_temp;
    _Jdqd.block(18,0,6,1)=_j_t_4_dot_temp;
    
    _JdqdCOM=_Jdqd+_j*_t_inv_der*_v_c;
    _JdqdCOM_lin= Eigen::MatrixXd::Zero(12,1);
    _JdqdCOM_lin=_B*_JdqdCOM;

    _j_st_dot_0.block(0,0,3,1)=_JdqdCOM_lin.block(0,0,3,1);
    _j_st_dot_0.block(3,0,3,1)=_JdqdCOM_lin.block(3,0,3,1);
    _j_st_dot_0.block(6,0,3,1)=_JdqdCOM_lin.block(6,0,3,1);
    _j_st_dot_0.block(9,0,3,1)=_JdqdCOM_lin.block(9,0,3,1);

    ////////////////////////////////////////////////

    /*_j_st_dot.block(0,0,6,18)= _j_t_1_dot_temp*_v_pseudo;
    _j_st_dot.block(6,0,6,18)= _j_t_2_dot_temp*_v_pseudo;
    _j_st_dot.block(12,0,6,18)=_j_t_3_dot_temp*_v_pseudo;
    _j_st_dot.block(18,0,6,18)=_j_t_4_dot_temp*_v_pseudo;

    Eigen::Matrix<double,24,18> j_st_dot_0_temp;

    j_st_dot_0_temp=_j_st_dot*_t_bar.inverse() + _j*_t_inv_der; //cambiare j_st_dot
        
    Eigen::Matrix<double,12,18> j_st_dot_0_temp_red;
    j_st_dot_0_temp_red.block(0,0,3,18)=j_st_dot_0_temp.block(0,0,3,18);
    j_st_dot_0_temp_red.block(3,0,3,18)=j_st_dot_0_temp.block(6,0,3,18);
    j_st_dot_0_temp_red.block(6,0,3,18)=j_st_dot_0_temp.block(12,0,3,18);
    j_st_dot_0_temp_red.block(9,0,3,18)=j_st_dot_0_temp.block(18,0,3,18);*/
    


    if(flag==1 || flag==2){

        //_j_st_dot_0.block(_stl1,0,3,18)=j_st_dot_0_temp_red.block(_stl1,0,3,18);
        //_j_st_dot_0.block(_stl2,0,3,18)=j_st_dot_0_temp_red.block(_stl2,0,3,18);
        //_j_st_dot_0.block(_swl1,0,3,18)=j_st_dot_0_temp_red.block(_swl1,0,3,18);
        //_j_st_dot_0.block(_swl2,0,3,18)=j_st_dot_0_temp_red.block(_swl2,0,3,18);

        _j_st_dot_1.block(0,0,3,1)=_j_st_dot_0.block(_stl1,0,3,1);

        _j_st_dot_1.block(3,0,3,1)=_j_st_dot_0.block(_stl2,0,3,1);       

        _j_sw_dot.block(0,0,3,1)=_j_st_dot_0.block(_swl1,0,3,1);

        _j_sw_dot.block(3,0,3,1)=_j_st_dot_0.block(_swl2,0,3,1);
                

    }



    //////////////////////////////////////////////////////// 

    return 0;

}



void QUADRUPED::compute_m22_bar(){

    //_pb[0]=_base_pos_temp[0];
    //_pb[1]=_base_pos_temp[1];
    //_pb[2]=_base_pos_temp[2];

    _pbc = _pc - _pb;
    
    
    _s<<0.0, -_pbc[2], _pbc[1],
       _pbc[2], 0.0, -_pbc[0],
       -_pbc[1], _pbc[0], 0.0;


    /*_x<<Eigen::MatrixXd::Identity(3, 3), -_s,
       Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Identity(3, 3);*/

    _x.block(0,0,3,3)=  Eigen::MatrixXd::Identity(3, 3);
    _x.block(0,3,3,3)= _s.transpose();
    _x.block(3,0,3,3)= Eigen::MatrixXd::Zero(3, 3);
    _x.block(3,3,3,3)= Eigen::MatrixXd::Identity(3, 3);

    
    _m11=_eig_mass_matrix.block(0,0,6,6);

    _m12=_eig_mass_matrix.block(0,6,6,12);

    //Mb_Mj= _m11.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(_m12);

    _j_s=_x*(_m11.inverse()*_m12);

    //_j_s=_x*Mb_Mj;


    //Some elements for centroidal dynamics
    
    /*_t_bar<<_x, _j_s,
           Eigen::MatrixXd::Zero(12, 6), Eigen::MatrixXd::Identity(12, 12);*/

    _t_bar.block(0,0,6,6)= _x;
    _t_bar.block(0,6,6,12)= _j_s;
    _t_bar.block(6,0,12,6)=   Eigen::MatrixXd::Zero(12, 6);
    _t_bar.block(6,6,12,12)=   Eigen::MatrixXd::Identity(12, 12);   
    
    
    _m_bar=_t_bar.transpose().inverse()*_eig_mass_matrix*_t_bar.inverse();

    //incertezza

    //_m_bar(0,0)-=2.1191;
    //_m_bar(1,1)-=2.1191;
    //_m_bar(2,2)-=2.1191;

    
    _m22_bar=_m_bar.block(6,6,12,12);   

}



void QUADRUPED::compute_c2_bar(){

    /////////COMPUTE C///////////////////

    iDynTree::FreeFloatingGeneralizedTorques bias_force(* model); 
    kinDynComp.generalizedBiasForces(bias_force);
    
    
    _h<<iDynTree::toEigen(bias_force.baseWrench()),
       iDynTree::toEigen(bias_force.jointTorques());

    
    //iDynTree::FreeFloatingGeneralizedTorques gravity_force(* model); 
    //kinDynComp.generalizedGravityForces(gravity_force);    
    //_g<<iDynTree::toEigen(gravity_force.baseWrench()),
    //   iDynTree::toEigen(gravity_force.jointTorques());

    
    //_c=(_h - _g)*_v_pseudo;

    ///////////////////////////////////////////////

    //////////COMPUTE T_inv_der/////////////////

    //_pc_dot = iDynTree::toEigen(kinDynComp.getCenterOfMassVelocity());

    
    Eigen::Matrix<double,6,1> pb_dot_temp = iDynTree::toEigen(kinDynComp.getBaseTwist()); //getBaseTwist mi dà 6 componenti, ma a me ne servono solo 3 per avere consistenza dimensionale con _pc_dot


    _pb_dot[0]=pb_dot_temp(0,0);
    _pb_dot[1]=pb_dot_temp(1,0);
    _pb_dot[2]=pb_dot_temp(2,0);

    //_pb_dot[0]=_eig_robot_state.baseVel[0];
    //_pb_dot[1]=_eig_robot_state.baseVel[1];
    //_pb_dot[2]=_eig_robot_state.baseVel[2];
    

    _pbc_dot = _pc_dot - _pb_dot;

    //incertezza

    _m_dr = _robot_mass*_pbc_dot;

    _s_dot<<0.0, -_pbc_dot[2], _pbc_dot[1],
       _pbc_dot[2], 0.0, -_pbc_dot[0],
       -_pbc_dot[1], _pbc_dot[0], 0.0;

    
    /*_dx<<Eigen::MatrixXd::Zero(3, 3), -_s_dot,
       Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3);*/

    //_dx.block(0,0,3,3)= Eigen::MatrixXd::Zero(3, 3);
    _dx.block(0,3,3,3)= _s_dot.transpose(); //qui c'era -s
    //_dx.block(3,0,3,3)= Eigen::MatrixXd::Zero(3, 3);
    //_dx.block(3,3,3,3)= Eigen::MatrixXd::Zero(3, 3);

    
    _s_mdr<<0.0, -_m_dr[2], _m_dr[1],
           _m_dr[2], 0.0, -_m_dr[0],
          -_m_dr[1], _m_dr[0], 0.0;

    
    /*_dmb<<Eigen::MatrixXd::Zero(3, 3), -_s_mdr,
       _s_mdr, Eigen::MatrixXd::Zero(3, 3);*/

    //_dmb.block(0,0,3,3)= Eigen::MatrixXd::Zero(3, 3);
    //_dmb.block(0,3,3,3)= -_s_mdr;    
    _dmb.block(0,3,3,3)= _s_mdr.transpose();
    _dmb.block(3,0,3,3)= _s_mdr;
    //_dmb.block(3,3,3,3)= Eigen::MatrixXd::Zero(3, 3);   
    
    
    _dmb_1=(_m11.transpose().inverse()*_dmb.transpose()).transpose();

    //Eigen::MatrixXd inv_dMb1=(_m11.transpose().bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(_dmb.transpose())).transpose();

    _dmb_2=-_m11.inverse()*_dmb_1;

    //Eigen::MatrixXd inv_dMb2=-(_m11.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve( inv_dMb1));

    

    
    _dj_s=_dx*(_m11.inverse()*_m12)+_x*_dmb_2*_m12;

    //_dj_s=_dx*Mb_Mj+_x*inv_dMb2*_m12;
    
    //_t_inv_der.block(0,0,3,3)= Eigen::MatrixXd::Zero(3, 3);
    _t_inv_der.block(0,3,3,3)=_s_dot;
    _t_inv_der.block(0,6,3,12)= -_dj_s.block(0,0,3,12);

    //_t_inv_der.block(3,0,3,3)= Eigen::MatrixXd::Zero(3, 3);
    //_t_inv_der.block(3,3,3,3)= Eigen::MatrixXd::Zero(3, 3);
    //_t_inv_der.block(3,6,3,12)= Eigen::MatrixXd::Zero(3, 12);

    //_t_inv_der.block(6,0,12,3)= Eigen::MatrixXd::Zero(12, 3);
    //_t_inv_der.block(6,3,12,3)= Eigen::MatrixXd::Zero(12, 3);
    //_t_inv_der.block(6,6,12,12)= Eigen::MatrixXd::Zero(12, 12);

    /////////////////////////////////////////

    ///////COMPUTE C_BAR////////////////////
    //_c_bar=_t_bar.transpose().inverse()*_c*_t_bar.inverse() + _t_bar.transpose().inverse()*_eig_mass_matrix*_t_inv_der;
    ////////////////////////////////////////

    //////////COMPUTE C2_BAR//////////////
    //_c2_bar=_c_bar.block(6,0,12,18);
    /////////////////////////////////////

}



void QUADRUPED::compute_j_st_j_bar(int flag){

    ////////COMPUTE J_ST_J_BAR////////////////

    _j_st_bar_0=_j_st_0*_t_bar.inverse();

    _j_st_j_bar_0=_j_st_bar_0.block(0,6,12,12); //calcola questo termine sempre, mi serve per lo stimatore

    if(flag==0){
        

        

        _j_st_c_bar_0=_j_st_bar_0.block(0,0,12,6);
    }

    if(flag==1 || flag==2){
        //_j_st_bar_1=_j_st_1*_t_bar.inverse();

        _j_st_bar_1.block(0,0,3,18)=_j_st_bar_0.block(_stl1,0,3,18);

        _j_st_bar_1.block(3,0,3,18)=_j_st_bar_0.block(_stl2,0,3,18);

        _j_st_j_bar_1=_j_st_bar_1.block(0,6,6,12);

        _j_st_c_bar_1=_j_st_bar_1.block(0,0,6,6);

        //_j_sw_bar=_j_sw*_t_bar.inverse();

        _j_sw_bar.block(0,0,3,18)=_j_st_bar_0.block(_swl1,0,3,18);

        _j_sw_bar.block(3,0,3,18)=_j_st_bar_0.block(_swl2,0,3,18);


        _j_sw_j_bar=_j_sw_bar.block(0,6,6,12);

        _j_sw_c_bar=_j_sw_bar.block(0,0,6,6);



        _j_st_bar_1_new=_j_st_1_new*_t_bar.inverse();

        _j_st_j_bar_1_new=_j_st_bar_1_new.block(0,6,12,12);
    }

    //////////////////////////////////////////

    _j_bar=_j*_t_bar.inverse();

    //_j_bar_red.block(0,0,3,18)=_j_bar.block(0,0,3,18);
    //_j_bar_red.block(3,0,3,18)=_j_bar.block(6,0,3,18);
    //_j_bar_red.block(6,0,3,18)=_j_bar.block(12,0,3,18);
    //_j_bar_red.block(9,0,3,18)=_j_bar.block(18,0,3,18);
}



int QUADRUPED::init_model(){


    model = &kinDynComp.model();

    kinDynComp.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

    std::string path = ros::package::getPath("dogbot_description"); 

    path += "/urdf/dogbot.urdf";
    
    //bool ok = mdlLoader.loadModelFromFile(modelFile);

    

    bool ok = mdlLoader.loadModelFromFile(path);


    if( !ok )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
        return EXIT_FAILURE;
    }



    ok = kinDynComp.loadRobotModel(mdlLoader.model());


    if( !ok )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return EXIT_FAILURE;
    }

    //std::cerr<< mdlLoader.model().toString() << std::endl;

    _robot_mass=model->getTotalMass();


    //If you are interested in a frame with a given name, you can obtain its associated index with
    //the appropriate method (TRAVERSARO)
    //arbitraryFrameName = model->getFrameName(model->getNrOfFrames()/2);

    //arbitraryFrameIndex = model->getFrameIndex(arbitraryFrameName);

    /////////////////////////////////////////////////////////
    //DEFINITION OF FRAMES
    /*_base_link_frame = model->getFrameName(0);
    _body_frame = model->getFrameName(1);
    _bodytext_frame = model->getFrameName(2);
    _back_left_hip_frame = model->getFrameName(3);
    _back_left_upperleg_frame = model->getFrameName(4);
    _back_left_lowerleg_frame = model->getFrameName(5);
    _back_left_foot_frame = model->getFrameName(6);
    _back_right_hip_frame = model->getFrameName(7);
    _back_right_upperleg_frame = model->getFrameName(8);
    _back_right_lowerleg_frame = model->getFrameName(9);
    _back_right_foot_frame = model->getFrameName(10);
    _front_left_hip_frame = model->getFrameName(11);
    _front_left_upperleg_frame = model->getFrameName(12);
    _front_left_lowerleg_frame = model->getFrameName(13);
    _front_left_foot_frame = model->getFrameName(14);
    _front_right_hip_frame = model->getFrameName(15);
    _front_right_upperleg_frame = model->getFrameName(16);
    _front_right_lowerleg_frame = model->getFrameName(17);
    _front_right_foot_frame = model->getFrameName(18);*/

    //////////////////////////////////////////////////////////////////


    //More complex quantities (such as jacobians and matrices) need to be handled in a different way for efficency reasons
    //Compute the mass matrix M (TRAVERSARO)
    
    _eig_mass_matrix=Eigen::MatrixXd::Identity(6+12, 6+12);
    
    //Compute a general jacobian (TRAVERSARO)
    
    _eig_jacobian=Eigen::MatrixXd::Zero(6, 6+12);
  
    
    //Compute the center of mass jacobian (TRAVERSARO)
    
    _eig_com_jacobian=Eigen::MatrixXd::Zero(3, 6+12);
   
    //_eig_robot_state.resize(12);
    _eig_robot_state.random();
    //For inverse dynamics, we need to pass the acceleration, of the robot
    //as it is not part of the "state" (TRAVERSARO)
    //_eig_robot_acceleration.resize(12);
    //_eig_robot_acceleration.random();

    


    //Stacking v
    //_eig_robot_state.v.resize(_eig_robot_state.baseVel.size() + _eig_robot_state.jointVel.size(),1);
    

    

    //Stacking a
    /*_eig_robot_acceleration.a.resize(_eig_robot_acceleration.baseAcc.size() + _eig_robot_acceleration.jointAcc.size());
    _eig_robot_acceleration.a << _eig_robot_acceleration.baseAcc, 
                                 _eig_robot_acceleration.jointAcc;*/

    //Selection matrix
    
    //_select_matrix = Eigen::MatrixXd::Zero(12,6+12);
    //_select_matrix.block(0,0,12,6)=Eigen::MatrixXd::Zero(12,6);
    //_select_matrix.block(0,6,12,12)=Eigen::MatrixXd::Identity(12, 12);
    
    //Pseudo-inverse for the velocity vector v (needed to compute the Coriolis matrix)
    //_v_pseudo.resize(1, _eig_robot_state.baseVel.size() + _eig_robot_state.jointVel.size());
    //_v_pseudo = Eigen::MatrixXd::Zero(1, _eig_robot_state.baseVel.size() + _eig_robot_state.jointVel.size());
    //_v_pseudo = _eig_robot_state.v.completeOrthogonalDecomposition().pseudoInverse();

    

    ////////////////////////////////////////////////////
    //FLOATING BASE DYNAMICS

    //_n_st=4;

    //_j_st.resize(3*_n_st, 6+12);
    //_j_st = Eigen::MatrixXd::Zero(3*_n_st, 6+12);

    //_j_st_b.resize(3*_n_st,6);   
    //_j_st_b = Eigen::MatrixXd::Zero(3*_n_st, 6);

    //_j_st_j.resize(3*_n_st,12);
    //_j_st_j = Eigen::MatrixXd::Zero(3*_n_st, 12);

    //_j_st_0 = Eigen::MatrixXd::Zero(12, 18);
    //_j_st_b_0 = Eigen::MatrixXd::Zero(12, 6);
    //_j_st_j_0 = Eigen::MatrixXd::Zero(12, 12);

    //_j_st_1 = Eigen::MatrixXd::Zero(6, 18);
    //_j_st_b_1 = Eigen::MatrixXd::Zero(6, 6);
    //_j_st_j_1 = Eigen::MatrixXd::Zero(6, 12);

    //_j_st_1_new=Eigen::MatrixXd::Zero(12,18);
    //_j_st_j_1_new=Eigen::MatrixXd::Zero(12,12);

    //_j_st_bar_1_new=Eigen::MatrixXd::Zero(12,18);
    //_j_st_j_bar_1_new=Eigen::MatrixXd::Zero(12,12);



    //////////////////CONTACT POINT JACOBIAN (ONLY FEET)///////////////////////

    //BACK LEFT
    
    //_j_t_1 = Eigen::MatrixXd::Zero(6, 6+12);
    
    //BACK RIGHT
    
    //_j_t_2 = Eigen::MatrixXd::Zero(6, 6+12);;

    //FRONT LEFT
    
    //_j_t_3 = Eigen::MatrixXd::Zero(6, 6+12);
   
    //FRONT RIGHT
    
    //_j_t_4 = Eigen::MatrixXd::Zero(6, 6+12);
        

    ///////////////////////////////////////////////////////
    //DEFINITIONS FOR M22_BAR
    
    /*for(int i=0; i<3; i++){
        _pc[i]=0.0;
    }
    //_pc = iDynTree::toEigen(kinDynComp.getCenterOfMassPosition());



    for(int i=0; i<3; i++){
        _pb[i]=0.0;
    }
    //_pb = iDynTree::toEigen(pb_temp_2);

    for(int i=0; i<3; i++){
        _pbc[i]=0.0;
    }
    _pbc = _pc - _pb;*/

    


    
    //_s= Eigen::MatrixXd::Zero(3,3);

    
    //_x = Eigen::MatrixXd::Zero(6, 6);

    
    //_m11 = Eigen::MatrixXd::Zero(6, 6);

  
    //_m12 = Eigen::MatrixXd::Zero(6, 12);
 
   
    //_j_s = Eigen::MatrixXd::Zero(6, 12);

    
    //_t_bar = Eigen::MatrixXd::Zero(6+12, 6+12);

    
    //_m_bar = Eigen::MatrixXd::Zero(6+12, 6+12);
    
    
    //_m22_bar = Eigen::MatrixXd::Zero(12, 12);

    ///////////////////////////////////////////////////////

    //////////////////////////////////////////////////////
    //DEFINITIONS FOR C2_BAR

    
    /*for(int i=0; i<6+12; i++){
        _h(i,0)=0.0;
    }


    
    for(int i=0; i<6+12; i++){
        _g(i,0)=0.0;
    }*/


    
    //_c = Eigen::MatrixXd::Zero(6+12, 6+12);

    /*for(int i=0; i<3; i++){
        _pc_dot[i]=0.0;
    }


    for(int i=0; i<3; i++){
        _pb_dot[i]=0.0;
    }

    for(int i=0; i<3; i++){
        _pbc_dot[i]=0.0;
    }

    for(int i=0; i<3; i++){
        _m_dr[i]=0.0;
    }*/

    
    //_s_dot = Eigen::MatrixXd::Zero(3, 3);

    
    _dx = Eigen::MatrixXd::Zero(6, 6);

    
    //_s_mdr = Eigen::MatrixXd::Zero(3, 3);

   
    _dmb = Eigen::MatrixXd::Zero(6, 6);


    
    //_dmb_1 = Eigen::MatrixXd::Zero(6, 6);

    
    //_dmb_2 = Eigen::MatrixXd::Zero(6, 6);

    
    //_dj_s = Eigen::MatrixXd::Zero(6, 12);

    
    _t_inv_der = Eigen::MatrixXd::Zero(18, 18);

    
    //_c_bar = Eigen::MatrixXd::Zero(6+12, 6+12);

    
    //_c2_bar = Eigen::MatrixXd::Zero(12, 6+12);

    /////////////////////////////////////////////////////

    /////////////////////////////////////////////////////
    //DEFINITIONS FOR _j_st_J_BAR

    //_j_st_bar.resize(3*_n_st, 6+12);
    //_j_st_bar = Eigen::MatrixXd::Zero(3*_n_st, 6+12);

    //_j_st_j_bar.resize(3*_n_st, 12);
    //_j_st_j_bar = Eigen::MatrixXd::Zero(3*_n_st, 12);

    //_j_st_c_bar.resize(3*_n_st, 6);
    //_j_st_c_bar = Eigen::MatrixXd::Zero(3*_n_st, 6);

    _j_st_bar_0 = Eigen::MatrixXd::Zero(12, 18);
    _j_st_c_bar_0 = Eigen::MatrixXd::Zero(12, 6);
    _j_st_j_bar_0 = Eigen::MatrixXd::Zero(12, 12);

    _j_st_bar_1 = Eigen::MatrixXd::Zero(6, 18);
    _j_st_c_bar_1 = Eigen::MatrixXd::Zero(6, 6);
    _j_st_j_bar_1 = Eigen::MatrixXd::Zero(6, 12);



    /////////////////////////////////////////////////////

    /////////////////////////////////////////////////////
    //DEFINITIONS FOR F_GR_STAR
    //_f_gr_star.resize(3*_n_st,1);

    /*for(int i=0; i<12; i++){
        _f_gr_star(i,0)=0.0;
    }*/

    
    

    ////////////////////////////////////////////////////

    /////////////////////////////////////////////////////
    //DEFINITIONS F0R Q_DOT_DOT_DES

    
    /*for(int i=0; i<12; i++){
        _q_dot_dot_des(i,0)=0.0;
    }*/
    

    ///////////////////////////////////////////////////

    //////////////////////////////////////////////////
    //DEFINITIONS FOR JNT_TORQUE_STAR

    
    /*for(int i=0; i<12; i++){
        _jnt_torques_star(i,0)=0.0;
    }*/


    //////////////////////////////////////////////////

    //////////////////////////////////////////////////
    //DEFINITIONS FOR Q_DOT_DOT

    
    /*for(int i=0; i<18; i++){
        _q_dot_dot(i,0)=0.0;
    }*/    


    ///////////////////////////////////////////////
    //DEFINITIONS FOR Q

    /*for(int i=0; i<18; i++){
        _q_dot(i,0)=0.0;
    }*/

    
    
    ////////////////////////////////////////7/////

    /////////////////////////////////////////////
    //DEFINITIONS FOR Q

    /*for(int i=0; i<18; i++){
        _q(i,0)=0.0;
    }*/

    //////////////////////////////////////////////

    /////////////////////////////////////////////
    //DEFINITIONS FOR F_GR
    //_f_gr.resize(3*_n_st,1);
    /*for(int i=0; i<12; i++){
        _f_gr(i,0)=0.0;
    }*/

    /////////////////////////////////////////////

    //_v_c=Eigen::MatrixXd::Zero(18,1);

    ////////////////////////////////////////////
    //DEFINITIONS FOR W_COM_DES

    

    //_k_p=Eigen::MatrixXd::Identity(6,6);

    //_k_d=Eigen::MatrixXd::Identity(6,6);

    //_r_c=Eigen::MatrixXd::Zero(6,1);

    //_r_c_ref=Eigen::MatrixXd::Zero(6,1);

    //_r_c_dot=Eigen::MatrixXd::Zero(6,1);

    //_r_c_ref_dot=Eigen::MatrixXd::Zero(6,1);

    //_r_c_dot_dot=Eigen::MatrixXd::Zero(6,1);

    //_r_c_ref_dot_dot=Eigen::MatrixXd::Zero(6,1);

    //_w_com_des=Eigen::MatrixXd::Zero(6,1);

    


    

    ////////////////////////////////////////////

    ////////////////////////////////////////////
    //DEFINITIONS FOR QUADRATIC PROBLEM

    //_sigma.resize(3*_n_st, 6+12+3*_n_st);
    //_sigma.block(0,0,3*_n_st,6+12)=Eigen::MatrixXd::Zero(3*_n_st,6+12);
    //_sigma.block(0,6+12,3*_n_st,3*_n_st)=Eigen::MatrixXd::Identity(3*_n_st,3*_n_st);

    //_f_st_hat.resize(3*_n_st,1);
    /*_f_st_hat_0=Eigen::MatrixXd::Zero(12,1);
    _f_st_hat_1=Eigen::MatrixXd::Zero(6,1);
    _f_sw_hat=Eigen::MatrixXd::Zero(6,1);*/

    /*_A.resize(6+12+3*_n_st,6+12+3*_n_st);
    _A.block(0,0,6+12+3*_n_st,6+12+3*_n_st)=Eigen::MatrixXd::Zero(6+12+3*_n_st,6+12+3*_n_st);

    _b1.resize(1,6+12+3*_n_st);
    _b1.block(0,0,1,6+12+3*_n_st)=Eigen::MatrixXd::Zero(1,6+12+3*_n_st);

    _b2.resize(6+12+3*_n_st,1);
    _b2.block(0,0,6+12+3*_n_st,1);

    _A_con.resize(6+3*_n_st, 6+12+3*_n_st+3*_n_sw);
    _A_con.block(0,0,6+3*_n_st,6+12+3*_n_st+3*_n_sw)=Eigen::MatrixXd::Zero(6+3*_n_st,6+12+3*_n_st+3*_n_sw);

    _b_con.resize(6+3*_n_st,1);
    _b_con=Eigen::MatrixXd::Zero(6+3*_n_st,1);

    _D_con.resize(4*_n_st + 12 + 3*_n_sw + 3*_n_sw, 6 + 12 + 3*_n_st + 3*_n_sw);
    _D_con=Eigen::MatrixXd::Zero(4*_n_st + 12 + 3*_n_sw + 3*_n_sw, 6 + 12 + 6 + 3*_n_sw);
    
    _c_con.resize(4*_n_st + 12 + 12 + _n_sw + _n_sw, 1);
    _c_con=Eigen::MatrixXd::Zero(4*_n_st + 12 + 12 + _n_sw + _n_sw,1);*/
    
    //_L_bar_1.resize(3,_n_st);
    //_L_bar_1=Eigen::MatrixXd::Zero(3,_n_st);

    //_L_bar_2.resize(3,_n_st);
    //_L_bar_2=Eigen::MatrixXd::Zero(3,_n_st);

    //_n_bar.resize(3,_n_st);
    //_n_bar=Eigen::MatrixXd::Zero(3,_n_st);

    //_d_fr.resize(4*_n_st, 3*_n_st);
    //_d_fr=Eigen::MatrixXd::Zero(4*_n_st, 3*_n_st);

    

    /*for(int i=0; i<4; i++){
        _contact[i]=true;
    }*/

    

    //_j_sw.resize(3*_n_st, 6+12);
    //_j_sw = Eigen::MatrixXd::Zero(6, 6+12);

    //_j_sw_b.resize(3*_n_st,6);   
    //_j_sw_b = Eigen::MatrixXd::Zero(6, 6);

    //_j_sw_j.resize(3*_n_st,12);
    //_j_sw_j = Eigen::MatrixXd::Zero(6, 12);

    //_j_sw_bar.resize(3*_n_st, 6+12);
    //_j_sw_bar = Eigen::MatrixXd::Zero(6, 6+12);

    //_j_sw_j_bar.resize(3*_n_st, 12);
    //_j_sw_j_bar = Eigen::MatrixXd::Zero(6, 12);

    //_j_sw_c_bar.resize(3*_n_st, 6);
    _j_sw_c_bar = Eigen::MatrixXd::Zero(6, 6);

    //_j_st_dot.resize(3*_n_st,1);
    _j_st_dot_0 = Eigen::MatrixXd::Zero(12,18);
    _j_st_dot_1 = Eigen::MatrixXd::Zero(6,1);

    _j_st_dot=Eigen::MatrixXd::Zero(24,18);

    _j_sw_dot= Eigen::MatrixXd::Zero(6,1);

    _j_st_temp=Eigen::MatrixXd::Zero(12,18);

    _j_t_1_dot_temp=Eigen::MatrixXd::Zero(6,1);

    _j_t_2_dot_temp=Eigen::MatrixXd::Zero(6,1);

    _j_t_3_dot_temp=Eigen::MatrixXd::Zero(6,1);

    _j_t_4_dot_temp=Eigen::MatrixXd::Zero(6,1);

    _j_st_dot_temp = Eigen::MatrixXd::Zero(24,1);

    //_gamma_1=Eigen::MatrixXd::Zero(12,1);

    //_gamma_2=Eigen::MatrixXd::Zero(12,1);

    //_gamma_3=Eigen::MatrixXd::Zero(12,1);

    //_j=Eigen::MatrixXd::Zero(24,18);

    //_j_bar=Eigen::MatrixXd::Zero(24,18);

    //_Tbl=Eigen::MatrixXd::Zero(3,3);

    //_Tbr=Eigen::MatrixXd::Zero(3,3);

    //_Tfl=Eigen::MatrixXd::Zero(3,3);

    //_Tfr=Eigen::MatrixXd::Zero(3,3);

    //_x_sw_des_dot_dot=Eigen::MatrixXd::Zero(6,1);
    //_x_sw_des_dot=Eigen::MatrixXd::Zero(6,1);
    //_x_sw_dot=Eigen::MatrixXd::Zero(6,1);
    //_x_sw_des=Eigen::MatrixXd::Zero(6,1);
    //_x_sw=Eigen::MatrixXd::Zero(6,1);

    //_accd=Eigen::MatrixXd::Zero(6,1);
    //_posdelta=Eigen::MatrixXd::Zero(6,1);
    //_veldelta=Eigen::MatrixXd::Zero(6,1);

     

    //_x_sw_cmd_dot_dot.resize(3*_n_sw,1);
    //_x_sw_cmd_dot_dot=Eigen::MatrixXd::Zero(6,1);

   
   
    //Time derivative of Jacobian
    
    //_roll=0.0;
    //_pitch=0.0;
    //_yaw=0.0;

    //_init_pos= Eigen::MatrixXd::Zero(6,1);
    //_init_vel= Eigen::MatrixXd::Zero(6,1);
    //_fin_pos = Eigen::MatrixXd::Zero(6,1);

    _Jdqd=Eigen::MatrixXd::Zero(24,1);

    _JdqdCOM=Eigen::MatrixXd::Zero(24,1);

    _JdqdCOM_lin=Eigen::MatrixXd::Zero(12,1);

    _B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
         Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	     Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	     Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);

    _bias_com=Eigen::MatrixXd::Zero(18,1);
    


    return 0;
}


//void QUADRUPED::compute_q(){
    

    //ros::Rate rate(1000);

    /*double h=1.0/100.0; //if the rate changes, it is sufficient to change the denominator

    Eigen::Matrix<double,12,1> q_dot_dot_old;

    q_dot_dot_old=Eigen::MatrixXd::Zero(12,1);

    Eigen::Matrix<double,12,1> q_dot_old;

    q_dot_old=Eigen::MatrixXd::Zero(12,1);

    Eigen::Matrix<double,12,1> q_old;

    q_old=Eigen::MatrixXd::Zero(12,1);

    for(int i=0; i<12; i++){
        q_dot_old(i,0)=_eig_robot_state.jointVel[i];

        q_old(i,0)=_eig_robot_state.jointPos[i];
    }


	

    /////////////////////////////////////
    //FORWARD DYNAMICS
    //In the computation, one can neglet the external forces term
    _q_dot_dot=_eig_mass_matrix.inverse()*(-_h + _select_matrix.transpose()*_jnt_torques_star + _j_st_0.transpose()*_f_gr);
    //_q_dot_dot=_m22_bar.inverse()*(-_c2_bar*_v_c + _jnt_torques_star + _j_st_j_bar.transpose()*_f_gr);

    //_q_dot_dot.block(6,0,12,1)=_q_dot_dot_des;
    
    //Now let's compute the integration with forward Euler method
    _q_dot.block(6,0,12,1)=q_dot_old + h*_q_dot_dot.block(6,0,12,1);
    
    _q.block(6,0,12,1)=q_old + h*_q_dot.block(6,0,12,1);

    _already_compute=true;

    /*std::cout<<"desired ground reaction forces: "<<endl;

    for(int i=0; i<12; i++){
        std::cout<<_f_gr_star(i,0)<<endl;
    }*/
    

    //Updating old variables
	//q_dot_dot_old=_q_dot_dot.block(6,0,12,1);

    //q_dot_old=_q_dot.block(6,0,12,1);

    //q_old=_q.block(6,0,12,1);*/

//}



//get functions for optimal

//void QUADRUPED::set_pc(Eigen::Vector3d data){
//    _pc=data;
//}

//void QUADRUPED::set_pc_dot(Eigen::Vector3d data){
//    _pc_dot=data;
//}

//void QUADRUPED::set_r_c(Eigen::Matrix<double,6,1> data){
//    _r_c=data;
//}

//void QUADRUPED::set_r_c_dot(Eigen::Matrix<double,6,1> data){
//    _r_c_dot=data;
//}

void QUADRUPED::set_w_com_des(Eigen::Matrix<double,6,1> data){
    _w_com_des=data;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_r_c(){
    return _r_c;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_r_c_dot(){
    return _r_c_dot;
}

Eigen::Vector3d QUADRUPED::get_pc(){
    return _pc;
}

Eigen::Vector3d QUADRUPED::get_pc_dot(){
    return _pc_dot;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_r_c_ref(){
    return _r_c_ref;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_r_c_ref_dot(){
    return _r_c_ref_dot;
}

Eigen::Matrix<double,18,18> QUADRUPED::get_mass_matrix(){
    return _m_bar;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_r_c_ref_dot_dot(){
    return _r_c_ref_dot_dot;
}

double QUADRUPED::get_mu(){
    return _mu;
}

//void QUADRUPED::set_f_st_hat_0(Eigen::Matrix<double,12,1> data){
//    _f_st_hat_0=data;
//}

//void QUADRUPED::set_f_st_hat_1(Eigen::Matrix<double,6,1> data){
//    _f_st_hat_1=data;
//}

//void QUADRUPED::set_f_sw_hat(Eigen::Matrix<double,6,1> data){
//    _f_sw_hat=data;
//}

Eigen::Matrix<double,12,6> QUADRUPED::get_j_st_c_bar_0(){
    return _j_st_c_bar_0;
}

Eigen::Matrix<double,6,6> QUADRUPED::get_j_st_c_bar_1(){
    return _j_st_c_bar_1;
}

Eigen::Matrix<double,12,18> QUADRUPED::get_j_st_bar_0(){
    return _j_st_bar_0;
}

Eigen::Matrix<double,6,18> QUADRUPED::get_j_st_bar_1(){
    return _j_st_bar_1;
}

Eigen::Matrix<double,18,18> QUADRUPED::get_t_bar(){
    return _t_bar;

}

Eigen::Matrix<double,18,1> QUADRUPED::get_h(){
    return _h;

}

Eigen::Matrix<double,18,18> QUADRUPED::get_t_inv_der(){
    return _t_inv_der;
}

Eigen::Matrix<double,18,1> QUADRUPED::get_v_c(){
    return _v_c;
}

Eigen::Matrix<double,24,18> QUADRUPED::get_j_bar(){
    return _j_bar;
}

Eigen::Matrix<double,12,18> QUADRUPED::get_j_st_dot_0(){
    return _j_st_dot_0;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_j_st_dot_1(){
    return _j_st_dot_1;
}

Eigen::Matrix<double,12,12> QUADRUPED::get_j_st_j_bar_0(){
    return _j_st_j_bar_0;
}

Eigen::Matrix<double,6,12> QUADRUPED::get_j_st_j_bar_1(){
    return _j_st_j_bar_1;
}


Eigen::Matrix<double,12,18> QUADRUPED::get_c2_bar(){
    return _c2_bar;
}

void QUADRUPED::set_q_dot_dot_des(Eigen::Matrix<double,12,1> data){
    _q_dot_dot_des=data;
}

void QUADRUPED::set_f_gr_star(Eigen::Matrix<double,12,1> data){
    _f_gr_star=data;
}

Eigen::Matrix<double,6,12> QUADRUPED::get_j_sw_j_bar(){
    return _j_sw_j_bar;
}

Eigen::Matrix<double,6,6> QUADRUPED::get_j_sw_c_bar(){
    return _j_sw_c_bar;
}

/*Eigen::Matrix<double,6,1> QUADRUPED::get_x_sw_cmd_dot_dot(){
    return _x_sw_cmd_dot_dot;
}*/

Eigen::Matrix<double,6,1> QUADRUPED::get_j_sw_dot(){
    return _j_sw_dot;
}

/*Eigen::Matrix<double,12,1> QUADRUPED::get_gamma_1(){
    return _gamma_1;
}

Eigen::Matrix<double,12,1> QUADRUPED::get_gamma_2(){
    return _gamma_2;
}

Eigen::Matrix<double,12,1> QUADRUPED::get_gamma_3(){
    return _gamma_3;
}

void QUADRUPED::set_gamma_1(Eigen::Matrix<double,12,1> data){
    _gamma_1=data;
}

void QUADRUPED::set_gamma_2(Eigen::Matrix<double,12,1> data){
    _gamma_2=data;
}

void QUADRUPED::set_gamma_3(Eigen::Matrix<double,12,1> data){
    _gamma_3=data;
}*/

Eigen::Matrix<double,12,1> QUADRUPED::get_jnt_torques_star(){
    return _jnt_torques_star;
}

Eigen::Matrix<double,12,12> QUADRUPED::get_j_st_j_bar_1_new(){
    return _j_st_j_bar_1_new;
}

//Eigen::Matrix<double,12,1> QUADRUPED::get_f_gr(){
//    return _f_gr;
//}

Eigen::Matrix<double,6,1> QUADRUPED::get_w_com_des(){
    return _w_com_des;
}

//Eigen::Matrix<double,12,1> QUADRUPED::get_f_st_hat_0(){
//    return _f_st_hat_0;
//}

//Eigen::Matrix<double,6,1> QUADRUPED::get_f_st_hat_1(){
//    return _f_st_hat_1;
//}


//Eigen::Matrix<double,6,1> QUADRUPED::get_f_sw_hat(){
//    return _f_sw_hat;
//}


Eigen::Quaterniond QUADRUPED::get_quaternion(){
    return q;
}

//Eigen::Matrix<double,3,3> QUADRUPED::get_Tbl(){
//    return _Tbl;
//}

//Eigen::Matrix<double,3,3> QUADRUPED::get_Tbr(){
//    return _Tbr;
//}

//Eigen::Matrix<double,3,3> QUADRUPED::get_Tfl(){
//    return _Tfl;
//}

//Eigen::Matrix<double,3,3> QUADRUPED::get_Tfr(){
//    return _Tfr;
//}

//

//get functions for planning

Eigen::Matrix<double,6,1> QUADRUPED::get_init_pos(){
    return _init_pos;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_init_vel(){
    return _init_vel;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_fin_pos(){
    return _fin_pos;
}



//void QUADRUPED::set_r_c_ref(Eigen::Matrix<double,6,1> data){
//    _r_c_ref=data;
//}

//void QUADRUPED::set_r_c_ref_dot(Eigen::Matrix<double,6,1> data){
//    _r_c_ref_dot=data;
//}

//void QUADRUPED::set_r_c_ref_dot_dot(Eigen::Matrix<double,6,1> data){
//    _r_c_ref_dot_dot=data;
//}

Eigen::MatrixXd QUADRUPED::getBRpos(){

	iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform(11);
	return toEigen(World_br.getPosition());
}

Eigen::MatrixXd QUADRUPED::getBLpos(){

	iDynTree::Transform  World_bl;
    World_bl=kinDynComp.getWorldTransform(8);
	return toEigen(World_bl.getPosition());

}

Eigen::MatrixXd QUADRUPED::getFLpos(){

	iDynTree::Transform  World_fl;
    World_fl=kinDynComp.getWorldTransform(14);
	return toEigen(World_fl.getPosition());

}

Eigen::MatrixXd QUADRUPED::getFRpos(){

	iDynTree::Transform  World_fr;
    World_fr=kinDynComp.getWorldTransform(17);
	return toEigen(World_fr.getPosition());

}

Eigen::MatrixXd QUADRUPED::getBRvel(){
    
    iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel(11);
	return toEigen(br_vel.getLinearVec3() );

}

Eigen::MatrixXd QUADRUPED::getBLvel(){ 

	iDynTree::Twist bl_vel;
	bl_vel=kinDynComp.getFrameVel(8);
	return toEigen(bl_vel.getLinearVec3() );

}

Eigen::MatrixXd QUADRUPED::getFLvel(){

    iDynTree::Twist fl_vel;
	fl_vel=kinDynComp.getFrameVel(14);
	return toEigen(fl_vel.getLinearVec3() );

}

Eigen::MatrixXd QUADRUPED::getFRvel(){

	iDynTree::Twist fr_vel;
	fr_vel=kinDynComp.getFrameVel(17);
	return toEigen(fr_vel.getLinearVec3() );

}

Eigen::Matrix4d QUADRUPED::get_world_base(){
    return _eig_robot_state.world_H_base;
}

Eigen::Matrix<double,6,18> QUADRUPED::get_j_st_1(){
    return _j_st_1;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_accd(){
    return _accd;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_veldelta(){
    return _veldelta;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_posdelta(){
    return _posdelta;
}

Eigen::Matrix<double,6,18> QUADRUPED::get_j_sw(){
    return _j_sw;
}

//Eigen::Matrix<double,18,18> QUADRUPED::get_c(){
//    return _c;
//}

int QUADRUPED::get_stl1(){
    return _stl1;
}

int QUADRUPED::get_stl2(){
    return _stl2;
}

int QUADRUPED::get_swl1(){
    return _swl1;
}

int QUADRUPED::get_swl2(){
    return _swl2;
}

Eigen::Matrix<double,18,1>  QUADRUPED::get_bias_com(){
    return _bias_com;
}

//bool QUADRUPED::get_overlap(){
//    return _overlap;
//}

//void QUADRUPED::set_overlap(bool data){
//    _overlap=data;
//
//}

//void QUADRUPED::set_diff(double data){
//    _diff=data;
//}

//double QUADRUPED::get_diff(){
//    return _diff;
//}

Eigen::Matrix<double,12,1> QUADRUPED::get_joint_pos(){
    return _eig_robot_state.jointPos;
}

double QUADRUPED::get_roll(){
    return _roll;
}

double QUADRUPED::get_pitch(){
    return _pitch;
}

double QUADRUPED::get_yaw(){
    return _yaw;
}

ros::NodeHandle QUADRUPED::get_nh(){
    return _nh;
}

/*Eigen::Matrix<double,12,1> QUADRUPED::get_f_ext(){
    return _f_ext;
}*/

double QUADRUPED::get_m(){
    return _robot_mass;
}

bool QUADRUPED::get_started(){
    return _start;
}



//ultimo


void QUADRUPED::loop(){

    //ros::Rate r(1000);

    gazebo_msgs::SetModelConfiguration robot_init_config;
    std_srvs::Empty pauseSrv;
    std_srvs::Empty unpauseSrv;

    ros::ServiceClient set_model_configuration_srv = _nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient set_model_state_srv = _nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient resetGazeboSimulation = _nh.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    ros::ServiceClient pauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    ros::ServiceClient unpauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");



    _pub = _node->Advertise<gazebo::msgs::WorldControl>("~/world_control");

    _pub->WaitForConnection();

    _stepper.set_step(1); 
    

    //label: 

    label:while(ros::ok()){

        if(!_already_config){

            
            ////INITIAL CONFIGURATION/////////////////////////           

            
            robot_init_config.request.model_name = "dogbot";
            robot_init_config.request.urdf_param_name = "robot_description";
            robot_init_config.request.joint_names.push_back("back_left_roll_joint");
            robot_init_config.request.joint_names.push_back("back_left_pitch_joint");
            robot_init_config.request.joint_names.push_back("back_left_knee_joint");
            robot_init_config.request.joint_names.push_back("back_right_roll_joint");
            robot_init_config.request.joint_names.push_back("back_right_pitch_joint");
            robot_init_config.request.joint_names.push_back("back_right_knee_joint");
            robot_init_config.request.joint_names.push_back("front_left_roll_joint");
            robot_init_config.request.joint_names.push_back("front_left_pitch_joint");
            robot_init_config.request.joint_names.push_back("front_left_knee_joint");
            robot_init_config.request.joint_names.push_back("front_right_roll_joint");
            robot_init_config.request.joint_names.push_back("front_right_pitch_joint");
            robot_init_config.request.joint_names.push_back("front_right_knee_joint");
            robot_init_config.request.joint_positions.push_back( 0.0004875394147498824);
            robot_init_config.request.joint_positions.push_back( -0.884249947977489);
            robot_init_config.request.joint_positions.push_back(-1.6039026405138666);
            robot_init_config.request.joint_positions.push_back( 0.0006243098169198547);
            robot_init_config.request.joint_positions.push_back(0.8861978063639038);
            robot_init_config.request.joint_positions.push_back(1.6032646991719783);
            robot_init_config.request.joint_positions.push_back(-3.197670677312914e-05);
            robot_init_config.request.joint_positions.push_back(-0.8848124990461947);
            robot_init_config.request.joint_positions.push_back(-1.6039627256817717);
            robot_init_config.request.joint_positions.push_back(-0.0005127385581351618);
            robot_init_config.request.joint_positions.push_back(0.886353788084274);
            robot_init_config.request.joint_positions.push_back( 1.60361055049274);

            if(set_model_configuration_srv.call(robot_init_config))
              ROS_INFO("Robot configuration set.");
            else
              ROS_INFO("Failed to set robot configuration.");
        
    
            gazebo_msgs::SetModelState robot_init_state;
            robot_init_state.request.model_state.model_name = "dogbot";
            robot_init_state.request.model_state.reference_frame = "world";
            //robot_init_state.request.model_state.pose.position.x=-0.00;
            robot_init_state.request.model_state.pose.position.x=1.0;
            //robot_init_state.request.model_state.pose.position.y=-0.034102251365;
            robot_init_state.request.model_state.pose.position.y=3.0;
            robot_init_state.request.model_state.pose.position.z=0.430159040502;
            robot_init_state.request.model_state.pose.orientation.x=0.0;
            robot_init_state.request.model_state.pose.orientation.y=0.0;
            robot_init_state.request.model_state.pose.orientation.z=0.0;
            robot_init_state.request.model_state.pose.orientation.w=1;

            if(set_model_state_srv.call(robot_init_state))
              ROS_INFO("Robot state set.");
            else
              ROS_INFO("Failed to set robot state.");
    


            
        
            /*while(!_joint_state_available || !_base_state_available ){

                ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
                ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
                ros::spinOnce();
                //std::cout<<"dentro il while"<<endl;
            }*/

            //ros::spinOnce();


            while( !_joint_state_available  )
                usleep(0.1*1e6);

            while( !_base_state_available  )
                usleep(0.1*1e6);

            _already_config=true;

            /////////////////////////////////////////////////////////////////////////
    }         
     
    if(plan->get_start_scan()){

        update(0);

        _init_pos=_r_c;

        _init_vel=_v_c.block(0,0,6,1);

        pauseGazebo.call(pauseSrv);
   
        if(pauseGazebo.call(pauseSrv))
           ROS_INFO("Simulation paused.");
        else
           ROS_INFO("Failed to pause simulation.");

        plan->compute_planning(0);

        unpauseGazebo.call(unpauseSrv);

        _duration=plan->get_formulation().params_.ee_phase_durations_.at(0)[0];

        _begin = ros::Time::now();

        stand_phase();

        goto label;

    }

    ///////////////////////////////////////////////
    //ros::spinOnce();

    update(0);

    ///////SET INITIAL POSITION AND FINAL POSITION///////////////

    _init_pos=_r_c;

    _init_vel=_v_c.block(0,0,6,1);

    //_fin_pos=quad->get_r_c();

    //_fin_pos(1,0)=quad->_fin_pos(1,0)  - 0.1;

    //////////////////////////////////////////////////////////
    //PLANNING
    
    pauseGazebo.call(pauseSrv);
   
    if(pauseGazebo.call(pauseSrv))
       ROS_INFO("Simulation paused.");
    else
       ROS_INFO("Failed to pause simulation.");
    

    /////////////////ARTPOT///////////////////////

    
    //_ready=true;

    //artpot
    //while(!_ref_ready)
    //usleep(0.1*1e6);

    /*std::cout<<"total force: "<<glob_plan->get_total_force()<<endl;

    std::cout<<"ref pos: "<<_pos<<endl;

    std::cout<<"ref pos old: "<<_pos_old<<endl;

    std::cout<<"ref vel: "<<_vel<<endl;

    std::cout<<"ref vel dot: "<<_vel_old<<endl;

    std::cout<<"fin pos: "<<_fin_pos<<endl;*/

    //////////////////////////////////////////////

    //std::cout<<"inizio planning: "<<ros::Time::now()<<endl;

    plan->compute_planning(0);

    //std::cout<<"fine planning: "<<ros::Time::now()<<endl;

    //std::cerr<<"planning done!"<<endl;

    unpauseGazebo.call(unpauseSrv);
 
    ///////////////////////STAND//////////////////////////

    _duration=plan->get_formulation().params_.ee_phase_durations_.at(1)[0];

    //std::cout<<"duration stand_phase:"<<_duration<<endl;

    //std::cout<<"INIZIO STAND 1"<<endl;

    _begin = ros::Time::now(); 

    _begin_first = 0.0; 

    stand_phase();

    //std::cout<<"FINE STAND 1"<<endl;

    //ros::spinOnce();

    //update(0);


    //////////////////////////////////////////////////////

    //////////////////////SWING///////////////////////////

    _duration=plan->get_formulation().params_.ee_phase_durations_.at(1)[1]+plan->get_formulation().params_.ee_phase_durations_.at(1)[0];

    //std::cout<<"duration swing phase:"<<_duration<<endl;

    //std::cout<<"INIZIO SWING 1"<<endl;

    swing_phase();

    //std::cout<<"FINE SWING 1"<<endl;

    //ros::spinOnce();

    //update(1);

    ////////////////////////////////////////////////////

    ///////////////////STAND///////////////////////////

    _duration=plan->get_formulation().params_.ee_phase_durations_.at(0)[0];

    //std::cout<<"duration stand phase:"<<_duration<<endl;

    //std::cout<<"INIZIO STAND 2"<<endl;

    stand_phase();  

    //publish
    _begin_first=(ros::Time::now()).toSec();

    //std::cout<<"FINE STAND 2"<<endl;

    //ros::spinOnce();

    update(0);

    //////////////////////////////////////////////////

    _init_pos=_r_c;

    _init_vel=_v_c.block(0,0,6,1);

    

    ////////////REPLANNING////////////////////////////

    pauseGazebo.call(pauseSrv);

    if(pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused. Replanning...");
     else
        ROS_INFO("Failed to pause simulation.");  

    plan->compute_planning(1);    

    unpauseGazebo.call(unpauseSrv);

    ///////////////////////////////////////////////////

    ///////////////////STAND///////////////////////////

    _duration=plan->get_formulation().params_.ee_phase_durations_.at(0)[0];

    //std::cout<<"duration stand phase:"<<_duration<<endl;

    _begin=ros::Time::now();

    //std::cout<<"INIZIO STAND 3"<<endl;

    stand_phase();  

    //std::cout<<"FINE STAND 3"<<endl;

    //update(0);

    //////////////////////////////////////////////////

    

    //////////////////SWING 2/////////////////////////


    _duration=plan->get_formulation().params_.ee_phase_durations_.at(0)[0] + plan->get_formulation().params_.ee_phase_durations_.at(0)[1];
    
    //std::cout<<"duration swing phase 2:"<<_duration<<endl;

    //std::cout<<"INIZIO SWING 2"<<endl;

    swing_phase2();

    //std::cout<<"FINE SWING 2"<<endl; 

    //ros::spinOnce();

    //update(2);

    //////////////////////////////////////////////////

    ///////////////////STAND//////////////////////////

    //_duration=plan->get_formulation().params_.ee_phase_durations_.at(0)[0] + plan->get_formulation().params_.ee_phase_durations_.at(0)[1] + plan->get_formulation().params_.ee_phase_durations_.at(0)[2];
    
    
    _duration=plan->get_formulation().params_.ee_phase_durations_.at(1)[0];

    //std::cout<<"duration stand phase:"<<_duration<<endl;

    //std::cout<<"INIZIO STAND 4"<<endl;

    stand_phase();

    //std::cout<<"FINE STAND 4"<<endl;

    //ros::spinOnce();

    //update(0);

    


    //_already_plan=false;



    /////////////////////////////////////////////////


    //r.sleep();
    
    }
}

int QUADRUPED::update(int flag){

    ////////////UPDATE////////////////////////////
    bool ok;
    //std::cout<<"tempo prim di setrobot: "<<ros::Time::now().toSec()<<endl;

    

    iDynTree::Transform world_H_base;
    iDynTree::fromEigen(world_H_base,quad->get_world_base());

    Eigen::Vector3d worldeigen=toEigen(world_H_base.getPosition());

    while (worldeigen==Eigen::Vector3d::Zero()){
        iDynTree::fromEigen(world_H_base,quad->get_world_base());
        worldeigen=toEigen(world_H_base.getPosition());
    }

    

    index++;

    //Now we create the KinDynComputations class, so we can set the state (TRAVERSARO)
    kinDynComp.setRobotState(iDynTree::make_matrix_view(_eig_robot_state.world_H_base),
                             iDynTree::make_span(_eig_robot_state.jointPos),
                             iDynTree::make_span(_eig_robot_state.baseVel),
                             iDynTree::make_span(_eig_robot_state.jointVel),
                             iDynTree::make_span(_eig_robot_state.gravity));

    //Once we called the setRobotState, we can call all the methods of KinDynComputations

    //std::cout<<"tempo dopo di setrobot: "<<ros::Time::now().toSec()<<endl;

    worldeigen=toEigen(kinDynComp.getCenterOfMassPosition());

    while (worldeigen==Eigen::Vector3d::Zero()){
        //iDynTree::fromEigen(world_H_base,quad->get_world_base());
        worldeigen=toEigen(kinDynComp.getCenterOfMassPosition());
    }

    if(index>=2){
        worldeigen=toEigen(kinDynComp.getCenterOfMassVelocity());

        while (worldeigen==Eigen::Vector3d::Zero()){
            //iDynTree::fromEigen(world_H_base,quad->get_world_base());
            worldeigen=toEigen(kinDynComp.getCenterOfMassVelocity());
        }


    }
    

    ok = kinDynComp.getFreeFloatingMassMatrix(iDynTree::make_matrix_view(_eig_mass_matrix));

    if (!ok)
    {
        std::cerr << "Matrix of wrong size passed to KinDynComputations::getFreeFloatingMassMatrix" << std::endl;
        //return EXIT_FAILURE;
    }

    //std::cout<<"tempo dopo mass matrix: "<<ros::Time::now().toSec()<<endl;
    
    /*ok = kinDynComp.getFrameFreeFloatingJacobian(arbitraryFrameIndex, iDynTree::make_matrix_view(_eig_jacobian));

    if (!ok)
    {
        std::cerr << "Wrong frame index or wrong size passed to KinDynComputations::getFrameFreeFloatingJacobian" << std::endl;
        //return EXIT_FAILURE;
    }

    std::cout<<"tempo dopo eigjac: "<<ros::Time::now().toSec()<<endl;

    ok = kinDynComp.getCenterOfMassJacobian(iDynTree::make_matrix_view(_eig_com_jacobian));

    if (!ok)
    {
        std::cerr << "Matrix of wrong size passed to KinDynComputations::getCenterOfMassJacobian" << std::endl;
        //return EXIT_FAILURE;
    }

    std::cout<<"tempo dopo eigcomjac: "<<ros::Time::now().toSec()<<endl;*/

    //////////////////////////////////////////////////

    /////////COMPUTE R_C//////////////////////////////   
    
    
    ///////////////////////////////////////
    //UPDATE RC
    //quad->set_pc(iDynTree::toEigen(kinDynComp.getCenterOfMassPosition()));

    _pc=iDynTree::toEigen(kinDynComp.getCenterOfMassPosition());

    //std::cout<<"tempo dopo pc: "<<ros::Time::now().toSec()<<endl;

    //Eigen::Matrix<double,6,1> r_c_temp;    
    //r_c_temp(0,0)=quad->get_pc()[0];
    //r_c_temp(1,0)=quad->get_pc()[1];
    //r_c_temp(2,0)=quad->get_pc()[2];

    for(int i=0; i<3; i++){
        _r_c(i,0)=_pc[i];
    }

    //iDynTree::Vector3 base_angle=world_H_base.getRotation().asRPY();
    //Eigen::Vector3d base_angle_eig = toEigen(base_angle);




    if(_ang_prec<0.0 && std::abs(_ang_prec)>3.0){
       _ang_was_negative=true;
    }

    if(_ang_prec>0.0  && std::abs(_ang_prec)>3.0){
        _ang_was_positive=true;
    }


    iDynTree::Vector3 base_angle=world_H_base.getRotation().asRPY();
    Eigen::Vector3d base_angle_eig = toEigen(base_angle);

    _ang_prec=base_angle_eig[2];

    

    if(base_angle_eig[2]>0.0 && _ang_was_negative){
        base_angle_eig[2]=base_angle_eig[2]-2*M_PI;
    }

    if(base_angle_eig[2]<0.0 && _ang_was_positive){
        base_angle_eig[2]=base_angle_eig[2]+2*M_PI;
    }

    if(std::abs(base_angle_eig[2])>=2*3.14){
        base_angle_eig[2]=0.0;
        _ang_was_negative=false;
        _ang_was_positive=false;

    }

    _r_c(3,0)=base_angle_eig[0];
    _r_c(4,0)=base_angle_eig[1];
    _r_c(5,0)=base_angle_eig[2];

    


    //quad->set_r_c(r_c_temp);

    //std::cout<<"tempo dopo rc: "<<ros::Time::now().toSec()<<endl;

    //////////////////////////////////////////

    //////////////////////////////////////////
    //UPDATE RC_DOT

    //quad->set_pc_dot(iDynTree::toEigen(kinDynComp.getCenterOfMassVelocity()));

    _pc_dot=iDynTree::toEigen(kinDynComp.getCenterOfMassVelocity());

    //std::cout<<"tempo dopo pcdot: "<<ros::Time::now().toSec()<<endl;

    //Eigen::Matrix<double,6,1> r_c_dot_temp;
    //center of mass linear velocity
    //for(int i=0; i<3; i++){
    //    r_c_dot_temp(i,0)=quad->get_pc_dot()(i);
    //}
    
    for(int i=0; i<3; i++){
        _r_c_dot(i,0)=_pc_dot(i);
    }

    //orientation
    //for(int i=3; i<6; i++){
    //    r_c_dot_temp(i,0)=_eig_robot_state.baseVel[i];
    //}

    for(int i=3; i<6; i++){
        _r_c_dot(i,0)=_eig_robot_state.baseVel[i];
    }

    //quad->set_r_c_dot(r_c_dot_temp);

    //std::cout<<"tempo dopo rcdot: "<<ros::Time::now().toSec()<<endl;

    //////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////
    //UPDATE V_C

    for(int i=0; i<3; i++){
        _v_c(i,0)=_pc_dot[i];
    }

    for(int i=3; i<6;i++){
        _v_c(i,0)=_eig_robot_state.baseVel[i];
    }

    for(int i=6; i<6+12; i++){
        //_v_c(i,0)=_eig_robot_state.jointVel[i-6];
        _v_c(i,0)=_eig_robot_state.jointVel(i-6,0);
    }

    //std::cout<<"tempo dopo vc: "<<ros::Time::now().toSec()<<endl;

    ////////////////////////////////////////////////////////////

    //Stacking v
    //_eig_robot_state.v.resize(_eig_robot_state.baseVel.size() + _eig_robot_state.jointVel.size(),1);
    //_eig_robot_state.v.block(0,0,6,1)=_eig_robot_state.baseVel;
    //_eig_robot_state.v.block(6,0,12,1)=_eig_robot_state.jointVel;

    //Stacking a
    /*_eig_robot_acceleration.a.resize(_eig_robot_acceleration.baseAcc.size() + _eig_robot_acceleration.jointAcc.size());
    _eig_robot_acceleration.a << _eig_robot_acceleration.baseAcc, 
                                 _eig_robot_acceleration.jointAcc;*/

    //Selection matrix
    
    //_select_matrix = Eigen::MatrixXd::Zero(12,6+12);
    //_select_matrix.block(0,0,12,6)=Eigen::MatrixXd::Zero(12,6);
    //_select_matrix.block(0,6,12,12)=Eigen::MatrixXd::Identity(12, 12);
    
    //Pseudo-inverse for the velocity vector v (needed to compute the Coriolis matrix)
    //_v_pseudo.resize(1, _eig_robot_state.baseVel.size() + _eig_robot_state.jointVel.size());
    //_v_pseudo = Eigen::MatrixXd::Zero(1, _eig_robot_state.baseVel.size() + _eig_robot_state.jointVel.size());
    //_v_pseudo = _eig_robot_state.v.completeOrthogonalDecomposition().pseudoInverse();

    compute_m22_bar();

    compute_c2_bar();
    
    compute_j_st(flag);

    compute_j_st_j_bar(flag);

    _bias_com=_t_bar.transpose().inverse()*_h+_t_bar.transpose().inverse()*_eig_mass_matrix*_t_inv_der*_v_c;

    
    return 0;
    

}

void QUADRUPED::run() {

    ros::MultiThreadedSpinner spinner(2);

    boost::thread loop_t ( &QUADRUPED::loop, this );

    //boost::thread artpot_t ( &QUADRUPED::artpot, this );

    //boost::thread compute_estimation_t ( &QUADRUPED::compute_estimation, this);

    boost::thread scan_t ( &QUADRUPED::camera_scan, this );
    

    //ros::spin();
   
    spinner.spin();

}

/////////SECONDO THREAD///////////////////

/*void QUADRUPED::artpot(){

    ros::Rate r(20);

    while(!_ready){
        std::cout<<"in attesa di update..."<<endl;
    }

    while(ros::ok()){
        

            glob_plan->set_goal(-1.0, -2.0, -0.785);

            std::cout<<"dopo set goal"<<endl;

            if(!_already_set_old){
                _vel_old=_r_c_dot;
                _pos_old=_r_c;
                _already_set_old=true;
            }

            std::cout<<"dopo set old"<<endl;
        
            _vel = _vel_old + 0.05*glob_plan->get_total_force();

            _pos = _pos_old + 0.05*_vel_old; //uso il vecchio valore di velocità

            _vel_old=_vel;

            _pos_old=_pos; 

            std::cout<<"dopo vel e pos"<<endl;
    
            _fin_pos=_pos;             
            
            _ref_ready=true;         

            
        r.sleep();

    }
    
}*/

//////////////////////////////////////////////////

//////TERZO THREAD////////////////////////////////

/*void QUADRUPED::compute_estimation(){

    ros::Rate r(1000);

       

    while(!_ready_est){
        std::cout<<"in attesa di update..."<<endl;
    }

    while(ros::ok()){

       

        Tbl=kinDynComp.getWorldTransform(8);
        _Tbl=toEigen(Tbl.getRotation());
        std::cout<<"dopo 8"<<endl;

        Tbr=kinDynComp.getWorldTransform(11);
        _Tbr=toEigen(Tbr.getRotation());
        std::cout<<"dopo 11"<<endl;

        Tfl=kinDynComp.getWorldTransform(14);
        _Tfl=toEigen(Tfl.getRotation());
        std::cout<<"dopo 14"<<endl;

        Tfr=kinDynComp.getWorldTransform(17);
        _Tfr=toEigen(Tfr.getRotation());
        std::cout<<"dopo 17"<<endl;
        

        _rho=_m_bar.block(6,6,12,12)*_v_c.block(6,0,12,1);
    
        _Fgrf<< _Tbl*_f_gr.block(6,0,3,1),
                _Tbr*_f_gr.block(9,0,3,1),
                _Tfl*_f_gr.block(3,0,3,1), 
                _Tfr*_f_gr.block(0,0,3,1); 

        
       
            
        _integral_1_dot=_gamma_1 + _bias_com.block(6,0,12,1) + _jnt_torques_star + _j_st_j_bar_0.transpose()*_Fgrf;
        _integral_1=_integral_1_old + 0.001*_integral_1_dot;
        //_gamma_1=((Eigen::Matrix<double,12,12>::Identity()+_K_1*0.001).inverse()*_K_1)*(_rho - _integral_1);
        _gamma_1=_K_1*_rho - _K_1*_integral_1;
        _integral_1_old=_integral_1;
        

        std::cout<<"integral 1 dot: "<<_integral_1_dot<<endl; //stampo
    
    
        _integral_2_dot=-_gamma_3 + _gamma_1;
        _integral_2= _integral_2_old + 0.001*_integral_2_dot;
        _gamma_2=_K_2*_integral_2;
        //_gamma_2=((Eigen::Matrix<double,12,12>::Identity()+_K_2*0.001).inverse()*_K_2)*_integral_2;

    
        _integral_3_dot=-_gamma_3 + _gamma_2;
        _integral_3= _integral_3_old + 0.001*_integral_3_dot;
        _gamma_3=_K_3*_integral_3;
        //_gamma_3=((Eigen::Matrix<double,12,12>::Identity()+_K_3*0.001).inverse()*_K_3)*_integral_3;
    
        _integral_1_old=_integral_1;
        _integral_2_old=_integral_2;
        _integral_3_old=_integral_3;

        _f_ext=(_B*_j_bar).block(0,6,12,12).transpose().inverse()*_gamma_1;
    


        r.sleep();
    }

}*/

////////////QUARTO THREAD/////////////////////

void QUADRUPED::camera_scan(){

    ros::Rate r(1);

    
    while(ros::ok()){

        scan->qrcode_scanning();      

        r.sleep();

    }
    
}

////////////////////////////////////////////////

///////////////QUINTO THREAD///////////////////

/*void QUADRUPED::publish_values(){

    ros::Rate publish_rate(1000);

    while(!_start_stand)
    usleep(0.1*1e6);

    double time_stamp=0.0;

    while(ros::ok()){

        std_msgs::Float64MultiArray array_temp;

        array_temp.data.resize(13);

        Tbl=kinDynComp.getWorldTransform(8);
        _Tbl=toEigen(Tbl.getRotation());

        Tbr=kinDynComp.getWorldTransform(11);
        _Tbr=toEigen(Tbr.getRotation());

        Tfl=kinDynComp.getWorldTransform(14);
        _Tfl=toEigen(Tfl.getRotation());

        Tfr=kinDynComp.getWorldTransform(17);
        _Tfr=toEigen(Tfr.getRotation());
        
    
        _Fgrf<< _Tbl*_f_gr.block(6,0,3,1),
                _Tbr*_f_gr.block(9,0,3,1),
                _Tfl*_f_gr.block(3,0,3,1), 
                _Tfr*_f_gr.block(0,0,3,1); 


        array_temp.data[0]=_Fgrf(0,0);
        array_temp.data[1]=_Fgrf(1,0);
        array_temp.data[2]=_Fgrf(2,0);

        array_temp.data[3]=_Fgrf(3,0);
        array_temp.data[4]=_Fgrf(4,0);
        array_temp.data[5]=_Fgrf(5,0);

        array_temp.data[6]=_Fgrf(6,0);
        array_temp.data[7]=_Fgrf(7,0);
        array_temp.data[8]=_Fgrf(8,0);

        array_temp.data[9]=_Fgrf(9,0);
        array_temp.data[10]=_Fgrf(10,0);
        array_temp.data[11]=_Fgrf(11,0);

        array_temp.data[12]=time_stamp;

        _pub_val.publish(array_temp);

        time_stamp+=0.001;

        publish_rate.sleep();

    }
}*/





  




int main(int argc, char **argv){

	ros::init(argc, argv,"compute_torque_node");     

    quad = new QUADRUPED();

    plan = new PLANNING(*quad);

    opti = new OPTIMAL(*quad);

    //glob_plan = new GlobalPlanner(*quad);

    scan = new SCANNER(*quad);

    gazebo::transport::NodePtr node(new gazebo::transport::Node());	

    quad->_node=node;

    quad->_node->Init();  

    gazebo::client::setup(argc,argv);
    

    quad->run();
       


	return 0;
}
