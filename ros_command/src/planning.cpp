#include "planning.h"

using namespace towr;
using namespace std;



PLANNING::PLANNING(QUADRUPED &quadruped)

{
    dogbot = &quadruped;
}


void PLANNING::compute_planning(int flag){


    

        towr::NlpFormulation formulation;

        _formulation=formulation;
        
        //Eigen::Matrix<double,3,1> bl, br, fl, fr;
        //NlpFormulation formulation;

        _formulation.terrain_ = std::make_shared<towr::FlatGround>(0.0); 
    
        _formulation.model_ = towr::RobotModel(towr::RobotModel::Dogbot); //change in dogbot?  
      
        _formulation.initial_base_.lin.at(towr::kPos) << dogbot->get_init_pos()(0,0), dogbot->get_init_pos()(1,0), dogbot->get_init_pos()(2,0);
        _formulation.initial_base_.ang.at(towr::kPos) << dogbot->get_init_pos()(3,0), dogbot->get_init_pos()(4,0), dogbot->get_init_pos()(5,0); 
        _formulation.initial_base_.lin.at(towr::kVel) << dogbot->get_init_vel()(0,0), dogbot->get_init_vel()(1,0), dogbot->get_init_vel()(2,0); 
        _formulation.initial_base_.ang.at(towr::kVel) << dogbot->get_init_vel()(3,0), dogbot->get_init_vel()(4,0), dogbot->get_init_vel()(5,0); 

        /*std::cout<<"init_pos"<<endl;
        std::cout<<dogbot->get_init_pos()(0,0)<<endl;
        std::cout<<dogbot->get_init_pos()(1,0)<<endl;
        std::cout<<dogbot->get_init_pos()(2,0)<<endl;
        std::cout<<dogbot->get_init_pos()(3,0)<<endl;
        std::cout<<dogbot->get_init_pos()(4,0)<<endl;
        std::cout<<dogbot->get_init_pos()(5,0)<<endl;

        std::cout<<"init_vel"<<endl;
        std::cout<<dogbot->get_init_vel()(0,0)<<endl;
        std::cout<<dogbot->get_init_vel()(1,0)<<endl;
        std::cout<<dogbot->get_init_vel()(2,0)<<endl;
        std::cout<<dogbot->get_init_vel()(3,0)<<endl;
        std::cout<<dogbot->get_init_vel()(4,0)<<endl;
        std::cout<<dogbot->get_init_vel()(5,0)<<endl;*/


        
        if(_step_1){ //cammina
            _formulation.final_base_.lin.at(towr::kPos) << 1.0,_formulation.initial_base_.lin.at(towr::kPos)[1]-0.04, 0.40229;
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, 0.0;

            if(dogbot->get_pc()(1)<=-2.9){
                _step_1=false;
                _step_2=true;
            }

        }

        if(_step_2){ //svolta
            
            //_formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0]+0.03*std::sin(_formulation.initial_base_.ang.at(towr::kPos)[2]), _formulation.initial_base_.lin.at(towr::kPos)[1]-0.03*std::cos(_formulation.initial_base_.ang.at(towr::kPos)[2]), 0.40229;
            
            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0],_formulation.initial_base_.lin.at(towr::kPos)[1], 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2]+(1*M_PI)/180;

            //_formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2]+(1*M_PI)/180;

            if(_formulation.initial_base_.ang.at(towr::kPos)[2] >=1.57){
                _step_2=false;
                _step_3=true;
            }

            _y_temp=_formulation.initial_base_.lin.at(towr::kPos)[1];

            _yaw_temp=_formulation.initial_base_.ang.at(towr::kPos)[2];
   
        }
            
        

        if(_step_3){ //cammina

            
            
            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0]+0.04, _y_temp, 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _yaw_temp;

            if(dogbot->get_pc()(0)>=6.0){
                //_start_scan=true; //if a further analysis is needed, dogbot can stop and analyse the qrcode deeply
                _step_3=false;
                _step_4=true;
            }
   
        }

        if(_step_4){ //svolta

            //if(_start_scan){
//
            //    _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0],_formulation.initial_base_.lin.at(towr::kPos)[1], 0.40229;            
            //    _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2];
//
            //}else
            
            
            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0],_formulation.initial_base_.lin.at(towr::kPos)[1], 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2]+(1*M_PI)/180;

            if(_formulation.initial_base_.ang.at(towr::kPos)[2] >=4.71){
                _step_4=false;
                _step_5=true;
            }  

            _y_temp=_formulation.initial_base_.lin.at(towr::kPos)[1];          
   
        }

        

        if(_step_5){ //cammina

            //double y_temp=_formulation.initial_base_.lin.at(towr::kPos)[1];
            
            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0]-0.04, _y_temp, 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2];

           if(_formulation.initial_base_.lin.at(towr::kPos)[0] <=-1.0 ){
                _step_5=false;
                _step_6=true;
            }

            _yaw_temp=_formulation.initial_base_.ang.at(towr::kPos)[2];
   
        }

        

        if(_step_6){ //svolta

            
            
            //_formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0]+0.03*std::sin(_formulation.initial_base_.ang.at(towr::kPos)[2]), _formulation.initial_base_.lin.at(towr::kPos)[1]-0.03*std::cos(_formulation.initial_base_.ang.at(towr::kPos)[2]), 0.40229;
            
            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0],_formulation.initial_base_.lin.at(towr::kPos)[1], 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2]-(1*M_PI)/180;

            //_formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2]+(1*M_PI)/180;

            if(dogbot->get_yaw()<=3.25){
                _step_6=false;
                _step_7=true;
            }

            _x_temp=_formulation.initial_base_.lin.at(towr::kPos)[0];
   
        }

        if(_step_7){ //cammina
 
            
            _formulation.final_base_.lin.at(towr::kPos) << _x_temp, _formulation.initial_base_.lin.at(towr::kPos)[1]+0.04, 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2];

           if(_formulation.initial_base_.lin.at(towr::kPos)[1] >=0.0){
                _step_7=false;
                _step_8=true;
            }
   
        }

        if(_step_8){ //svolta
            
            
            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0],_formulation.initial_base_.lin.at(towr::kPos)[1], 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2]+(1*M_PI)/180;

            if(_formulation.initial_base_.ang.at(towr::kPos)[2] >=4.71){
                _step_8=false;
                _step_9=true;
            }   

            _y_temp=_formulation.initial_base_.lin.at(towr::kPos)[1];

            _yaw_temp=_formulation.initial_base_.ang.at(towr::kPos)[2];         
   
        }


         if(_step_9){ //cammina

            
            
            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0]-0.04, _y_temp, 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _yaw_temp;

            if(dogbot->get_pc()(0)<=-6.0){
                //_start_scan=true; //if a further analysis is needed, dogbot can stop and analyse the qrcode deeply
                _step_9=false;
                _step_10=true;
            }
   
        }

        if(_step_10){ //svolta
            
            //_formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0]+0.03*std::sin(_formulation.initial_base_.ang.at(towr::kPos)[2]), _formulation.initial_base_.lin.at(towr::kPos)[1]-0.03*std::cos(_formulation.initial_base_.ang.at(towr::kPos)[2]), 0.40229;
            
            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0], _formulation.initial_base_.lin.at(towr::kPos)[1], 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2]-(1*M_PI)/180;

            //_formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2]+(1*M_PI)/180;

            if(dogbot->get_yaw() <= 3.19){
                _step_10=false;
                _step_11=true;
            }

            _x_temp=_formulation.initial_base_.lin.at(towr::kPos)[0];

            _yaw_temp=_formulation.initial_base_.ang.at(towr::kPos)[2];
   
        }

        if(_step_11){ //cammina

            
            
            _formulation.final_base_.lin.at(towr::kPos) << _x_temp, _formulation.initial_base_.lin.at(towr::kPos)[1]+0.04, 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _yaw_temp;

            if(dogbot->get_pc()(1)>=3.0){
                //_start_scan=true; //if a further analysis is needed, dogbot can stop and analyse the qrcode deeply
                _step_11=false;
                _step_12=true;
            }
   
        }

        if(_step_12){ //svolta

            
            
            
            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0],_formulation.initial_base_.lin.at(towr::kPos)[1], 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2]-(1*M_PI)/180;

            if(_formulation.initial_base_.ang.at(towr::kPos)[2] <=0.05){
                _step_12=false;
                _step_13=true;
            }  

            _x_temp=_formulation.initial_base_.lin.at(towr::kPos)[0]; 

            _yaw_temp=_formulation.initial_base_.ang.at(towr::kPos)[2];         
   
        }

        if(_step_13){ //cammina

            //double y_temp=_formulation.initial_base_.lin.at(towr::kPos)[1];
            
            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0]+0.002, _formulation.initial_base_.lin.at(towr::kPos)[1]-0.04, 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _yaw_temp;

           if(_formulation.initial_base_.lin.at(towr::kPos)[1] <=-3.5 ){
                _step_13=false;
                _step_14=true;
            }

            _yaw_temp=_formulation.initial_base_.ang.at(towr::kPos)[2];
   
        }

        if(_step_14){ //svolta

            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0],_formulation.initial_base_.lin.at(towr::kPos)[1], 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2]-(1*M_PI)/180;

            if(dogbot->get_yaw() <=-3.08){
                _step_14=false;
                _step_15=true;
                //_start_scan=true; 
            }  

            _yaw_temp=_formulation.initial_base_.ang.at(towr::kPos)[2];

        }

        if(_step_15){ //cammina

            
            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0]-0.001,_formulation.initial_base_.lin.at(towr::kPos)[1]+0.04, 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _yaw_temp;

            if(_formulation.initial_base_.lin.at(towr::kPos)[1] >=0.0){
                _step_15=false;
                _step_16=true;
                //_start_scan=true; 
            }  

   
        }


        //if(_step_16){ //svolta
//
        //    _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0],_formulation.initial_base_.lin.at(towr::kPos)[1], 0.40229;
        //    
        //    _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2]-(1*M_PI)/180;
//
        //    if(dogbot->get_yaw() <=-0.3){
        //        _step_16=false;
        //        _step_17=true;
        //        //_start_scan=true; 
        //    }  
//
        //}

        if(_step_16){ //cammina

            
            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0]+0.04,_formulation.initial_base_.lin.at(towr::kPos)[1], 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _yaw_temp;

            if(_formulation.initial_base_.lin.at(towr::kPos)[0] >=1.0){
                _step_16=false;
                _step_17=true;
                //_start_scan=true; 
            }  

   
        }

        //if(_step_18){ //svolta
//
        //    _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0],_formulation.initial_base_.lin.at(towr::kPos)[1], 0.40229;
        //    
        //    _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2]-(1*M_PI)/180;
//
        //    //if(dogbot->get_yaw() <=-0.3){
        //    //    _step_18=false;
        //    //    _step_19=true;
        //    //    _start_scan=true; 
        //    //}  
//
        //}

        if(_step_17){ //svolta

            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0]-0.001,_formulation.initial_base_.lin.at(towr::kPos)[1]+0.04, 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2];

            if(_formulation.initial_base_.lin.at(towr::kPos)[1] >=3.0){
                _step_17=false;
                _step_18=true;
                _start_scan=true; 
            }  

        }





        if(_step_18){ //svolta

            
            
            
            _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0],_formulation.initial_base_.lin.at(towr::kPos)[1], 0.40229;
            
            _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2];

             

             

         
   
        }









        

        

   
        
        //artpot
        //_formulation.final_base_.lin.at(towr::kPos) << dogbot->get_fin_pos()(0,0), dogbot->get_fin_pos()(1,0), 0.40229;
        //_formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, dogbot->get_fin_pos()(5,0);//_formulation.initial_base_.ang.at(towr::kPos)[2];//dogbot-
        
        
    
    

        auto stance = _formulation.model_.kinematic_model_->GetNominalStanceInBase();
        _formulation.initial_ee_W_ = stance;

        Eigen::Vector3d pos_ee;
        for (int ee=0; ee<4; ee++){
    
            switch(ee){
    
                case 0: pos_ee=dogbot->getBLpos();
                break;
                case 1: pos_ee=dogbot->getBRpos();
                break;
                case 2: pos_ee=dogbot->getFLpos();
                break;
                case 3: pos_ee=dogbot->getFRpos();
                break;
            }
    
            _formulation.initial_ee_W_.at(ee)[0]=pos_ee[0];
            _formulation.initial_ee_W_.at(ee)[1]=pos_ee[1];
        }



        std::for_each(_formulation.initial_ee_W_.begin(), _formulation.initial_ee_W_.end(),
                      [&](Eigen::Vector3d& p){ p.z() = 0.0; }); 

    
        auto gategen = towr::GaitGenerator::MakeGaitGenerator(4); 

        if(flag==0){
            auto gait_type = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C5);
            gategen->SetCombo(gait_type);
        }
        else if(flag==1){
            auto gait_type = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C6);
            gategen->SetCombo(gait_type);
        }
        else if(flag==2){
            auto gait_type = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C7);
            gategen->SetCombo(gait_type);
        }
       

        //gategen->SetGaits({_stand, _run}); // Stand + Run1

        _formulation.params_.ee_phase_durations_.clear();
        for(int i=0;i<4;++i){
            _formulation.params_.ee_phase_durations_.push_back(gategen->GetPhaseDurations(0.5,i)); 
  	        _formulation.params_.ee_in_contact_at_start_.push_back(gategen->IsInContactAtStart(i));
        }
    

        ifopt::Problem nlp;

        towr::SplineHolder solution;

        _solution=solution;
    
        //Init nonlinear programming with vars, constraints and costs
        for(auto c: _formulation.GetVariableSets(_solution))
            nlp.AddVariableSet(c);

        for(auto c: _formulation.GetConstraints(_solution))
            nlp.AddConstraintSet(c);

        for(auto c: _formulation.GetCosts())
            nlp.AddCostSet(c);


        auto solver = std::make_shared<ifopt::IpoptSolver>();
        solver -> SetOption("jacobian_approximation","exact");
        solver -> SetOption("max_cpu_time",20.0);
        //solver -> SetOption("print_level", 5);
        solver -> Solve(nlp);


       
    
	/*double t = 0.0;
		
		
	nlp.PrintCurrent(); 
	while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
		
		cout << "t=" << t << "\n";
		cout << "Base linear position x,y,z:   \t";
		
		cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;
		cout << "Base Euler roll, pitch, yaw:  \t";
		Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
		cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;
		
		for( int i=0; i<4; i++ ) {
			cout << "Foot [" << (i+1) << "] position x,y,z: ";
			cout << solution.ee_motion_.at(i)->GetPoint(t).p().transpose() << "\t[m]" << endl;
			cout << "Contact force x,y,z:          \t";
			cout << solution.ee_force_.at(i)->GetPoint(t).p().transpose() << "\t[N]" << endl;
			bool contact = solution.phase_durations_.at(i)->IsContactPhase(t);
			std::string foot_in_contact = contact? "yes" : "no";
			cout << "Foot in contact:              \t" + foot_in_contact << endl;
		}			
		cout << endl;
		t += 0.2;
	}*/


		
}

towr::NlpFormulation PLANNING::get_formulation(){
    return _formulation;
}

towr::SplineHolder PLANNING::get_solution(){
    return _solution;
}

double PLANNING::get_start_scan(){
    return _start_scan;
}
