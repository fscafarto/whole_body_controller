#include "optimal.h"

OPTIMAL::OPTIMAL(QUADRUPED &quadruped){
    dogbot = &quadruped;

    initialization();

}

void OPTIMAL::compute_w_com_des(){

    
    
    
    Eigen::Matrix<double,6,1> w_com_des_temp=Eigen::MatrixXd::Zero(6,1);

    _k_p=3500*Eigen::MatrixXd::Identity(6,6); //3500

    _k_d=50*Eigen::MatrixXd::Identity(6,6); //50

    //std::cerr<<"riferimenti posizione:"<<endl;
    //std::cerr<<dogbot->get_r_c_ref()<<endl;

    Eigen::Matrix<double,6,1> temp_matrix=Eigen::MatrixXd::Zero(6,1);
    temp_matrix=dogbot->get_r_c() - dogbot->get_r_c_ref();

    temp_matrix.block(3,0,3,1)=(dogbot->get_world_base()).block(0,0,3,3)*temp_matrix.block(3,0,3,1);

    w_com_des_temp= -_k_p*temp_matrix -_k_d*(dogbot->get_r_c_dot() - dogbot->get_r_c_ref_dot()) + dogbot->get_m()*g + dogbot->get_mass_matrix().block(0,0,6,6)*dogbot->get_r_c_ref_dot_dot();

    dogbot->set_w_com_des(w_com_des_temp);

    /*if(flag==1 || flag==2){
        dogbot->set_w_com_des(w_com_des_temp);
    }*/
}

void OPTIMAL::compute_d_fr(int flag){


    if(flag==0){

       for(int i=0; i<4; i++){

            _d_fr_0.block<5,3>(i*5,i*3) << (_t1 - dogbot->get_mu()*_n).transpose(),
                                          -(_t1 + dogbot->get_mu()*_n).transpose(),
                                           (_t2 - dogbot->get_mu()*_n).transpose(),
                                          -(_t2 + dogbot->get_mu()*_n).transpose(),
                                          -_n.transpose();
        }

    }

    if(flag==1 || flag==2){

        for(int i=0; i<2; i++){

            _d_fr_1.block<5,3>(i*5,i*3) << (_t1 - dogbot->get_mu()*_n).transpose(),
                                          -(_t1 + dogbot->get_mu()*_n).transpose(),
                                           (_t2 - dogbot->get_mu()*_n).transpose(),
                                          -(_t2 + dogbot->get_mu()*_n).transpose(),
                                          -_n.transpose();
        }

    }

}

void OPTIMAL::compute_optimization(int flag){
    //initialization();
  
    compute_w_com_des();

    compute_d_fr(flag);   

    //compute_estimation(flag); 

    //yd_prev = yd;
    //yw_prev = yw;

    /*_JacCOM_lin=_B*dogbot->get_j_bar();

    Eigen::Matrix<double,12,1> w3=dogbot->get_f_ext();
    

    std::cout<<"f cappello: "<<w3<<endl;

    std::cout<<"gamma 3: "<<dogbot->get_gamma_1()<<endl;

    Eigen::Matrix<double,12,1> f_st_hat_temp_0;
    f_st_hat_temp_0=Eigen::MatrixXd::Zero(12,1);

    Eigen::Matrix<double,6,1> f_st_hat_temp_1;
    f_st_hat_temp_1=Eigen::MatrixXd::Zero(6,1);

    Eigen::Matrix<double,6,1> f_sw_hat_temp;
    f_sw_hat_temp=Eigen::MatrixXd::Zero(6,1);*/

 

    /*if(flag==0){

        //f_st_hat_temp_0.block(0,0,3,1)=dogbot->get_gamma_3().block(0,0,3,1);
        //f_st_hat_temp_0.block(3,0,3,1)=dogbot->get_gamma_3().block(3,0,3,1);
        //f_st_hat_temp_0.block(6,0,3,1)=dogbot->get_gamma_3().block(6,0,3,1);
        //f_st_hat_temp_0.block(9,0,3,1)=dogbot->get_gamma_3().block(9,0,3,1);

        f_st_hat_temp_0.block(0,0,3,1)=w3.block(0,0,3,1);
        f_st_hat_temp_0.block(3,0,3,1)=w3.block(3,0,3,1);
        f_st_hat_temp_0.block(6,0,3,1)=w3.block(6,0,3,1);
        f_st_hat_temp_0.block(9,0,3,1)=w3.block(9,0,3,1);

        dogbot->set_f_st_hat_0(f_st_hat_temp_0);
    }*/
    

    /*if(flag==1 || flag==2){
        
        //f_st_hat_temp_1.block(0,0,3,1)=dogbot->get_gamma_3().block(dogbot->get_stl1(),0,3,1);
        //f_st_hat_temp_1.block(3,0,3,1)=dogbot->get_gamma_3().block(dogbot->get_stl2(),0,3,1);                
        //dogbot->set_f_st_hat_1(f_st_hat_temp_1);
        //f_sw_hat_temp.block(0,0,3,1)=dogbot->get_gamma_3().block(dogbot->get_swl1(),0,3,1);
        //f_sw_hat_temp.block(3,0,3,1)=dogbot->get_gamma_3().block(dogbot->get_swl2(),0,3,1);

        f_st_hat_temp_1.block(0,0,3,1)=w3.block(dogbot->get_stl1(),0,3,1);
        f_st_hat_temp_1.block(3,0,3,1)=w3.block(dogbot->get_stl2(),0,3,1);  
        dogbot->set_f_st_hat_1(f_st_hat_temp_1);

        f_sw_hat_temp.block(0,0,3,1)=w3.block(dogbot->get_swl1(),0,3,1);
        f_sw_hat_temp.block(3,0,3,1)=w3.block(dogbot->get_swl2(),0,3,1);            
        dogbot->set_f_sw_hat(f_sw_hat_temp);
    }*/


    if(flag==1 || flag==2){ //introdurre fext_lambda qui


       
	    //_Si<<Eigen::Matrix<double,6,18>::Zero(),
	    //    Eigen::Matrix<double,12,6>::Zero(),Eigen::Matrix<double,12,12>::Identity();        
        //_Jst.block(0,0,3,18)=_JacCOM_lin.block(dogbot->get_stl1(),0,3,18);
        //_Jst.block(3,0,3,18)=_JacCOM_lin.block(dogbot->get_stl2(),0,3,18);
        //_P=Eigen::Matrix<double,18,18>::Identity()-_Jst.transpose()*(_Jst*dogbot->get_mass_matrix().inverse()*_Jst.transpose()).inverse()*_Jst*dogbot->get_mass_matrix().inverse();

        

        //STIMATORE

        //_fext_lambda<<_JacCOM_lin.block(dogbot->get_swl1(),0,3,18)*dogbot->get_mass_matrix().inverse()*_Si.transpose()*_P*_JacCOM_lin.transpose()*w3,
	    //              _JacCOM_lin.block(dogbot->get_swl2(),0,3,18)*dogbot->get_mass_matrix().inverse()*_Si.transpose()*_P*_JacCOM_lin.transpose()*w3;

        _k_d_sw=50*Eigen::MatrixXd::Identity(6,6); //50

        _k_p_sw=250*Eigen::MatrixXd::Identity(6,6); //250
    
        //Eigen::Matrix<double,18,6> j_st_pseudo;
    
        //j_st_pseudo=dogbot->get_mass_matrix().inverse()*dogbot->get_j_st_1().transpose()*(dogbot->get_j_st_1()*dogbot->get_mass_matrix().inverse()*dogbot->get_j_st_1().transpose()).inverse();
    
        //_n_c=Eigen::MatrixXd::Identity(18,18) - j_st_pseudo*dogbot->get_j_st_1();
    
        //_m_c=_n_c*dogbot->get_mass_matrix() + Eigen::MatrixXd::Identity(18,18) - _n_c;
    
        _x_sw_cmd_dot_dot= dogbot->get_accd() + _k_d_sw*dogbot->get_veldelta() + _k_p_sw*dogbot->get_posdelta();// -_fext_lambda; //- dogbot->get_j_sw()*_m_c.inverse()*_n_c*dogbot->get_j_sw().transpose()*dogbot->get_f_sw_hat();

        //std::cout<<"xswcmddotdot: "<<_x_sw_cmd_dot_dot<<endl;
        
    }
       
    
    if(flag==0){ //qui manca un termine aggiuntivo sulla funzione di costo

        //std::cout<<"preparazione delle matrici per alglib"<<ros::Time::now().toSec()<<endl;

        

        //_A_0= _sigma_0.transpose()*dogbot->get_j_st_c_bar_0()*weight_1*dogbot->get_j_st_c_bar_0().transpose()*_sigma_0 + Eigen::MatrixXd::Identity(6+12+12,6+12+12);
    
        _A_0= _sigma_0.transpose()*dogbot->get_j_st_c_bar_0()*weight_1*dogbot->get_j_st_c_bar_0().transpose()*_sigma_0 + Eigen::MatrixXd::Identity(30,30);
        
        //_b1_0= dogbot->get_f_st_hat_0().transpose()*dogbot->get_j_st_c_bar_0()*dogbot->get_j_st_c_bar_0().transpose()*_sigma_0 - dogbot->get_w_com_des().transpose()*dogbot->get_j_st_c_bar_0().transpose()*_sigma_0;

        //STIMATORE
    
        _b2_0= -_sigma_0.transpose()*dogbot->get_j_st_c_bar_0()*weight_1.transpose()*dogbot->get_w_com_des();// +  _sigma_0.transpose()*dogbot->get_j_st_c_bar_0()*weight_1.transpose()*dogbot->get_j_st_c_bar_0().transpose()*dogbot->get_f_st_hat_0();
    
        //_b1_0=-_sigma_0.transpose()*dogbot->get_j_st_c_bar_0()*weight_1.transpose()*dogbot->get_w_com_des();
        
        ////_b1_0=-dogbot->get_w_com_des().transpose()*weight_1*dogbot->get_j_st_c_bar_0().transpose()*_sigma_0; //+ dogbot->get_f_st_hat_0().transpose()*dogbot->get_j_st_c_bar_0()*weight_1*dogbot->get_j_st_c_bar_0().transpose()*_sigma_0;
        
        //cost function
        
        A.setlength(_A_0.rows(),_A_0.cols());
        
    
        for(int i=0;i<_A_0.rows();i++){
            for(int j=0; j<_A_0.cols();j++){
                A(i,j)=_A_0(i,j);
            }
        }
    
        
        //b.setlength(_b1_0.cols());      
        //for(int i=0; i<_b1_0.cols(); i++){
        //    b(i)=_b2_0(i,0);//_b1_0(0,i);//+_b2_0(i,0);
        //}

        b.setlength(_b2_0.rows());
        for(int i=0; i<_b2_0.rows(); i++){
            b(i)=_b2_0(i,0);//_b1_0(0,i);//+_b2_0(i,0);
        }


    
        ///////////////////////////////////////////////////////////////
    
        //////////////////////////////////////////////////////////////
        //CONSTRAINTS('con' stands for the matrix which are passed to alglib script)
    
        _A_con_0.block(0,0,6,6)=dogbot->get_mass_matrix().block(0,0,6,6);
        //_A_con_0.block(0,6,6,12)=Eigen::MatrixXd::Zero(6,12);
        _A_con_0.block(0,18,6,12)=-dogbot->get_j_st_c_bar_0().transpose();
        //_A_con_0.block(0,6+12+12,6,3*dogbot->get_n_sw())=Eigen::MatrixXd::Zero(6,3*dogbot->get_n_sw());
    
        _A_con_0.block(6,0,12,6)=dogbot->get_j_st_c_bar_0();
        _A_con_0.block(6,6,12,12)=dogbot->get_j_st_j_bar_0();
        //_A_con_0.block(6,6+12,12,12)=Eigen::MatrixXd::Zero(12, 12);
        //_A_con.block(6,6+12+12,12,3*dogbot->get_n_sw())=Eigen::MatrixXd::Zero(3*dogbot->get_n_st(),3*dogbot->get_n_sw());
    

        
        ////////termine gravitazionale/////////////////
    
        //Eigen::Matrix<double,18,1> bias_com;
        //bias_com=dogbot->get_t_bar().transpose().inverse()*dogbot->get_h()+dogbot->get_t_bar().transpose().inverse()*dogbot->get_mass_matrix()*dogbot->get_t_inv_der()*dogbot->get_v_c(); 

        /*Eigen::Matrix<double,24,18> Jstcom;
        Eigen::Matrix<double,12,18> JacCOM_lin;
        Eigen::Matrix<double,12,24> B;
        B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
            Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
    	    Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
    	    Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);

        JacCOM_lin=B*dogbot->get_j_bar();*/

        //STIMATORE
        
        _b_con_0.block(0,0,6,1)=-dogbot->get_bias_com().block(0,0,6,1);// + _JacCOM_lin.block(0,0,12,6).transpose()*w3;
        
        ///////////////////////////////////////////////


        //_b_con_0.block(0,0,6,1)=-dogbot->get_mass_matrix()(0,0)*g;

        _b_con_0.block(6,0,12,1)=-dogbot->get_j_st_dot_0();

        //_b_con_0.block(6,0,12,1)=-dogbot->get_j_st_dot_0().block(0,0,12,6)*dogbot->get_v_c().block(0,0,6,1) - dogbot->get_j_st_dot_0().block(0,6,12,12)*dogbot->get_v_c().block(6,0,12,1);
    
        //put A and b together
       
        A_b_con.setlength(_A_con_0.rows() + _D_con_0.rows(), _A_con_0.cols()+1);
    
        for(int i=0;i<_A_con_0.rows();i++){
            for(int j=0; j< _A_con_0.cols();j++){
                A_b_con(i,j)=_A_con_0(i,j);
            }
        }
        
    
        //_D_con_0.block(0,0,4*4,6)=Eigen::MatrixXd::Zero(4*4,6);
        //_D_con_0.block(0,6,4*4,12)=Eigen::MatrixXd::Zero(4*4, 12);

        ///_D_con_0.block(0,6+12,4*4, 3*4)=_d_fr_0;
        _D_con_0.block(0,6+12,4*5, 3*4)=_d_fr_0;

        //_D_con.block(0,6+12+3*4,4*4,3*dogbot->get_n_sw())=Eigen::MatrixXd::Zero(4*4, 3*dogbot->get_n_sw());
    
        //_D_con_0.block(4*4,0,12,6)=Eigen::MatrixXd::Zero(12,6);
        _D_con_0.block(4*5,6,12,12)=dogbot->get_mass_matrix().block(6,6,12,12);
        _D_con_0.block(4*5,6+12,12,3*4)=-dogbot->get_j_st_j_bar_0().transpose();
        //_D_con.block(4*4,6+12+3*4,12, 3*dogbot->get_n_sw())=Eigen::MatrixXd::Zero(12, 3*dogbot->get_n_sw());

        //_D_con_0.block(4*4+12,0,12,6)=Eigen::MatrixXd::Zero(12,6);
        _D_con_0.block(4*5+12,6,12,12)=-dogbot->get_mass_matrix().block(6,6,12,12);
        _D_con_0.block(4*5+12,6+12,12,3*4)=dogbot->get_j_st_j_bar_0().transpose();
        //_D_con.block(4*4+12,6+12+3*4,12, 3*dogbot->get_n_sw())=Eigen::MatrixXd::Zero(12, 3*dogbot->get_n_sw());

        //_D_con.block(4*4+2*12,0, 3*dogbot->get_n_sw(), 6)=dogbot->get_j_sw_c_bar();
        //_D_con.block(4*4+2*12,6, 3*dogbot->get_n_sw(), 12)=dogbot->get_j_sw_j_bar();
        //_D_con.block(4*4+2*12,6+12,3*dogbot->get_n_sw(),3*4)=Eigen::MatrixXd::Zero(3*dogbot->get_n_sw(), 3*4);
        //_D_con.block(4*4+2*12,6+12+3*4,3*dogbot->get_n_sw(),3*dogbot->get_n_sw())=Eigen::MatrixXd::Identity(3*dogbot->get_n_sw(),3*dogbot->get_n_sw());
    
        //_D_con.block(4*4+2*12+3*dogbot->get_n_sw(),0, 3*dogbot->get_n_sw(), 6)=-dogbot->get_j_sw_c_bar();
        //_D_con.block(4*4+2*12+3*dogbot->get_n_sw(),6, 3*dogbot->get_n_sw(), 12)=-dogbot->get_j_sw_j_bar();
        //_D_con.block(4*4+2*12+3*dogbot->get_n_sw(),6+12,3*dogbot->get_n_sw(),3*4)=Eigen::MatrixXd::Zero(3*dogbot->get_n_sw(), 3*4);
        //_D_con.block(4*4+2*12+3*dogbot->get_n_sw(),6+12+3*4,3*dogbot->get_n_sw(),3*dogbot->get_n_sw())=Eigen::MatrixXd::Identity(3*dogbot->get_n_sw(),3*dogbot->get_n_sw());
    

        _D_con_0.block(44,6,12,12)=Eigen::MatrixXd::Identity(12,12);
        _D_con_0.block(56,6,12,12)=-Eigen::MatrixXd::Identity(12,12);
    
        //Eigen::Matrix<double,18,1> complete_bias;
        //complete_bias=dogbot->get_c()*dogbot->get_v_c();

    
        //_c_con_0.block(0,0,4*4,1)=Eigen::MatrixXd::Zero(4*4,1);

        //come facevo prima
        //_c_con_0.block(16,0,12,1)=_tau_max - dogbot->get_c2_bar().block(0,6,12,12)*dogbot->get_v_c().block(6,0,12,1);
        //_c_con_0.block(16+12,0,12,1)=-_tau_min + dogbot->get_c2_bar().block(0,6,12,12)*dogbot->get_v_c().block(6,0,12,1);

        ///////////////aggiunta//////////////////////

        for(int i=0; i<4; i++){
		    _c_con_0.block(4+i*5,0,1,1)<<-20;
	    }

        /////////////////////////////////////////////

        //or anon uso più la matrice C che si presume sia stata calcolata in maniera errata
        _c_con_0.block(20,0,12,1)=_tau_max - dogbot->get_bias_com().block(6,0,12,1);
        _c_con_0.block(20+12,0,12,1)=-_tau_min + dogbot->get_bias_com().block(6,0,12,1);

        //_c_con_0.block(4*dogbot->get_n_sw()+2*12,0,3*0,1)=dogbot->_x_sw_cmd_dot_dot - dogbot->_j_sw_dot;
        //_c_con_0.block(4*dogbot->get_n_sw()+2*12+3*0,0,3*0,1)=-dogbot->_x_sw_cmd_dot_dot + dogbot->_j_sw_dot;
        
        Eigen::Matrix<double,12,1> qmin;
        Eigen::Matrix<double,12,1> qmax;
        
        
        double deltat=0.01;
        Eigen::Matrix<double,12,1> eigenq=dogbot->get_joint_pos();
	    Eigen::Matrix<double,12,1> eigendq=dogbot->get_v_c().block(6,0,12,1);
	    qmin<<-1.75 , -1.75,-1.75,-1.75,-1.58, -2.62,-3.15, -0.02,   -1.58, -2.62, -3.15, -0.02;
        qmax<<1.75, 1.75, 1.75, 1.75, 3.15, 0.02, 1.58, 2.62,  3.15, 0.02, 1.58, 2.62;
	    Eigen::Matrix<double,12,1> ddqmin=(2/pow(deltat,2))*(qmin-eigenq-deltat*eigendq);
	    Eigen::Matrix<double,12,1> ddqmax=(2/pow(deltat,2))*(qmax-eigenq-deltat*eigendq);

        _c_con_0.block(44,0,12,1)=ddqmax;
        _c_con_0.block(56,0,12,1)=-ddqmin;
    
        for(int i=_A_con_0.rows();i<_A_con_0.rows() + _D_con_0.rows();i++){
            for(int j=0; j<_A_con_0.cols(); j++){
                A_b_con(i,j)=_D_con_0(i-_A_con_0.rows(),j);
            }
        }
    
        //insert scalar coefficients for A/b constraint
        for(int i=0;i<_A_con_0.rows(); i++){
            A_b_con(i,_A_con_0.cols())=_b_con_0(i,0);
        }
      
        //insert scalar coefficients for C/d constraint
        for(int i=_A_con_0.rows();i<_A_con_0.rows() + _D_con_0.rows(); i++){
            A_b_con(i,_A_con_0.cols())=_c_con_0(i-_A_con_0.rows(),0);
        }
    
    
        ////////////////////////////////////////////////////////////////////////////////////

        
        ct.setlength(_A_con_0.rows() + _D_con_0.rows());
    
        for(int i=0; i<_A_con_0.rows(); i++){
            ct(i)=0.0;
        }
    
        for(int i=_A_con_0.rows(); i<_A_con_0.rows() + _D_con_0.rows(); i++){
            ct(i)=-1.0;
        }

        //std::cout<<"fine preparazione: "<<ros::Time::now().toSec()<<endl;

        alglib::real_1d_array x;
        alglib::minqpstate state;
        alglib::minqpreport rep;
        
        //alglib::real_1d_array s;

        //std::cout<<"dopo la dichiarazione delle variabili alglib: "<<ros::Time::now().toSec()<<endl;

    
        // create solver, set quadratic/linear terms
        

        //std::cout<<"prima delle istruzioni alglib: "<<ros::Time::now().toSec()<<endl;

        //double start_opti, end_opti;
            
        alglib::minqpcreate(30, state);
        alglib::minqpsetquadraticterm(state, A);
        alglib::minqpsetlinearterm(state, b);
        alglib::minqpsetlc(state, A_b_con, ct);    
        alglib::minqpsetscaleautodiag(state); 

        //start_opti=(ros::Time::now()).toSec();  
            
        alglib::minqpsetalgodenseaul(state, 1.0e-2, 1.0e+4, 5); 

        //end_opti=(ros::Time::now()).toSec();  

        //if(end_opti - start_opti >= 0.006){
        //    dogbot->set_overlap(true);
        //    //dogbot->set_diff(end_opti-start_opti);
        //}

        //while((ros::Time::now()).toSec()-start_opti<=0.005){
        //    alglib::minqpsetalgodenseaul(state, 1.0e-2, 1.0e+4, 5);
        //}

        alglib::minqpoptimize(state);
        alglib::minqpresults(state, x, rep);
        //printf("%s\n", x.tostring(1).c_str()); 

        //std::cout<<"dopo le istruzioni alglib"<<ros::Time::now().toSec()<<endl;

        q_dot_dot_des_temp= Eigen::MatrixXd::Zero(12,1);        
        f_gr_star_temp= Eigen::MatrixXd::Zero(12,1);
    
    
        for(int i=6; i<18; i++){
            q_dot_dot_des_temp(i-6,0)=x[i];
        }
    
        for(int i=18;i<30;i++){
            f_gr_star_temp(i-18,0)=x[i];
        }


        dogbot->set_q_dot_dot_des(q_dot_dot_des_temp);

        dogbot->set_f_gr_star(f_gr_star_temp);
    }

    if(flag==1 || flag==2){ 
        
        _A_1= _sigma_1.transpose()*dogbot->get_j_st_c_bar_1()*weight_1*dogbot->get_j_st_c_bar_1().transpose()*_sigma_1 + weight_2;
        
        //_A_1= _sigma_1.transpose()*dogbot->get_j_st_c_bar_1()*dogbot->get_j_st_c_bar_1().transpose()*_sigma_1 + Eigen::MatrixXd::Identity(24,24);
        
        //_b1_1= dogbot->get_f_st_hat_1().transpose()*dogbot->get_j_st_c_bar_1()*dogbot->get_j_st_c_bar_1().transpose()*_sigma_1 - dogbot->get_w_com_des().transpose()*dogbot->get_j_st_c_bar_1().transpose()*_sigma_1;

        //STIMATORE
    
        _b2_1= -_sigma_1.transpose()*dogbot->get_j_st_c_bar_1()*weight_1.transpose()*dogbot->get_w_com_des();// + _sigma_1.transpose()*dogbot->get_j_st_c_bar_1()*weight_1.transpose()*dogbot->get_j_st_c_bar_1().transpose()*dogbot->get_f_st_hat_1();
    
        ////_b1_1=-dogbot->get_w_com_des().transpose()*weight_1*dogbot->get_j_st_c_bar_1().transpose()*_sigma_1; //+  dogbot->get_f_st_hat_1().transpose()*dogbot->get_j_st_c_bar_1()*weight_1*dogbot->get_j_st_c_bar_1().transpose()*_sigma_1;
        
        //cost function
        
        A.setlength(_A_1.rows(),_A_1.cols());
        
    
        for(int i=0;i<_A_1.rows();i++){
            for(int j=0; j<_A_1.cols();j++){
                A(i,j)=_A_1(i,j);
            }
        }
        
    
        
        //b.setlength(_b1_1.cols());    
        //for(int i=0; i<_b1_1.cols(); i++){
        //    b(i)=_b2_1(i,0);//_b1_1(0,i);//+_b2_1(i,0);
        //}


        b.setlength(_b2_1.rows());
        for(int i=0; i<_b2_1.rows(); i++){
            b(i)=_b2_1(i,0);//_b1_1(0,i);//+_b2_1(i,0);
        }
      
    
        ///////////////////////////////////////////////////////////////
    
        //////////////////////////////////////////////////////////////
        //CONSTRAINTS('con' stands for the matrix which are passed to alglib script)
    
        _A_con_1.block(0,0,6,6)=dogbot->get_mass_matrix().block(0,0,6,6);
        //_A_con_1.block(0,6,6,12)=Eigen::MatrixXd::Zero(6,12);
        _A_con_1.block(0,18,6,3*2)=-dogbot->get_j_st_c_bar_1().transpose();
        //_A_con_1.block(0,6+12+3*2,6,3*2)=Eigen::MatrixXd::Zero(6,3*2);
    
        _A_con_1.block(6,0,3*2,6)=dogbot->get_j_st_c_bar_1();
        _A_con_1.block(6,6,3*2,12)=dogbot->get_j_st_j_bar_1();
        //_A_con_1.block(6,6+12,3*2,3*2)=Eigen::MatrixXd::Zero(3*2, 3*2);
        //_A_con_1.block(6,6+12+3*2,3*2,3*2)=Eigen::MatrixXd::Zero(3*2,3*2);
    
        
    
        //Eigen::Matrix<double,18,1> bias_com;
        //bias_com=dogbot->get_t_bar().transpose().inverse()*dogbot->get_h()+dogbot->get_t_bar().transpose().inverse()*dogbot->get_mass_matrix()*dogbot->get_t_inv_der()*dogbot->get_v_c(); 
        

        //Eigen::Matrix<double,12,18> JacCOM_lin;

        //JacCOM_lin=_B*dogbot->get_j_bar();

        //STIMATORE

        //Eigen::Matrix<double,6,1> fext_st;
        //fext_st.block(0,0,3,1)=w3.block(dogbot->get_stl1(),0,3,1);
        //fext_st.block(3,0,3,1)=w3.block(dogbot->get_stl2(),0,3,1);
        //Eigen::Matrix<double, 6, 6> Jstcom= Eigen::Matrix<double,6,6>::Zero();
        //Jstcom.block(0,0,3,6)= _JacCOM_lin.block(dogbot->get_stl1(),0,3,6);
	    //Jstcom.block(3,0,3,6)= _JacCOM_lin.block(dogbot->get_stl2(),0,3,6);
	    
        _b_con_1.block(0,0,6,1)=-dogbot->get_bias_com().block(0,0,6,1);// + Jstcom.transpose()*fext_st;

        

        //_b_con_1.block(0,0,6,1)=-dogbot->get_mass_matrix()(0,0)*g;
    
        _b_con_1.block(6,0,6,1)=-dogbot->get_j_st_dot_1();

        //_b_con_1.block(6,0,6,1)=-dogbot->get_j_st_dot_1().block(0,0,6,6)*dogbot->get_v_c().block(0,0,6,1) - dogbot->get_j_st_dot_1().block(0,6,6,12)*dogbot->get_v_c().block(6,0,12,1);
        
        //put A and b together
        
        A_b_con.setlength(_A_con_1.rows() + _D_con_1.rows(), _A_con_1.cols()+1);
    
        for(int i=0;i<_A_con_1.rows();i++){
            for(int j=0; j<_A_con_1.cols();j++){
                A_b_con(i,j)=_A_con_1(i,j);
            }
        }  
    

        //_D_con_1.block(0,0,4*2,6)=Eigen::MatrixXd::Zero(4*2,6);
        //_D_con_1.block(0,6,4*2,12)=Eigen::MatrixXd::Zero(4*2, 12);
        _D_con_1.block(0,6+12,5*2, 3*2)=_d_fr_1;
        //_D_con_1.block(0,6+12+3*2,4*2,3*2)=Eigen::MatrixXd::Zero(4*2, 3*2);
    
        //_D_con_1.block(4*2,0,12,6)=Eigen::MatrixXd::Zero(12,6);
        _D_con_1.block(5*2,6,12,12)=dogbot->get_mass_matrix().block(6,6,12,12);
        _D_con_1.block(5*2,6+12,12,3*2)=-dogbot->get_j_st_j_bar_1().transpose();
        //_D_con_1.block(4*2,6+12+3*2,12, 3*2)=Eigen::MatrixXd::Zero(12, 3*2);
    
        //_D_con_1.block(4*2+12,0,12,6)=Eigen::MatrixXd::Zero(12,6);
        _D_con_1.block(5*2+12,6,12,12)=-dogbot->get_mass_matrix().block(6,6,12,12);
        _D_con_1.block(5*2+12,6+12,12,3*2)=dogbot->get_j_st_j_bar_1().transpose();
        //_D_con_1.block(4*2+12,6+12+3*2,12, 3*2)=Eigen::MatrixXd::Zero(12, 3*2);
    
        _D_con_1.block(5*2+2*12,0, 3*2, 6)=dogbot->get_j_sw_c_bar();
        _D_con_1.block(5*2+2*12,6, 3*2, 12)=dogbot->get_j_sw_j_bar();
        //_D_con_1.block(4*2+2*12,6+12,3*2,3*2)=Eigen::MatrixXd::Zero(3*2, 3*2);
        _D_con_1.block(5*2+2*12,6+12+3*2,3*2,3*2)=Eigen::MatrixXd::Identity(3*2,3*2);
    
        _D_con_1.block(5*2+2*12+3*2,0, 3*2, 6)=-dogbot->get_j_sw_c_bar();
        _D_con_1.block(5*2+2*12+3*2,6, 3*2, 12)=-dogbot->get_j_sw_j_bar();
        //_D_con_1.block(4*2+2*12+3*2,6+12,3*2,3*2)=Eigen::MatrixXd::Zero(3*2, 3*2);
        _D_con_1.block(5*2+2*12+3*2,6+12+3*2,3*2,3*2)=-Eigen::MatrixXd::Identity(3*2,3*2);   

        _D_con_1.block(46,6,12,12)=Eigen::MatrixXd::Identity(12,12);
        _D_con_1.block(58,6,12,12)=-Eigen::MatrixXd::Identity(12,12);

        //AGGIUNTA (non serve)

        
        //for(int i=0; i<2; i++){
		//    _c_con_1.block(4+i*5,0,1,1)<<-20;
	    //}

        //STIMATORE

        //_c_con_1.block(0,0,4*2,1)=Eigen::MatrixXd::Zero(4*2,1);
        _c_con_1.block(5*2,0,12,1)=_tau_max - dogbot->get_bias_com().block(6,0,12,1);
        _c_con_1.block(5*2+12,0,12,1)=-_tau_min + dogbot->get_bias_com().block(6,0,12,1);
        _c_con_1.block(5*2+2*12,0,3*2,1)=_x_sw_cmd_dot_dot - dogbot->get_j_sw_dot();// -_fext_lambda;  //.block(0,0,6,6)*dogbot->get_v_c().block(0,0,6,1) - dogbot->get_j_sw_dot().block(0,6,6,12)*dogbot->get_v_c().block(6,0,12,1);
        _c_con_1.block(5*2+2*12+3*2,0,3*2,1)=-_x_sw_cmd_dot_dot + dogbot->get_j_sw_dot();// +_fext_lambda; //.block(0,0,6,6)*dogbot->get_v_c().block(0,0,6,1) + dogbot->get_j_sw_dot().block(0,6,6,12)*dogbot->get_v_c().block(6,0,12,1);
    
        Eigen::Matrix<double,12,1> qmin;
        Eigen::Matrix<double,12,1> qmax;
        
        double deltat=0.01;
        Eigen::Matrix<double,12,1> eigenq=dogbot->get_joint_pos();
	    Eigen::Matrix<double,12,1> eigendq=dogbot->get_v_c().block(6,0,12,1);
	    qmin<<-1.75 , -1.75,-1.75,-1.75,-1.58, -2.62,-3.15, -0.02,   -1.58, -2.62, -3.15, -0.02;
        qmax<<1.75, 1.75, 1.75, 1.75, 3.15, 0.02, 1.58, 2.62,  3.15, 0.02, 1.58, 2.62;
	    Eigen::Matrix<double,12,1> ddqmin=(2/pow(deltat,2))*(qmin-eigenq-deltat*eigendq);
	    Eigen::Matrix<double,12,1> ddqmax=(2/pow(deltat,2))*(qmax-eigenq-deltat*eigendq);

        _c_con_1.block(46,0,12,1)=ddqmax;
        _c_con_1.block(58,0,12,1)=-ddqmin;
        
        
        
        for(int i=_A_con_1.rows();i<_A_con_1.rows() + _D_con_1.rows();i++){
            for(int j=0; j<_A_con_1.cols();j++){
                A_b_con(i,j)=_D_con_1(i-_A_con_1.rows(),j);
            }
        }
    
        //insert scalar coefficients for A/b constraint
        for(int i=0;i<_A_con_1.rows(); i++){
            A_b_con(i,_A_con_1.cols())=_b_con_1(i,0);
        }
      
        //insert scalar coefficients for C/d constraint
        for(int i=_A_con_1.rows();i<_A_con_1.rows() + _D_con_1.rows(); i++){
            A_b_con(i,_A_con_1.cols())=_c_con_1(i-_A_con_1.rows(),0);
        }
    
    
        ////////////////////////////////////////////////////////////////////////////////////
       
        
        
        
        
        ct.setlength(_A_con_1.rows() + _D_con_1.rows());
    
        for(int i=0; i<_A_con_1.rows(); i++){
            ct(i)=0.0;
        }
    
    
    
        for(int i=_A_con_1.rows(); i<_A_con_1.rows() + _D_con_1.rows(); i++){
            ct(i)=-1.0;
        }

        alglib::real_1d_array x;
        alglib::minqpstate state;
        alglib::minqpreport rep;

        //double start_opti, end_opti;


        // create solver, set quadratic/linear terms
        alglib::minqpcreate(30, state);
        alglib::minqpsetquadraticterm(state, A);
        alglib::minqpsetlinearterm(state, b);
        alglib::minqpsetlc(state, A_b_con, ct);
        alglib::minqpsetscaleautodiag(state);
    
      
        //minqpsetalgodenseaul(state, 1.0e-5, 1.0e+4, 10);


        //start_opti=(ros::Time::now()).toSec();  
            
        alglib::minqpsetalgodenseaul(state, 1.0e-2, 1.0e+4, 5); 

        //end_opti=(ros::Time::now()).toSec();  

        //if(end_opti - start_opti >= 0.004){
        //    dogbot->set_overlap(true);
        //    dogbot->set_diff(end_opti-start_opti);
        //}

        alglib::minqpoptimize(state);
        alglib::minqpresults(state, x, rep);
        //printf("%s\n", x.tostring(1).c_str());        
    
    
        Eigen::Matrix<double,12,1> q_dot_dot_des_temp;
        q_dot_dot_des_temp= Eigen::MatrixXd::Zero(12,1);
    
        for(int i=6; i<18; i++){
            q_dot_dot_des_temp(i-6,0)=x[i];
        }

        dogbot->set_q_dot_dot_des(q_dot_dot_des_temp);

        //prendo solo front right e back left (piedi a contatto con il suolo)
        if(flag==1){
            Eigen::Matrix<double,12,1> f_gr_star_temp;
            f_gr_star_temp= Eigen::MatrixXd::Zero(12,1);

            f_gr_star_temp(0,0)=x[18];
            f_gr_star_temp(1,0)=x[19];
            f_gr_star_temp(2,0)=x[20];     
        
            f_gr_star_temp(9,0)=x[21];
            f_gr_star_temp(10,0)=x[22];
            f_gr_star_temp(11,0)=x[23];

            dogbot->set_f_gr_star(f_gr_star_temp);
            
        }

        if(flag==2){
            Eigen::Matrix<double,12,1> f_gr_star_temp;
            f_gr_star_temp= Eigen::MatrixXd::Zero(12,1); 

            f_gr_star_temp(3,0)=x[18];
            f_gr_star_temp(4,0)=x[19];
            f_gr_star_temp(5,0)=x[20];     
        
            f_gr_star_temp(6,0)=x[21];
            f_gr_star_temp(7,0)=x[22];
            f_gr_star_temp(8,0)=x[23];

            dogbot->set_f_gr_star(f_gr_star_temp);
            
        }
    
        
    }
   
    
}    




void OPTIMAL::initialization(){

    //_k_d=Eigen::MatrixXd::Zero(6,6);

    //_k_p=Eigen::MatrixXd::Zero(6,6);

    //_d_fr_0=Eigen::MatrixXd::Zero(16,12);
    _d_fr_0=Eigen::MatrixXd::Zero(20,12);

    //_d_fr_1=Eigen::MatrixXd::Zero(8,6);
    _d_fr_1=Eigen::MatrixXd::Zero(10,6);

    _sigma_0.block(0,0,12,18)=Eigen::MatrixXd::Zero(12,18);
    _sigma_0.block(0,18,12,12)=Eigen::MatrixXd::Identity(12,12);

    _sigma_1.block(0,0,6,30)=Eigen::MatrixXd::Zero(6,30);
    _sigma_1.block(0,18,6,6)=Eigen::MatrixXd::Identity(6,6);

    _A_0=Eigen::MatrixXd::Zero(30,30);

    _A_1=Eigen::MatrixXd::Zero(30,30);

    _b1_0=Eigen::MatrixXd::Zero(1,30);

    _b1_1=Eigen::MatrixXd::Zero(1,30);

    _b2_0=Eigen::MatrixXd::Zero(30,1);

    _b2_1=Eigen::MatrixXd::Zero(30,1);

    _A_con_0=Eigen::MatrixXd::Zero(18,30);
    

    _A_con_1=Eigen::MatrixXd::Zero(12,30);

    _b_con_0=Eigen::MatrixXd::Zero(18,1);

    _b_con_1=Eigen::MatrixXd::Zero(12,1);

    //_D_con_0=Eigen::MatrixXd::Zero(40,30);
    _D_con_0=Eigen::MatrixXd::Zero(68,30);

    //_D_con_1=Eigen::MatrixXd::Zero(44,30);
    _D_con_1=Eigen::MatrixXd::Zero(70,30);

    //_c_con_0=Eigen::MatrixXd::Zero(40,1);
    _c_con_0=Eigen::MatrixXd::Zero(68,1);

    //_c_con_1=Eigen::MatrixXd::Zero(44,1);
    _c_con_1=Eigen::MatrixXd::Zero(70,1);

    for(int i=0; i<12; i++){
        _tau_max(i,0)=60.0;
        _tau_min(i,0)=-60.0;
    }



    //_k_d_sw=Eigen::MatrixXd::Zero(6,6);

    //_k_p_sw=Eigen::MatrixXd::Zero(6,6);

    //_n_c=Eigen::MatrixXd::Zero(18,18);

    //_m_c=Eigen::MatrixXd::Zero(18,18);

    _x_sw_cmd_dot_dot=Eigen::MatrixXd::Zero(6,1);

    _t1 << 1, 0, 0;
    _t2 << 0, 1, 0;
    _n  << 0, 0, 1;

    _B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
         Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
    	 Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
    	 Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);

    _JacCOM_lin=Eigen::MatrixXd::Zero(12,18);

    //_Si=Eigen::MatrixXd::Zero(18,18);
    //_Jst=Eigen::MatrixXd::Zero(6,18);
    //_P=Eigen::MatrixXd::Zero(18,18);
    //_fext_lambda=Eigen::MatrixXd::Zero(6,1);

    g=Eigen::MatrixXd::Zero(6,1);
    g(2,0)=9.81;

    weight_1=50*Eigen::MatrixXd::Identity(6,6); //togliere weight

    weight_2=Eigen::MatrixXd::Identity(30,30);
    weight_2.block(24,24,6,6)=1000*Eigen::MatrixXd::Identity(6,6);

}


