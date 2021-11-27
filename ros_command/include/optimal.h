#ifndef OPTIMAL_H
#define OPTIMAL_H

#include "for_dyn.h"

// Eigen headers
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR> 

#include "alglib/optimization.h"

#include <tf/transform_broadcaster.h>

class QUADRUPED;

class OPTIMAL{
    public:


    OPTIMAL(QUADRUPED &quadruped);

    void compute_w_com_des();

    void compute_d_fr(int flag);

    void compute_optimization(int flag);

    //void compute_estimation(int flag);

    void initialization();


    private:

    QUADRUPED*  dogbot;

    Eigen::Matrix<double, 6, 6> _k_d;

    Eigen::Matrix<double, 6, 6> _k_p;

    //Eigen::Matrix<double,16,12> _d_fr_0;
    Eigen::Matrix<double,20,12> _d_fr_0;

    //Eigen::Matrix<double,8,6> _d_fr_1;
    Eigen::Matrix<double,10,6> _d_fr_1;

    Eigen::Matrix<double,12,30> _sigma_0;
    Eigen::Matrix<double,6,30> _sigma_1;


    Eigen::Matrix<double,30,30> _A_0;
    Eigen::Matrix<double,30,30> _A_1;

    Eigen::Matrix<double,1,30> _b1_0;
    Eigen::Matrix<double,1,30> _b1_1;

    Eigen::Matrix<double,30,1> _b2_0;
    Eigen::Matrix<double,30,1> _b2_1;

    Eigen::Matrix<double,18,30> _A_con_0;
    Eigen::Matrix<double,12,30> _A_con_1;

    Eigen::Matrix<double,18,1> _b_con_0;
    Eigen::Matrix<double,12,1> _b_con_1;

    //Eigen::Matrix<double,40,30> _D_con_0;
    Eigen::Matrix<double,68,30> _D_con_0;

    
    //Eigen::Matrix<double,44,30> _D_con_1;
    Eigen::Matrix<double,70,30> _D_con_1;

    //Eigen::Matrix<double,40,1> _c_con_0;
    Eigen::Matrix<double,68,1> _c_con_0;


    //Eigen::Matrix<double,44,1> _c_con_1;
    Eigen::Matrix<double,70,1> _c_con_1;


    Eigen::Matrix<double,12,1> _tau_max;

    Eigen::Matrix<double,12,1> _tau_min;

    

    Eigen::Matrix<double,6,6> _k_d_sw;

    Eigen::Matrix<double,6,6> _k_p_sw;

    Eigen::Matrix<double,18,18> _n_c;

    Eigen::Matrix<double,18,18> _m_c;

    Eigen::Matrix<double,6,1> _x_sw_cmd_dot_dot;

    Eigen::Matrix<double,3,1> _t1; 
    Eigen::Matrix<double,3,1> _t2; 
    Eigen::Matrix<double,3,1> _n ; 

    

    Eigen::Matrix<double,12,24> _B;

    Eigen::Matrix<double,12,18> _JacCOM_lin;

    //Eigen::Matrix<double,18,18> _Si;
    //Eigen::Matrix<double, 6, 18> _Jst;
    //Eigen::Matrix<double,18,18> _P;
    //Eigen::Matrix<double,6,1> _fext_lambda;

    Eigen::Matrix<double,6,1> g;

    Eigen::Matrix<double,6,6> weight_1;

    //per velocizzare alglib dichiaro una sola volta le seguenti quantità

    alglib::real_2d_array A;

    alglib::real_1d_array b;

    alglib::real_2d_array A_b_con;

    alglib::integer_1d_array ct;

    Eigen::Matrix<double,12,1> q_dot_dot_des_temp;

    Eigen::Matrix<double,12,1> f_gr_star_temp;

    Eigen::Matrix<double,30,30> weight_2;

    






    
    
};

#endif