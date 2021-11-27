#ifndef PLANNING_H
#define PLANNING_H


#include "ros/ros.h"
#include "boost/thread.hpp"
#include <ifopt/ipopt_solver.h>
#include <towr/initialization/gait_generator.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>

#include "for_dyn.h"



class QUADRUPED;


class PLANNING{
    public:

    

    PLANNING(QUADRUPED &quadruped);

    void compute_planning(int flag);

    towr::SplineHolder get_solution();

    towr::NlpFormulation get_formulation();

    double get_start_scan();


    private:

    QUADRUPED*  dogbot;

    towr::SplineHolder _solution;

    towr::NlpFormulation _formulation;

    bool _turn=false;

    

    bool _step_1=true;
    bool _step_2=false;
    bool _step_3=false;
    bool _step_4=false;
    bool _step_5=false;
    bool _step_6=false;
    bool _step_7=false;
    bool _step_8=false;
    bool _step_9=false;
    bool _step_10=false;
    bool _step_11=false;
    bool _step_12=false;
    bool _step_13=false;
    bool _step_14=false;
    bool _step_15=false;
    bool _step_16=false;
    bool _step_17=false;
    bool _step_18=false;
    bool _step_19=false;

    bool _start_scan=false;

    double _x_temp=0.0;
    double _y_temp=0.0;
    double _z_temp=0.0;
    double _yaw_temp=0.0;





};

#endif