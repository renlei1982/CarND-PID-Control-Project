#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {

    p_error = 0;
    d_error = 0;
    i_error = 0;
}

PID::~PID() {}

void PID::Init(double p_, double i_, double d_) {
    Kp = p_;
    Ki = i_;
    Kd = d_;
}

void PID::UpdateError(double cte) {

    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    if(step_num > MIN_STEPS) {
        total_error += pow(cte,2);
        err = total_error/(step_num - MIN_STEPS);
    }
    step_num++;

}

double PID::TotalError() {

    return err;
}


// The twiddle function
void PID::twiddle(){

    step_num = 0;
    p_error = 0;
    i_error = 0;
    d_error = 0;
    total_error = 0;

    if (err < best_err && (switch_I == true || switch_II == true)){
        best_err = err;
        std::cout << "The best error is " << best_err << std::endl;
        switch(rotor){
            case 0: {dpp *= 1.1;
                     std::cout << "The dpp * 1.1" << std::endl;
                     break;
            }
            case 1: {dpi *= 1.1;
                     std::cout << "The dpi * 1.1" << std::endl;
                     break;
            }
            case 2: {dpd *= 1.1;
                     std::cout << "The dpd * 1.1" << std::endl;
                     break;
            }
        }
        std::cout << "Kp " << Kp << " Ki " << Ki << " Kd " << Kd <<std::endl;
        std::cout << "dpp " << dpp << " dpi " << dpi << " dpd " << dpd <<std::endl;


        rotor = (rotor + 1)%3;
        switch_I = false;
        switch_II = false;
    }
    else if(switch_I == false && switch_II == false){
        rotor = (rotor + 1)%3;
        std::cout << "best error unchanged, rotate to the next one" << std::endl;
        std::cout << "Kp " << Kp << " Ki " << Ki << " Kd " << Kd <<std::endl;
        std::cout << "dpp " << dpp << " dpi " << dpi << " dpd " << dpd <<std::endl;

    }

    //std::cout<< "rotor " << rotor << " switch I " << switch_I << " switch_II " << switch_II << std::endl;


    switch(rotor){
        case 0: {single_twiddle(Kp, dpp);
                 std::cout << "twiddle Kp" << std::endl;
                 break;
        }
        case 1: {single_twiddle(Ki, dpi);
                 std::cout << "twiddle Ki" << std::endl;
                 break;
        }
        case 2: {single_twiddle(Kd, dpd);
                 std::cout << "twiddle Kd" << std::endl;
                 break;
        }
    }
}

// Twiddle single component
void PID::single_twiddle(double &K, double &dp){

    if (switch_I == false && switch_II == false){
        K += dp;
        switch_I = true;
    }
    else{
        if (switch_I == true && switch_II ==false){
            K -= dp*2;
            switch_II = true;
        }
        else{
            K += dp;
            dp *= 0.9;
            switch_I = false;
            switch_II = false;
            }
    }
}


// Function to restart the simulator
void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){

  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
