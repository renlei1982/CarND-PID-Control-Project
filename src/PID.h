#include <uWS/uWS.h>
#ifndef PID_H
#define PID_H

#define MIN_STEPS 0

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;


  /*
  *Twiddle factors
  */
  double dpp = 0.05;
  double dpi = 0.0001;
  double dpd = 0.5;


  /*
  *error and best error
  */
  double total_error;
  double err;
  double best_err = 1e9;


  /*
  *create a rotate number for Kp(0), Ki(1), Kd(2)
  */
  int rotor = 0;


  /*
  *create a single switch
  */
  bool switch_I = false;
  bool switch_II = false;


  /*
  *step_num to count the steps
  */
  int step_num = 0;



  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * twiddle process.
  */
  void twiddle();

  void single_twiddle(double &K, double &dp);

  /*
  * A function to restart the simulator
  */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);

};

#endif /* PID_H */
