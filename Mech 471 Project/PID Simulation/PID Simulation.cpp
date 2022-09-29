
// DC motor + traction model

#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>


using namespace std;

// sign macro function
#define SIGN(a) ( (a) >= 0.0 ? 1.0 : -1.0 )

//Control Related Functions
double PID_selector(double reference_speed, double measured_speed, double slip_reading); //runs the PIDs
double traction_control(double r, double measured); //runs the traction PID
double speed_control(double r, double measured); //runs the speed PID
double slip_ratio_calculator(double wb, double ); //calculates the slip ratio
double volt_to_rads(double ADC_volts); //calculates the speed based on the ADC voltage
void sim_step(double& t, double x[], double u[], double dt, double y[]);


//Importants Constants
const int NS = 5; // number of state variables (order)
const int MS = 2; // number of inputs
const int PS = 5; // number of outputs

const float Vref = 3.3;
const float wmax = 810.0;
const double Rw = 3.2e-2;
const double tol = 1.0e-10;
// wf - front wheel speed	
// note: this model assumes the front wheel has no slip
// -- perfect rolling, v = wf*Rw -> wf = v / Rw	
// *** important formula to calculate slip ratio
// wb back wheel speed
// wm motor speed

double calculate_mu(double r);

double calculate_mu_bw(double r);

int main()
{
	int i;
	double t; // current time (seconds)	
	double dt; // time step (s)
	double tf; // final time (s)	
	double x[NS + 1];  // state vector
	double u[MS + 1]; // input vector u
	double y[PS + 1]; // outputs of interest for plotting
	//double u_PID[MS + 1];
	static double slip_ratio_reading = 0;
	static double speed_reading = 0;
	double slip_ratio_desired = 0.2; // desired / reference output (V)


	// open output text file -- you can view this file with notepad++, etc.
	// the csv extension indicates it's a file with columns separated by
	// commas which can be opened and plotted with Excel or Matlab
	ofstream fout("sim.csv");

	// note you need to close the file in excel before re-writing it
	if (!fout) {
		cout << "\nerror opening output file";
	}

	// use scientific notation to avoid loss of precision
	fout << scientific;

	// 8 digits of precision
	fout.precision(7);

	// label the columns (the % is so matlab plotting m-files ignore this line)
	fout << "%time(s),x1,x2,x3,x4,position,u1,u2,y1,speed_reading,slip_ratio_reading,y4,input_rads,\n";

	dt = 0.001; // time step
	tf = 20.0; // final simulation time (s)

	// first call to sim_step(...) just sets ICs and parameters
	sim_step(t, x, u, dt, y);

	cout << "\nsimulation is starting";

	// simulation loop
	while (t < tf) {

		// save time and state variables and input into a file
		fout << t;
		for (i = 1; i <= NS; i++) fout << "," << x[i];

		// calculate control inputs
		u[2] = 0.0; // disturbance torque Td(t)

		//function for input to test
		double input_rads;
		if (t < 14) input_rads = 200; 
		else input_rads = 400; 
		y[5] = input_rads;

		//voltage output from the PID controller
		u[1] = PID_selector(input_rads, speed_reading, slip_ratio_reading);




		for (i = 1; i <= MS; i++) fout << "," << u[i];

		// Euler simulation step of dt
		// simulate from t to t+dt
		sim_step(t, x, u, dt, y);

		// outputs of interest for plotting
		for (i = 1; i <= PS; i++) {
			// limit slip ratio if too large so plot scale isn't too large
			if (fabs(y[3]) > 10) y[3] = 10 * SIGN(y[3]);
			fout << "," << y[i];
		}
		slip_ratio_reading = y[3];
		speed_reading = y[2];

		// each row represents a given time
		if (t < tf) fout << "\n"; // move on to next row

	} // end while

	// close output file
	fout.close();

	cout << "\nsimulation is complete\n\n";

	return 0;
}


void sim_step(double& t, double x[], double u[], double dt, double y[])
{
	int i;
	static double xd[NS + 1]; // derivative vector at time t	

	// note static variables are used to store variables 
	// that need to be remembered between function calls.

	// 1. set model parameters (m, k, b, etc.)
	static double L, R, kb; // electrical model parameters
	static double J, km, b, fc; // mechanical model parameters
	static int init = 0; // initialization flag

	// new parameters for traction model
	static double m, Rw, g, Q, GR;
	double mu, Ft, Fn, wm, v, r, wb, wf;
	double tol = 1.0e-10;


	if (!init) {

		L = 0.003; ///
		R = 0.141; ///
		// note due to energy conservation kb = km always
		kb = km = 0.00574; /// 
		J = 0.0001; /// this might also include the load inertia (wheel, gears, etc.)
		b = 3.97e-6; ///
		fc = 0.0; /// 0.008 max

		// new parameters for traction model
		m = 1.136; /// (kg) total car mass
		Rw = 3.2e-2; /// (m) 3.2 cm tire radius
		g = 9.8; /// g (m/s^2)
		Q = 0.37; /// rear weight distribution
		GR = 2.5; /// gear ratio

		////////////////////////////////////

		// 2. set initial conditions (IC)
		t = 0.0; // initial time	
		x[1] = 0.0; // initial current, i (A)
		x[2] = 0.0; /// initial velocity of motor, wm (rad/s)
		x[3] = 0.0; // initial angle, theta (rad)	

		// new states for the traction model
		x[4] = 0.0; // initial forward velocity v (m/s)
		x[5] = 0.0; // initial x position (m)

		init = 1; // initialization complete

		return; // first call to sim_step(...) just sets ICs and parameters

	} // end of initialization section

	// 4. calculate the derivative vector xd at time t

	Fn = m * g * Q; /// normal force on back wheels

	// gear equations ////////
	/// wb = 1 / GR * wm
	/// tau_b = GR * tau_m	

	// calculate slip ratio r

	wm = x[2]; /// motor angular velocity, wm (rad/s)
	wb = (1 / GR) * wm; /// back wheel angular velocity, wb (rad/s)

	v = x[4]; // forward velocity v (m/s)

	// checking for / 0 is not needed because of tol
	r = (wb * Rw - v) / (fabs(v) + tol); ///

	// calculate friction coefficient
	mu = calculate_mu(r); ///

	// outputs of interest for plotting

	y[1] = wb; // back wheel velocity (rad/s)

	// calculate front wheel angular velocity wf
	// v = wf * Rw -> wf = v / Rw
	wf = v / Rw;

	y[2] = wf; // front wheel velocity (rad/s) 	

	y[3] = r;

	y[4] = mu;

	// calculate tire force
	Ft = mu * Fn;

	/// tau_b = GR * tau_m -> tau_m = tau_b / GR

	// DC motor equations (modified for tire torque Rw*Ft)
	xd[1] = (-x[1] * R - kb * x[2] + u[1]) / L; // di/dt
	xd[2] = (km * x[1] - b * x[2] - fc * SIGN(x[2]) - (Rw * Ft) / GR - u[2]) / J; /// dw/dt
	xd[3] = x[2]; // dth/dt = w

	// note that combining state variable equation models
	// normally requires exchange / sharing of coupling 
	// terms / variables between both sets of equations
	// -- in this case the tire force Ft

	// algebraic constrants may also occur with 
	// resulting coupling forces (lagrange multipliers, etc.)
	// -- requires differential algebraic equation (DAE) solvers 

	// new state-variable equations for the traction model
	xd[4] = Ft / m; // dv/dt
	xd[5] = x[4]; // dx/dt = v

	// 5. apply Euler's equation, x = x + dx, note x is a vector
	// this part is always the same
	// but calculating xd will normally be different
	for (i = 1; i <= NS; i++) x[i] = x[i] + xd[i] * dt;

	t = t + dt; // increment time

}


// magic formula for tire friction

// F = N*D*sin(C*atan(B*r-E*(B*r?atan(B*r))))

// coefficient table from
// https://www.mathworks.com/help/physmod/sdl/ref/tiremagicformula.html
// https://www.mathworks.com/help/physmod/sdl/ref/tireroadinteractionmagicformula.html

// Surface	B		C		D		E
// Dry		10		1.9		1		0.97
// Wet		12		2.3		0.82	1
// Snow		5		2		0.3		1
// Ice		4		2		0.1		1

double calculate_mu(double r)
{
	double mu;
	double B, C, D, E, rmax;

	//	B=10; C=1.9; D=1; E=0.97; // dry
	B = 12; C = 2.3; D = 0.82; E = 1; // wet	
//	B=5; C=2; D=0.3; E=1; // snow
//	B=4; C=2; D=0.1; E=1; // ice

	// maximum slip ratio for model -- prevents too much extrapolation
	rmax = 10.0;

	// limit range of r
	if (r > rmax)  r = rmax;
	if (r < -rmax) r = -rmax;

	mu = D * sin(C * atan(B * r - E * (B * r - atan(B * r))));

	return mu;
}


double calculate_mu_bw(double r)
// BW model
// similar to Burckhardt model but with added term a2 for dip in curve
{
	double mu;
	double a1, a2, a3, c1, c2, rmax;

	// note a2 is mainly for wet road dip in curve

	// the ai parameters can be readily estimated off-line or on-line
	a1 = 1.63; a2 = -0.9; a3 = -0.1; rmax = 1.0; // wet

	// rmax is the maximum slip ratio for model -- prevents too much 
	// extrapolation from the a3 term

	// normally c1 and c2 should be constant for different road conditions
	c1 = -27;
	c2 = -15;

	// limit range of r
	if (r > rmax)  r = rmax;
	if (r < -rmax) r = -rmax;

	if (r < 0) {
		r = fabs(r);
		mu = -(a1 * (1 - exp(c1 * r)) + a2 * (1 - exp(c2 * r)) + a3 * r);
	}
	else {
		mu = a1 * (1 - exp(c1 * r)) + a2 * (1 - exp(c2 * r)) + a3 * r;
	}

	return mu;
}

double traction_control(double r, double measured) {
	double y = measured;
	double T = 0.001;
	static double e, ei = 0.0, ep = 0.0, ed;
	double u; //output of pid
	//gains
	double kp = 300.0; // P 
	double ki = 80.0; // I 
	double kd = 50.0; // D 

	e = r - y; 

	//based on the dynamixel anti wind up logic 
	double ei_max;
	if (ki > 0.0) {
		ei_max = 0.14 * 12 / ki; // want ki*ei < 0.14*v_batt
	}
	else {
		ei_max = 0.0;
	}
	double z;
	if ((ei > ei_max) && (e > 0)) {
		z = 0; 
	}
	else if ((ei < -ei_max) && (e < 0)) {	
		z = 0; 
	}
	else { 
		z = e; 
	}
	//errors
	ei += z * T;
	ed = (e - ep) / T; // derivative of error
	ep = e;

	//output voltage
	u = kp * e + ki * ei + kd * ed; 

	return u;
}

double speed_control(double r, double measured) {

	double y = measured;
	double T = 0.001;
	static double e, ei = 0.0, ep = 0.0, ed;
	double u; //output of pid

	double kp = 50.0; // P controller
	double ki = 10.0; // I controller
	double kd = 12.0; // D controller


	e = r - y; // controller error
	T = 0.001;

	//anti windup logic as seen in the dynamixel example
	double ei_max;
	if (ki > 0.0) {
		ei_max = 0.14 * 12 / ki; // want ki*ei < 0.14*v_batt
	}
	else {
		ei_max = 0.0;
	}
	double z;
	if ((ei > ei_max) && (e > 0)) {
		z = 0; 
	}
	else if ((ei < -ei_max) && (e < 0)) {	
		z = 0; 
	}
	else { 
		z = e; 
	}
	
	//calculate the errors
	ei += z * T;

	ed = (e - ep) / T; // derivative of error

	ep = e;// save previous values

	u = kp * e + ki * ei + kd * ed; // PID controller Control Variable

	//saturate
	if (u > 12) u = 12;
	if (u < 0)u = 0;
	return u;

}

double PID_selector(double reference_speed, double measured_speed, double slip_reading) {

	double output;


	if (reference_speed * 1.03 < measured_speed) output = traction_control(-0.2, slip_reading); //we need to start braking, r=-0.2

	//is the speed measured 3% lower than the desired?
	else if (reference_speed * 0.97 > measured_speed) output = traction_control(0.2, slip_reading); //we need to accelerate, r=0.2

	//if within the 3% tolerance, control the speed

	else output = speed_control(reference_speed, measured_speed);


	if (reference_speed > 0) { // saturate the volts
		if (output > 12.0) output = 12.0;
		if (output < 0.0) output = 0.0;
	}
	else {
		if (output > 0.0) output = 0.0;
		if (output < -12.0) output = -12.0;
	}


	return output;
}
double slip_ratio_calculator(double wb, double wf) {
	double slip_ratio = (wb- wf) / (fabs(wb) + tol);
	return slip_ratio;
}

double volt_to_rads(double voltage) {

	return ((voltage - Vref) * 0.4 * wmax);


}