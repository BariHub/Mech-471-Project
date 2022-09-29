//evading robot

// in this program, we use multiple functions to try to evade from the attack robot, some of which were developed but not used
//at the end
#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>
static int priority = 0;
using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"
ofstream robo("myrobo.csv");
 double obstacleic[3];
 double deltatheta;
 double obstaclejc[3];
 int radius_obstacle[3];
 int w;
extern robot_system S1;
double pinkx;
double pinky;
double ic_green;
double jc_green;
//prototype of functions used for movement
void turn_CW(int& pwm_R, int& pwm_L);
void turn_CCW(int& pwm_R, int& pwm_L);
void robotforward(int& pw_r, int& pw_l);
void robotreverse(int& pw_r, int& pw_left);

//


//prototype of functions used to detect objects,opponents and our own robot
bool collision_detection(image& a, double& ic_car, double& jc_car, double& ic_enemy, double& jc_enemy);
void red_centroid(image& a, double& mi_red, double& mj_red, double& ic_red, double& jc_red);
void green_centroid(image& a, double& mi_green, double& mj_green, double& ic_green, double& jc_green);
void calculate_position(image& a, double& x, double& y, double& theta);
void calculate_op_position(image& a, double& x, double& y, double& theta);
void pink_centroid(image& a, double& mi_pink, double& mj_pink, double& ic_pink, double& jc_pink);//in this program used to find our cg
void blue_centroid(image& a, double& mi_blue, double& mj_blue, double& ic_blue, double& jc_blue);//since we are robot B
double angle_calc(double& myCGx, double& myCGy, double& op_CGx, double& op_CGy);
void check_borders(image rgb, int& pw_r, int& pw_l);
//

//computation of our robot relative to other objects
void obstacle_hiding(double& desired_x, double& desired_y, double& op_CGx, double& op_CGy, image& rgb,double& my_CGx,double & my_CGy);
void check_borders(image rgb, int& pw_r, int& pw_l);
void obstacle_detection(image& a, image& b, image& label, image& rgb);//used to determine whether an object is an obstacle based on how static it's CG is
int sobel(image& a, image& mag, image& theta, image& rgb);
void collision_detection(image& a);
void Aim_Opponent(int& pwm_R, int& pwm_L, double& myCGx, double& myCGy, double& op_CGx, double& op_CGy, double& theta);//not used in this program
double angle_calc(double& myCGx, double& myCGy, double& op_CGx, double& op_CGy);
void turncircles(double& robot_cx, double robot_cy, int& pw_r, int& pw_l, double& theta);
int hiding_obstacle_selection(double&myCGx,double &myCGy);
bool ObstacleInTheWay(double& myCGx, double& myCGy, double& theta);
bool evasion_initial(double& robot_cx, double& robot_cy, int& pw_r, int& pw_l, double& theta);



int main()
{


	double robot_cx, op_cx;
	double robot_cy, op_cy;
	double robot_theta, op_theta;
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;

	// TODO: it might be better to put this model initialization
	// section in a separate function

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1 = 640;
	height1 = 480;

	// number of obstacles
	N_obs = 2;

	x_obs[1] = 270; // pixels
	y_obs[1] = 270; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 135; // pixels
	y_obs[2] = 135; // pixels
	size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////

	D = 121.0; // distance between front wheels (pixels)

	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;

	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;

	alpha_max = 3.14159 / 2; // max range of laser / gripper (rad)
	int nlabels;
	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;

	cout << "\npress space key to begin program.";
	pause();

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file

	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	// note that the robot opponent is not currently implemented in 
	// the library, but it will be implemented soon.

	activate_simulation(width1, height1, x_obs, y_obs, size_obs, N_obs,
		"robot_B.bmp", "robot_A.bmp", "background.bmp", "obstacle.bmp", D, Lx, Ly,
		Ax, Ay, alpha_max, n_robot);

	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;

	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 0;
	level = 1;
	set_simulation_mode(mode, level);

	// set robot initial position (pixels) and angle (rad)
	x0 = 300;
	y0 = 170;
	theta0 = 3.14;
	set_opponent_position(x0, y0, theta0);

	// set opponent initial position (pixels) and angle (rad)
	x0 = 550;
	y0 = 175;
	theta0 = 3.14159 / 4;
	
	set_robot_position(x0, y0, theta0);
	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l = 1000; // pulse width for left wheel servo (us)
	pw_r = 1000; // pulse width for right wheel servo (us)
	pw_laser = 0; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)

	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)

	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;


	// set initial inputs
	set_inputs(pw_l, pw_r, pw_laser, laser,
		light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	// opponent inputs
	pw_l_o = 1800; // pulse width for left wheel servo (us)
	pw_r_o = 1300; // pulse width for right wheel servo (us)
	pw_laser_o = 0; // pulse width for laser servo (us)
	laser_o = 0; // laser input (0 - off, 1 - fire)

	// manually set opponent inputs for the simulation
	// -- good for testing your program
	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
		opponent_max_speed);

	// regular vision program ////////////////////////////////

	// note that at this point you can write your vision program
	// exactly as before.

	// in addition, you can set the robot inputs to move it around
	// the image and fire the laser.

	image rgb, rgb0;
	int height, width;
	image a, theta, mag, b;
	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	rgb0.type = RGB_IMAGE;
	rgb0.width = width;
	rgb0.height = height;

	a.type = GREY_IMAGE;
	a.width = width;
	a.height = height;
	mag.type = GREY_IMAGE;
	mag.width = width;
	mag.height = height;
	b.type = GREY_IMAGE;
	b.width = width;
	b.height = height;
	image label; // declare a label image (2 bytes/pixel)
	theta.type = GREY_IMAGE;
	theta.width = width;
	theta.height = height;
	label.type = LABEL_IMAGE;
	label.width = width;
	label.height = height;


	// allocate memory for the images
	allocate_image(rgb);
	allocate_image(a);
	allocate_image(rgb0);
	allocate_image(label);
	allocate_image(b);
	allocate_image(mag);
	allocate_image(theta);
	// measure initial clock time
	tc0 = high_resolution_time();

	while (1) {

		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;
		calculate_position(rgb, robot_cx, robot_cy, robot_theta);
		calculate_op_position(rgb, op_cx, op_cy, op_theta);
		obstacle_detection(a, b, label, rgb);
		(robot_cx, robot_cy, op_cx, op_cy);
		
		view_rgb_image(rgb);
		static double xd, yd;
		obstacle_hiding(xd, yd, op_cx, op_cy, rgb, robot_cx, robot_cy);
		
		if (evasion_initial(robot_cx, robot_cy, pw_r, pw_l, robot_theta)) {
			turncircles(robot_cx, robot_cy, pw_r, pw_l,robot_theta);
			

		}
		check_borders(rgb, pw_r, pw_l);
		if (ObstacleInTheWay(robot_cx, robot_cy, robot_theta)) {
			pw_r = 1500;
			pw_l = 1500;

		}
		set_inputs(pw_l, pw_r, pw_laser, laser,
			light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);

		// manually set opponent inputs for the simulation
		// -- good for testing your program
	
		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
			opponent_max_speed);
		collision_detection(rgb);
		view_rgb_image(rgb);

		// don't need to simulate too fast
		Sleep(10); // 100 fps max
	}

	// free the image memory before the program completes
	free_image(rgb);

	deactivate_vision();

	deactivate_simulation();

	cout << "\ndone.\n";

	return 0;
}
void red_centroid(image& a, double& mi_red, double& mj_red, double& ic_red, double& jc_red) {
	int i, j, k;
	double m = 0.0;
	ibyte R, G, B;
	ibyte* p, * p0;
	int height, width, size;
	width = a.width;
	height = a.height;
	size = width * height;
	p0 = a.pdata;
	mi_red = mj_red = 0.0;
	for (j = 0; j < height; j++) { // j coord

		for (i = 0; i < width; i++) { // i coord

			k = i + width * j;
			p = p0 + 3 * k; // pointer to the kth pixel (3 bytes/pixel)

			B = *p;
			G = *(p + 1);
			R = *(p + 2);

			// find blue pixels and calculate their centroid stats
			if ((B < 100) && (R > 220) && (G < 100)) {

				R = 255;
				G = 0;
				B = 0;

				// highlight blue pixels in the image
				*p = B;
				*(p + 1) = G;
				*(p + 2) = R;

				
				m += R;

				
				mi_red += i * 255; // (i moment of mk) = mk * i
				mj_red += j * 255; // (j moment of mk) = mk * j

					

			} // end if

		} // end for i

	} // end for j

	double eps = 1.0e-10; // small constant to protect against / 0
	ic_red = mi_red / (m + eps);
	jc_red = mj_red / (m + eps);
	//cout <<"ic_red: " << ic_red << endl;
	//cout << "jc_red: "<<jc_red << endl;
}

void green_centroid(image& a, double& mi_green, double& mj_green, double& ic_green, double& jc_green) {
	int i, j, k;
	double m = 0.0;
	ibyte R, G, B;
	ibyte* p, * p0;
	int height, width, size;
	width = a.width;
	height = a.height;
	size = width * height;
	p0 = a.pdata;
	mi_green = mj_green = 0.0;
	for (j = 0; j < height; j++) { // j coord

		for (i = 0; i < width; i++) { // i coord

			k = i + width * j;
			p = p0 + 3 * k; // pointer to the kth pixel (3 bytes/pixel)

			B = *p;
			G = *(p + 1);
			R = *(p + 2);

			// find blue pixels and calculate their centroid stats
			if ((B < 150) && (R < 80) && (G > 123)) {


				// to calculate the centroid you need to calculate mk 
				// - the mass of each pixel k

				// mk = volume * density
				// mk = (pixel area) * (blue intensity)
				// mk = (blue intensity) = B
				// since (pixel area) = 1 pixel x 1 pixel = 1

				// calculate total mass m = sum (mk)
				m += G;

				// * note: for more accuracy we can use B
				// before we set it to 255 earlier in the 
				// if statement

				// calculate total moments in the i and j directions
				mi_green += i * G; // (i moment of mk) = mk * i
				mj_green += j * G; // (j moment of mk) = mk * j

				// original version: ic = 203.427 , jc = 346.4

				// moving summations to top so B goes from 0 - 255:
				// ic = 203.338 , jc = 346.215				

			} // end if

		} // end for i

	} // end for j

	double eps = 1.0e-10; // small constant to protect against / 0
	ic_green = mi_green / (m + eps);
	jc_green = mj_green / (m + eps);
	//cout << "ic_green: " << ic_green << endl;
	//cout << "jc_green: " << jc_green << endl;
}


void calculate_op_position(image& a, double& x, double& y, double& theta) {
	ibyte* p;
	double mi_red, mi_green, mj_red, mj_green, ic_red, jc_red;
	red_centroid(a, mi_red, mj_red, ic_red, jc_red);
	draw_point_rgb(a, ic_red, jc_red, 0, 0, 0);
	green_centroid(a, mi_green, mj_green, ic_green, jc_green);
	draw_point_rgb(a, ic_green, jc_green, 0, 0, 0);
	x = (ic_green + ic_red) / 2;
	y = (jc_green + jc_red) / 2;
	theta = atan2(jc_red - jc_green, ic_red - ic_green);
	theta = (((theta - -3.14) * (359 - 0)) / (3.14 - -3.14)) + 0;
	//cout << theta << endl;
	draw_point_rgb(a, x, y, 0, 0, 0);


}


void blue_centroid(image& a, double& mi_blue, double& mj_blue, double& ic_blue, double& jc_blue) {
	int i, j, k;
	double m = 0.0;
	ibyte R, G, B;
	ibyte* p, * p0;
	int height, width, size;
	width = a.width;
	height = a.height;
	size = width * height;
	p0 = a.pdata;
	mi_blue = mj_blue = 0.0;
	for (j = 0; j < height; j++) { // j coord

		for (i = 0; i < width; i++) { // i coord

			k = i + width * j;
			p = p0 + 3 * k; // pointer to the kth pixel (3 bytes/pixel)

			B = *p;
			G = *(p + 1);
			R = *(p + 2);

			// find blue pixels and calculate their centroid stats
			if ((B > 221) && (R < 55) && (G < 170)) {


				// to calculate the centroid you need to calculate mk 
				// - the mass of each pixel k

				// mk = volume * density
				// mk = (pixel area) * (blue intensity)
				// mk = (blue intensity) = B
				// since (pixel area) = 1 pixel x 1 pixel = 1

				// calculate total mass m = sum (mk)
				m += B;

				// * note: for more accuracy we can use B
				// before we set it to 255 earlier in the 
				// if statement

				// calculate total moments in the i and j directions
				mi_blue += i * B; // (i moment of mk) = mk * i
				mj_blue += j * B; // (j moment of mk) = mk * j

				// original version: ic = 203.427 , jc = 346.4

				// moving summations to top so B goes from 0 - 255:
				// ic = 203.338 , jc = 346.215				

			} // end if

		} // end for i

	} // end for j

	double eps = 1.0e-10; // small constant to protect against / 0
	ic_blue = mi_blue / (m + eps);
	jc_blue = mj_blue / (m + eps);
	//cout << "ic_green: " << ic_green << endl;
	//cout << "jc_green: " << jc_green << endl;
}
void pink_centroid(image& a, double& mi_pink, double& mj_pink, double& ic_pink, double& jc_pink) {
	int i, j, k;
	double m = 0.0;
	ibyte R, G, B;
	ibyte* p, * p0;
	int height, width, size;
	width = a.width;
	height = a.height;
	size = width * height;
	p0 = a.pdata;
	mi_pink = mj_pink = 0.0;
	for (j = 0; j < height; j++) { // j coord

		for (i = 0; i < width; i++) { // i coord

			k = i + width * j;
			p = p0 + 3 * k; // pointer to the kth pixel (3 bytes/pixel)

			B = *p;
			G = *(p + 1);
			R = *(p + 2);

			// find blue pixels and calculate their centroid stats
			if ((B < 132) && (R > 245) && (G > 178)) {

				// to calculate the centroid you need to calculate mk 
				// - the mass of each pixel k

				// mk = volume * density
				// mk = (pixel area) * (blue intensity)
				// mk = (blue intensity) = B
				// since (pixel area) = 1 pixel x 1 pixel = 1

				// calculate total mass m = sum (mk)
				m += R;

				// * note: for more accuracy we can use B
				// before we set it to 255 earlier in the 
				// if statement

				// calculate total moments in the i and j directions
				mi_pink += i * R; // (i moment of mk) = mk * i
				mj_pink += j * R; // (j moment of mk) = mk * j

				// original version: ic = 203.427 , jc = 346.4

				// moving summations to top so B goes from 0 - 255:
				// ic = 203.338 , jc = 346.215				

			} // end if

		} // end for i

	} // end for j

	double eps = 1.0e-10; // small constant to protect against / 0
	ic_pink = mi_pink / (m + eps);
	pinkx = ic_pink;
	jc_pink = mj_pink / (m + eps);
	pinky = jc_pink;
	//cout << "ic_pink: " << pinkx << endl;
	//cout << "jc_green: " << jc_green << endl;
}
void calculate_position(image& a, double& x, double& y, double& theta) {
	ibyte* p;
	double mi_blue, mi_pink, mj_blue, mj_pink, ic_blue, ic_pink, jc_blue, jc_pink;
	pink_centroid(a, mi_pink, mj_pink, ic_pink, jc_pink);
	draw_point_rgb(a, ic_pink, jc_pink, 0, 0, 0);
	blue_centroid(a, mi_blue, mj_blue, ic_blue, jc_blue);
	draw_point_rgb(a, ic_blue, jc_blue, 0, 0, 0);
	x = (ic_blue + ic_pink) / 2;
	y = (jc_blue + jc_pink) / 2;
	theta = atan2(jc_pink - jc_blue, ic_pink - ic_blue);
	theta = (((theta - -3.14) * (359 - 0)) / (3.14 - -3.14)) + 0;
	//cout << theta << endl;
	//draw_point_rgb(a, x, y, 0, 255, 0);

}

void obstacle_detection(image& a, image& b, image& label, image& rgb) {
	static int firstpass = 0;
	
	static double ic[20];
	static double jc[20];
	static bool yellow[307500];
	double ictemp[20];
	double jctemp[20];
	int nlabels;
	ibyte* p;
	p = rgb.pdata;
	int width = rgb.width;
	int height = rgb.height;
	int size;
	size = height * width;
	i2byte* pl;
	pl = (i2byte*)label.pdata;
	copy(rgb, a);
	scale(a, b);
	copy(b, a);

	lowpass_filter(a, b);
	copy(b, a);
	threshold(a, b, 70);
	copy(b, a);
	invert(a, b);
	copy(b, a);
	dialate(a, b);
	copy(b, a);
	dialate(a, b);
	copy(b, a);
	dialate(a, b);
	copy(b, a);
	//pause();
	label_image(a, label, nlabels);

	if (firstpass == 0) {
		label_image(a, label, nlabels);
		for (int i = 1; i <= nlabels; i++) {
			centroid(a, label, i, ic[i], jc[i]);
			draw_point_rgb(rgb, ic[i], jc[i], 255, 255, 255);
			view_rgb_image(rgb);
			//pause();

		}
		firstpass++;
	}
	else if (firstpass == 1) {
		//cout << "first pass" << endl;
		for (int j = 1; j < nlabels; j++) {
			//cout << j << endl;
			centroid(a, label, j, ictemp[j], jctemp[j]);
			//draw_point_rgb(rgb, ictemp[j], jctemp[j], 255, 255, 255);
			view_rgb_image(rgb);
			//pause();
			for (int i = 1; i < nlabels; i++) {
				if ((ictemp[j] > (ic[i] - 2)) && (ictemp[j] < (ic[i] + 2)) && (jctemp[j] < (jc[i] + 2)) && (jctemp[j] > (ic[i] - 2)))
				{
					// obstacles,save each obnstacle ic and jc and label=1
					
					obstacleic[w] = ic[i];
					obstaclejc[w] = jc[i];
					w++;

					for (int k = 0; k < size; k++)
						if (pl[k] == j) {
							p[k * 3] = 0;
						p[k * 3 + 1] = 255;
							p[k * 3 + 2] = 255;
							yellow[k] = true;

						}

					//view_rgb_image(rgb);
					//pause();



				}


			}
		}

		firstpass++;
	}

	else for (int k = 0; k < size; k++)
		if (yellow[k] == true) {
			p[k * 3] = 0;
			p[k * 3 + 1] = 255;
			p[k * 3 + 2] = 255;

		}
}

void Aim_Opponent(int& pwm_R, int& pwm_L, double& myCGx, double& myCGy, double& op_CGx, double& op_CGy, double& theta) {
	if (abs(abs(angle_calc(myCGx, myCGy, op_CGx, op_CGy)) - theta) > 10) {
		//if (abs(abs(angle_calc(myCGx, myCGy, op_CGx, op_CGy)) - theta) < 180)
		turn_CW(pwm_R, pwm_L);
		//else if (abs(abs(angle_calc(myCGx, myCGy, op_CGx, op_CGy)) - theta) >240) turn_CCW(pwm_R, pwm_L);

	}
	else
	{
		robotforward(pwm_R, pwm_L);
	}

	//while

}
void turn_CW(int& pwm_R, int& pwm_L) {
	pwm_R = 2000;
	pwm_L = 1500;


}

void turn_CCW(int& pwm_R, int& pwm_L) {
	pwm_R = 1500;
	pwm_L = 1400;


}
double angle_calc(double& myCGx, double& myCGy, double& op_CGx, double& op_CGy) {
	double deltx;
	double delty;
	double desiredTheta;
	double deltatheta;
	deltx = myCGx - op_CGx;
	delty = myCGy - op_CGy;
	deltatheta = atan2(delty, deltx);
	deltatheta = (((deltatheta - -3.14) * (359 - 0)) / (3.14 - -3.14)) + 0;
	//cout << deltatheta << endl;

	//pause();
	return deltatheta;
}
void check_borders(image rgb, int& pw_r, int& pw_l) {
	image a;
	double mi_blue, mi_pink, mj_blue, mj_pink, ic_blue, ic_pink, jc_blue, jc_pink;
	blue_centroid(rgb, mi_blue, mj_blue, ic_blue, jc_blue);
	//pink_centroid(rgb, mi_pink, mj_pink, ic_pink, jc_pink);
	//draw_point_rgb(rgb, ic_pink, jc_pink, 0, 0, 0);
	//cout<< ic_pink<<endl;
	//cout << pinkx << endl;
	if (ic_blue < 40 || jc_blue < 40 || ic_blue>600 || jc_blue>440) {
		robotforward(pw_r, pw_l);
		cout << "border detection moving forward" << endl;
	}

	if (pinkx < 40 || pinky < 40 || pinkx>600 || pinky>440) {
		//cout << "ic_green" << ic_green << endl;
		//cout << "jc_green" << jc_green << endl;
		cout << "border detection reversing" << endl;
		robotreverse(pw_r, pw_l);

	}

}
void robotforward(int& pw_r, int& pw_l) {
	pw_r = 2000;
	pw_l = 1000;

}

void robotreverse(int& pw_r, int& pw_left) {
	pw_r = 1000;
	pw_left = 2000;

}


int hiding_obstacle_selection(double& myCGx, double& myCGy) {
	double distance;
	int selectedobject=0;
	double prev_distance = 1000000;
	for (int i = 0; i < w; i++) {
		distance = sqrt(((myCGx - obstacleic[i]) * (myCGx - obstacleic[i])) + (myCGy - obstaclejc[i]) * (myCGy - obstaclejc[i]));
		if (abs(prev_distance) > abs(distance)) {
			//cout << "distance= " << distance << endl;
			prev_distance = distance;
			selectedobject = i;
		}

	}
	//cout <<"obstacle cg : "<< obstacleic[selectedobject] << endl;
	//cout << "selected distance: " <<distance<< endl;
	return selectedobject;
	
}
void obstacle_hiding(double& desired_x, double& desired_y, double& op_CGx, double& op_CGy, image& rgb,double& my_CGx, double& my_CGy) {
	int selectedObstacle = hiding_obstacle_selection(my_CGx, my_CGy);
	double deltaX = op_CGx-obstacleic[selectedObstacle]  ;
	double deltaY = op_CGy - obstaclejc[selectedObstacle] ;
	double xgood;
	double ygood;
	if (deltaX < 0) {
		 xgood = obstacleic[selectedObstacle] + radius_obstacle[selectedObstacle];
	}
	if (deltaX > 0) {
		xgood = obstacleic[selectedObstacle] - radius_obstacle[selectedObstacle];
	}
	if (deltaY > 0) {
		ygood = obstaclejc[selectedObstacle] - radius_obstacle[selectedObstacle];

	}
	if (deltaY < 0) {
		ygood = obstaclejc[selectedObstacle] + radius_obstacle[selectedObstacle];

	}
	draw_point_rgb(rgb, xgood, ygood, 0, 0, 0);
	//double distance = 50.0;
	//double term1 = atan((op_CGy - obstaclejc[selectedObstacle])/(op_CGx - obstacleic[selectedObstacle]));
	//double term2 = tan(3.14 / 2 - term1)*tan(3.14/2-term1);
//	cout <<"term1: "<< term1 << endl;
	//double nominator = sqrt(term2 + 1);
//	double ygood = distance / nominator + obstaclejc[selectedObstacle];
	//double xgood = (distance/nominator) * tan(3.14 / 2 - term1)+ obstacleic[selectedObstacle];
	draw_point_rgb(rgb, xgood, ygood, 0, 0, 0);
}
void collision_detection(image& a) {
	ibyte* p;
	ibyte R, G, B;
	p = a.pdata;
	int width = a.width;
	int height = a.height;
	int size;
	size = height * width;
	for (int i = 0; i < w; i++) {
		p = a.pdata + 3*(int)obstacleic[i] + 3*(int)obstaclejc[i]*width;
		int z = 0;
		while (*(p) == 0 && *(p + 1) == 255 && *(p + 2) == 255) {
			z++;
			p = p + 3;
			//draw_point_rgb(a,obstacleic[i]+z,obstaclejc[i],255, 0, 0);
		}
	//	cout << "z= " <<z<< endl;
		draw_point_rgb(a, obstacleic[i] + z, obstaclejc[i], 255, 0, 0);
		radius_obstacle[i] = z;

	}



}
bool ObstacleInTheWay(double& myCGx, double& myCGy, double& theta) {
	//Calculate the distance between the robot and obstacle edge
	double distance;
	
	double ObstacleToRobot;
	for (int i = 0; i < w; i++) {
		//cout << "theta " << theta << endl;
		//cout << "cg and obstacle angle " << abs(abs(angle_calc(myCGx, myCGy, obstacleic[0], obstaclejc[0])))<<endl;
		distance = sqrt(((myCGx - obstacleic[i]) * (myCGx - obstacleic[i])) + (myCGy - obstaclejc[i]) * (myCGy - obstaclejc[i]));
		//deltatheta = abs((abs(angle_calc(myCGx, myCGy, obstacleic[0], obstaclejc[0]))) - theta);
		ObstacleToRobot = abs(abs(angle_calc(myCGx, myCGy, obstacleic[i], obstaclejc[i])));
		if (ObstacleToRobot > 180) { ObstacleToRobot = ObstacleToRobot - 180; }
		deltatheta = abs(ObstacleToRobot - theta);
		//cout << "obstacle to robot " << ObstacleToRobot << endl;
		//cout << "deltatheta: " << deltatheta << endl;
	//	cout << "distance : " << distance << endl;
		//cout << radius_obstacle[i]<<endl;
		if((distance <= (radius_obstacle[i] + 50))) { 
			
			//cout << radius_obstacle[i] << endl;
			//cout << "deltatheta: " << deltatheta << endl;
			//cout << "distance : " << distance << endl;
			return true; 
		}
	}

	return false;
}

bool evasion_initial(double& robot_cx, double& robot_cy, int& pw_r, int& pw_l,double &theta) {
	static bool done = false;
	if (done)
		return true;
	else if (robot_cy > 240 && robot_cx < 340) {
		if ((theta < 270 - 10) || (theta > 270 + 10)) {
			turn_CW(pw_r, pw_l);
		}

		else if (pinky < 420)robotforward(pw_r, pw_l);
		else {
			pw_r = 1500;
			pw_l = 1500;
			done = true;
		}
	}

	else if (robot_cy < 240 && robot_cx < 340) {
		if ((theta < 90 - 10) || (theta > 90 + 10)) {
			turn_CW(pw_r, pw_l);

		}

		else if (pinky >50)robotforward(pw_r, pw_l);
		else {
			pw_r = 1500;
			pw_l = 1500;
			done = true;
		}
	}
	else if (robot_cy < 240 && robot_cx > 340) {
		if ((theta < 180 - 10) || (theta > 180 + 10)) {
			turn_CW(pw_r, pw_l);
			done = true;
		}

		else if (pinkx > 620)robotforward(pw_r, pw_l);
		else {
			pw_r = 1500;
			pw_l = 1500;
			done = true;
		}
	}

	else if (robot_cy > 240 && robot_cx > 340) {
		if ((theta < 90 - 10) || (theta > 90 +10)) {
			turn_CW(pw_r, pw_l);
			done = true;
		}

		else if (pinkx > 620)robotforward(pw_r, pw_l);
		else {
			pw_r = 1500;
			pw_l = 1500;
			done = true;
		}
	}
	cout << "evading" << endl;
	return done;
}


void turncircles(double& robot_cx, double robot_cy, int& pw_r, int& pw_l, double& theta) {
	if ((robot_cy < 200 && robot_cx < 355)) {
		if (pinkx < 15 && (theta > 182 || theta < 178))turn_CW(pw_r, pw_l);
		else robotforward(pw_r, pw_l);

	}
	if ((robot_cy > 240 && robot_cx < 340)) {
		if (pinky < 460 && (theta > 182 || theta < 178))turn_CW(pw_r, pw_l);
		else robotforward(pw_r, pw_l);
	}
	if ((robot_cy > 240 && robot_cx > 340)) {
		if (pinkx < 620 && (theta > 92 || theta < 88))turn_CW(pw_r, pw_l);
		else robotforward(pw_r, pw_l);

	}
	if ((robot_cy < 240 && robot_cx > 340)) {
		if (pinkx < 620 && (theta < 272||theta>268))turn_CW(pw_r, pw_l);
		else robotforward(pw_r, pw_l);

	}
	cout << "turning" << endl;
	cout << "theta" <<theta<< endl;
}









