
#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>
//attacking robot
using namespace std; 

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"
ofstream robo("myrobo.csv");
extern robot_system S1;
double obstacleic[3];
double deltatheta;
double obstaclejc[3];
int radius_obstacle[3];
int w;
double ic_green;
double jc_green;
void robotforward(int& pw_r, int& pw_l);
void robotreverse(int& pw_r, int& pw_left);
void check_borders(image rgb, int& pw_r, int& pw_l);
void red_centroid(image& a, double& mi_red, double& mj_red, double& ic_red, double& jc_red);
void green_centroid(image& a, double& mi_green, double& mj_green, double& ic_green, double& jc_green);
void calculate_position(image& a, double& x, double& y, double& theta);
void calculate_op_position(image& a, double& x, double& y, double& theta);
void obstacle_detection(image& a, image& b, image& label, image& rgb);
//int sobel(image& a, image& mag, image& theta,image& rgb);
void turn_CW(int& pwm_R, int& pwm_L);
void turn_CCW(int& pwm_R, int& pwm_L);
void Aim_Opponent(int& pwm_R, int& pwm_L, double& myCGx, double& myCGy, double& op_CGx, double& op_CGy,double &theta);
double angle_calc(double& myCGx, double& myCGy, double& op_CGx, double& op_CGy);
//bool collision_detection(image& a, double& ic_car, double& jc_car, double& ic_enemy, double& jc_enemy);
int sobel(image& a, image& mag, image& theta);
bool ObstacleInTheWay(double& myCGx, double& myCGy, double& theta);
int hiding_obstacle_selection(double& myCGx, double& myCGy);


int main()
{


	double robot_cx,op_cx;
	double robot_cy,op_cy;
	double robot_theta,op_theta;
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
	width1  = 640;
	height1 = 480;
	
	// number of obstacles
	N_obs  = 2;

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
	
	alpha_max = 3.14159/2; // max range of laser / gripper (rad)
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

	activate_simulation(width1,height1,x_obs,y_obs,size_obs,N_obs,
		"robot_A.bmp","robot_B.bmp","background.bmp","obstacle.bmp",D,Lx,Ly,
		Ax,Ay,alpha_max,n_robot);	

	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;
	
	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 0;
	level = 1;
	set_simulation_mode(mode,level);	
	
	// set robot initial position (pixels) and angle (rad)
	x0 = 470;
	y0 = 170;
	theta0 = 3.14;
	set_robot_position(x0,y0,theta0);
	
	// set opponent initial position (pixels) and angle (rad)
	x0 = 150;
	y0 = 375;
	theta0 = 3.14159/4;
	set_opponent_position(x0,y0,theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
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
	set_inputs(pw_l,pw_r,pw_laser,laser,
		light,light_gradient,light_dir,image_noise,
		max_speed,opponent_max_speed);

	// opponent inputs
	pw_l_o = 1700; // pulse width for left wheel servo (us)
	pw_r_o = 1400; // pulse width for right wheel servo (us)
	pw_laser_o = 1500; // pulse width for laser servo (us)
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
	
	image rgb,rgb0;
	int height, width;
	image a,theta,mag,b;
	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width  = 640;
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

	while(1) {
		
		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;
		calculate_position(rgb, robot_cx, robot_cy, robot_theta);
		calculate_op_position(rgb, op_cx, op_cy, op_theta);
		obstacle_detection(a, b, label, rgb);
		(robot_cx, robot_cy, op_cx, op_cy);
		
		view_rgb_image(rgb);
		//pause();
		double distance = sqrt(((ic_green - op_cx) * (ic_green - op_cx)) + (jc_green - op_cy) * (jc_green - op_cy));
		double ti;
		cout <<"angle "<< abs(abs(angle_calc(ic_green, jc_green, op_cx, op_cy)) - robot_theta) << endl;
		// fire laser
		cout <<"distance: "<< distance << endl;
		//pause();
		if ((abs(abs(angle_calc(ic_green, jc_green, op_cx, op_cy)) - robot_theta) < 22 && (distance<250))) {
			laser = 1;
			
			//pause();
			
		}
		
		

		// change the inputs to move the robot around
		// or change some additional parameters (lighting, etc.)
		
		// only the following inputs work so far
		// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
		// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
		// pw_laser -- pulse width of laser servo (us) (from 1000 to 2000)
		// -- 1000 -> -90 deg
		// -- 1500 -> 0 deg
		// -- 2000 -> 90 deg
		// laser -- (0 - laser off, 1 - fire laser for 3 s)
		// max_speed -- pixels/s for right and left wheels
		Aim_Opponent(pw_r, pw_l, robot_cx, robot_cy, op_cx, op_cy,robot_theta);
		check_borders(rgb, pw_r, pw_l);
		if (ObstacleInTheWay(robot_cx, robot_cy, robot_theta)) {
			robotreverse(pw_r, pw_l);

		}
		check_borders(rgb, pw_r, pw_l);
		set_inputs(pw_l,pw_r,pw_laser,laser,
			light,light_gradient,light_dir,image_noise,
			max_speed,opponent_max_speed);

		// manually set opponent inputs for the simulation
		// -- good for testing your program
		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
					opponent_max_speed);

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
				mi_red += i * 255; // (i moment of mk) = mk * i
				mj_red += j * 255; // (j moment of mk) = mk * j

				// original version: ic = 203.427 , jc = 346.4

				// moving summations to top so B goes from 0 - 255:
				// ic = 203.338 , jc = 346.215				

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


void calculate_position(image& a, double& x, double& y, double& theta) {
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


void blue_centroid(image& a, double& mi_blue , double& mj_blue, double& ic_blue, double& jc_blue) {
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
	jc_pink = mj_pink / (m + eps);
	//cout << "ic_green: " << ic_green << endl;
	//cout << "jc_green: " << jc_green << endl;
}
void calculate_op_position(image& a, double& x, double& y, double& theta) {
	ibyte* p;
	double mi_blue, mi_pink, mj_blue, mj_pink, ic_blue, ic_pink, jc_blue, jc_pink;
	pink_centroid(a, mi_blue, mj_blue, ic_blue, jc_blue);
	draw_point_rgb(a, ic_blue, jc_blue, 0, 0, 0);
	blue_centroid(a, mi_pink, mj_pink, ic_pink, jc_pink);
	draw_point_rgb(a, ic_pink, jc_pink, 0, 0, 0);
	x = (ic_blue + ic_pink) / 2;
	y = (jc_blue + jc_pink) / 2;
	theta = atan2(jc_pink - jc_blue, ic_pink - ic_blue);
	//cout << theta << endl;
	draw_point_rgb(a, x, y, 0, 0, 0);

}
int sobel(image& a, image& mag, image& theta)
{
	i4byte size, i, j;
	ibyte* pa, * pa1, * pa2, * pa3, * pa4, * pa5, * pa6, * pa7, * pa8, * pa9;
	ibyte* p_mag, * p_theta;
	i2byte width, height;

	// note we use a signed in here since sx, sy could be < 0
	int sx, sy, M;
	int kx[10], ky[10];
	double A;

	// check for compatibility image sizes and types
	if (a.height != mag.height || a.width != mag.width ||
		a.height != theta.height || a.width != theta.width)
	{
		printf("\nerror in convolution: sizes images are not the same!");
		return 1;
	}

	if (a.type != GREY_IMAGE || mag.type != GREY_IMAGE
		|| theta.type != GREY_IMAGE)
	{
		printf("\nerror in convolution: input types are not valid!");
		return 1;
	}

	width = a.width;
	height = a.height;

	// initialize pointers
	pa = a.pdata + width + 1;
	p_mag = mag.pdata + width + 1;
	p_theta = theta.pdata + width + 1;

	// set neighbourhood pointers

	// make sure they don't point outside of the images at the boundaries
	// when you use them

	// note the order of the neighbourhood is correctly given below
	// as discussed in class (the old order was for a different
	// image coord system in an older version of the library).
	// pa7 pa8 pa9
	// pa4 pa5 pa6
	// pa1 pa2 pa3
	pa1 = pa - width - 1;
	pa2 = pa - width;
	pa3 = pa - width + 1;
	pa4 = pa - 1;
	pa5 = pa;
	pa6 = pa + 1;
	pa7 = pa + width - 1;
	pa8 = pa + width;
	pa9 = pa + width + 1;

	// number of pixels to process
	size = (i4byte)a.width * a.height - 2 * width - 2;

	// set convolution coefficients for sx and sy
	// k7 k8 k9
	// k4 k5 k6
	// k1 k2 k3
	kx[7] = -1; kx[8] = 0; kx[9] = 1;
	kx[4] = -2; kx[5] = 0; kx[6] = 2;
	kx[1] = -1; kx[2] = 0; kx[3] = 1;

	ky[7] = 1;  ky[8] = 2;  ky[9] = 1;
	ky[4] = 0;  ky[5] = 0;  ky[6] = 0;
	ky[1] = -1; ky[2] = -2; ky[3] = -1;

	// calculate sx and sy
	// here I calculate both at the same time in the loop
	// since I don't want to store them into an image array
	// (they can't store negative numbers which might occur for sx
	// and sy) and I need both to calculate mag and theta.
	for (i = 0; i < size; i++) {

		sx = kx[1] * (*pa1) + kx[2] * (*pa2) + kx[3] * (*pa3) +
			kx[4] * (*pa4) + kx[5] * (*pa5) + kx[6] * (*pa6) +
			kx[7] * (*pa7) + kx[8] * (*pa8) + kx[9] * (*pa9);

		sy = ky[1] * (*pa1) + ky[2] * (*pa2) + ky[3] * (*pa3) +
			ky[4] * (*pa4) + ky[5] * (*pa5) + ky[6] * (*pa6) +
			ky[7] * (*pa7) + ky[8] * (*pa8) + ky[9] * (*pa9);

		// might consider directly substituting kx, ky above
		// to reduce computation time

		// calculate mag and theta
		M = abs(sx) + abs(sy); // fast approx of sqrt(sx*sx + sy*sy)

		if (M > 255) M = 255; // check for overflow
		// alternatively M can be scaled by 1/2, 1/4, 1/8
		// to reduce (1/2) or avoid (1/8) possibility of overlow

		*p_mag = M;

		A = atan2((double)sy, (double)sx) / 3.14159 * 180; // deg
		// note that A ranges from -180 to 180 deg

		// scale A so that it ranges from 0 to 255
		// and will fit in a greyscale image range
		// -- add 0.01 to account for roundoff error
		A = (A + 180) / 360 * 255 + 0.01;

		*p_theta = (int)A;

		// note this line might be useful to cut down
		// on the noise / irrelevant info from theta
		if (M < 75) *p_theta = 0;

		// increment pointers
		pa1++; pa2++; pa3++; pa4++; pa5++;
		pa6++; pa7++; pa8++; pa9++;
		p_mag++, p_theta++;
	}

	// copy edges of image from valid regions
	p_mag = mag.pdata;
	p_theta = theta.pdata;

	// number of pixels
	size = (i4byte)a.width * a.height;

	for (i = 0; i < width; i++) {
		p_mag[i] = p_mag[i + width]; // bottom
		p_mag[size - i - 1] = p_mag[size - i - 1 - width]; // top
		p_theta[i] = p_theta[i + width]; // bottom
		p_theta[size - i - 1] = p_theta[size - i - 1 - width]; // top
	}

	for (i = 0, j = 0; i < height; i++, j += width) {
		p_mag[j] = p_mag[j + 1]; // left
		p_mag[size - j - 1] = p_mag[size - j - 2]; // right
		p_theta[j] = p_theta[j + 1]; // left
		p_theta[size - j - 1] = p_theta[size - j - 2]; // right
	}

	return 0;
}

void Aim_Opponent(int& pwm_R, int& pwm_L, double& myCGx, double& myCGy, double& op_CGx, double& op_CGy,double &theta) {
	if (abs(abs(angle_calc(myCGx,myCGy, op_CGx, op_CGy))-theta)>10) {
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
double angle_calc(double &myCGx,double &myCGy,double &op_CGx,double &op_CGy){
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
void check_borders(image rgb,int &pw_r,int &pw_l) {
	image a;
	double mi_red, mi_green, mj_red, mj_green, ic_red, ic_green, jc_red, jc_green;
	red_centroid(rgb, mi_red, mj_red, ic_red, jc_red);
	green_centroid(rgb, mi_green, mj_green, ic_green, jc_green);
	//draw_point_rgb(a, ic_red, jc_red, 0, 0, 0);
	if (ic_red < 30 || jc_red < 30 || ic_red>610 || jc_red>450) {
		robotforward(pw_r, pw_l);

	}

	if (ic_green < 30 || jc_green < 30 || ic_green>610 || jc_green>450) {
		//cout << "ic_green" << ic_green << endl;
		//cout << "jc_green" << jc_green << endl;
		
		robotreverse(pw_r, pw_l);

	}

}
void robotforward(int &pw_r,int &pw_l) {
	pw_r = 2000;
	pw_l = 1200;

}

void robotreverse(int& pw_r, int& pw_left) {
	pw_r = 1000;
	pw_left = 2000;

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
		if ((distance <= (radius_obstacle[i] + 90))) {

			//cout << radius_obstacle[i] << endl;
			//cout << "deltatheta: " << deltatheta << endl;
			//cout << "distance : " << distance << endl;
			return true;
		}
	}

	return false;
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
int hiding_obstacle_selection(double& myCGx, double& myCGy) {
	double distance;
	int selectedobject = 0;
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