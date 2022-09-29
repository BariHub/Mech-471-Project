
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>

#include <windows.h>

#include "serial_com.h"
#include "timer.h"

// keyboard input macro
// check if key c is currently pressed
// note that multiple keys can be pressed at once
#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

using namespace std;

int main()
{
	int angle;
	short int ref_speed;
	HANDLE h1;
	const int NMAX = 64;
	char buffer[NMAX];
	int i, n, k, speed;

	

	short int* v;
	short int* theta;
	ref_speed = 100;


	// note this v is exactly analogous to the v in the arduino program
	v = (short int*)buffer;
	theta = (short int*)(buffer +sizeof(v));
	speed = 1;
	open_serial("COM5", h1, speed);

	while (1) {

		if (KEY(VK_UP))   buffer[1] = 'a';
		else if (KEY(VK_DOWN)) buffer[1] = 'b';
	else ref_speed = 0;
		//cout << buffer[1]<<endl;
		//part above doesn't work but due to the unpredictability of this section
		//i am scared to comment it out
		

		// send new data every 100 ms
		Sleep(10);

		v[0] = ref_speed;

		n = 3; // number of bytes to send
		


	if (KEY(VK_RIGHT)) buffer[2] = '8';
	else if (KEY(VK_LEFT))buffer[2] = 't';
	else buffer[2] = 's';

		//cout<<"pointer "<<theta[0]<<endl;
		serial_send(buffer, n, h1);
			//cout << "\n" << th1;
		// should wait briefly to complete the send and/or
		// to receive data from Arduino
		Sleep(10);

		// print out what the Arduino sends back
		// this is like a mini serial monitor program
		while (serial_available(h1) > 0) {
		serial_recv(buffer, 1, h1);
		cout << buffer[0];
		}

	}

	close_serial(h1);

	return 0;
}

