/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"/dev/ttyUSB0";
#endif
	int baudrate = 57600;//57600

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);


	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

	/*
	 * Instantiate a serial port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * serial port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock.
	 *
	 */
	Serial_Port serial_port(uart_name, baudrate);


	/*
	 * Instantiate an autopilot interface object
	 *
	 * This starts two threads for read and write over MAVlink. The read thread
	 * listens for any MAVlink message and pushes it to the current_messages
	 * attribute.  The write thread at the moment only streams a position target
	 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
	 * is changed by using the method update_setpoint().  Sending these messages
	 * are only half the requirement to get response from the autopilot, a signal
	 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
	 * method.  Signal the exit of this mode with disable_offboard_control().  It's
	 * important that one way or another this program signals offboard mode exit,
	 * otherwise the vehicle will go into failsafe.
	 *
	 */
	Autopilot_Interface autopilot_interface(&serial_port);

	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	serial_port.start();
	autopilot_interface.start();




	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	/*
	 * commands_EKF_Position sends the Ekf_data and commands just send test data from 1 to 30
	 */

	commands_EKF_Position(autopilot_interface);
	//commands(autopilot_interface);




	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	autopilot_interface.stop();
	serial_port.stop();
	

	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}






// ------------------------------------------------------------------------------
//   COMMANDS_EKF_POSITION
// ------------------------------------------------------------------------------
void
commands_EKF_Position(Autopilot_Interface &api)
{

//include libraries for reading the EKF.txt file and set a declaration for the necesarry variables
#include <fstream>
#include <iostream>

    ifstream infile;
    string myArray[6];
    ofstream yaw_file;
    double EKF_Data[6];

	// --------------------------------------------------------------------------
	//   START OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.enable_offboard_control();
	usleep(100); // give some time to let it sink in

	// now the autopilot is accepting setpoint commands


	// --------------------------------------------------------------------------
	//   SEND OFFBOARD COMMANDS
	// --------------------------------------------------------------------------
	printf("SEND OFFBOARD COMMANDS\n");

	// initialize command data strtuctures
	mavlink_set_position_target_local_ned_t sp;







    //Read EKF Data#


	int i=0;
	while (i<1)		//Read EKF Data in the loop

    //for (int j=0; j<100; j++) //read j-times the EKF.txt file and tries to send it via mavlink
                             //sometimes it needs more time to send the data than to read it so it could
                             //be that it sends less times than it reads the data
    {



  // open the EKF.txt file, set the EKF parameters to EKF_Data and close the txt file
         infile.open("/home/pi/src/RF_Localization/EKF.txt");
         if(infile.is_open())
                {
                for(int i=0; i<6; ++i)
                    {
                    infile >>myArray[i];
                    EKF_Data[i] = atof(myArray[i].c_str());
                     }
                 infile.close();
                 }



	// Example 2 - Set EKF_Data
	    set_EKF_Data( EKF_Data[0]  , // X_Position
	                EKF_Data[1]  , // Y_Position
	                EKF_Data[2]  , // EKF Matrix [0,0]
	                EKF_Data[3]  , // EKF Matrix [0,1]
	                EKF_Data[4]  , // EKF Matrix [1,0]
	                EKF_Data[5]  , // EKF Matrix [1,1]
				   sp         );




	// SEND THE COMMAND
	    api.update_setpoint(sp);



        usleep(30*1000); //sleep for x milliseconds -->(x*1000)
	    printf("\n");


            // --------------------------------------------------------------------------
                    //   GET A MESSAGE and write it in YAW.txt
                    // --------------------------------------------------------------------------
                    printf("READ SOME MESSAGES \n");

                    // copy current messages
                    Mavlink_Messages messages = api.current_messages;

                    // local position in ned frame

                    mavlink_attitude_t att = messages.attitude;
                    mavlink_debug_vect_t debug_vect = messages.debug_vect;
                    mavlink_debug_vect_t debug = messages.debug;
                    printf("Got message Attitude\n");
                    printf("    Attitude:  %f %f %f (m)\n", att.roll, att.pitch, att.yaw );
                    printf("    Debug_vect:  %f %f %f (m)\n", debug_vect.x, debug_vect.y, debug_vect.z);
                    printf("    Debug:  %f (m)\n", debug.value);

               //yaw_file.open("/home/pi/Localization/RF_Localization_Test/YAW.txt");
                 yaw_file.open("YAW_r_des.txt");
                        if(yaw_file.is_open()){
                        //yaw_file<<"\t"<<(double)att.yaw<< ","<< (double)debug_vect.x<< ","<< (double)debug_vect.y;
                        yaw_file<<"\t"<<(double)att.yaw<< ","<< (double)debug.value<<;
                        yaw_file.close();
                        }

                    printf("\n");

            }

     // end the for loop which rewrite the EKF_Data variable with the EKF parameters from the EKF.txt file

	// --------------------------------------------------------------------------
	//   STOP OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.disable_offboard_control();




	// --------------------------------------------------------------------------
	//   END OF EKF_COMMANDS
	// --------------------------------------------------------------------------

	return;

}





// --------------------------------------------------------------------------
//   COMMANDS
// --------------------------------------------------------------------------


void
commands(Autopilot_Interface &api)
{

    ifstream infile;
    string myArray[6];
    double EKF_Data[6];


	// --------------------------------------------------------------------------
	//   START OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.enable_offboard_control();
	usleep(100); // give some time to let it sink in

	// now the autopilot is accepting setpoint commands


	// --------------------------------------------------------------------------
	//   SEND OFFBOARD COMMANDS
	// --------------------------------------------------------------------------
	printf("SEND OFFBOARD COMMANDS\n");

	// initialize command data strtuctures
	mavlink_set_position_target_local_ned_t sp;

// Test how many times I can read the .txt file if I Write 10 EKF_positions per second in the text file

/*
for (int n=0; n<35; ++n)
{
cout << n<<endl;
   infile.open("/home/pi/Localization/RF_Localization_Test/EKF.txt");
         if(infile.is_open())
                {
                for(int i=0; i<6; ++i)
                    {
                    infile >>myArray[i];
                    EKF_Data[i] = atof(myArray[i].c_str());
                     }
                 infile.close();
                 }

cout<<myArray[0] << endl;
usleep(28.52*1000);
}*/

    for (int j=0; j<10; j++) //read j-times the EKF.txt file and tries to send it via mavlink
                             //sometimes it needs more time to send the data than to read it so it could
                             //be that it sends less times than it reads the data
    {

/*
    //find out how long it takes for opening the .txt file
#include <time.h>
//float times, timed;
clock_t times = clock();
*/

                for(int i=0; i<6; ++i)
                    {
                
                    EKF_Data[i] = j;
                     }
                 
                 
/*
clock_t timed = clock();
clock_t dif = timed-times;
double Time_in_Seconds = dif / (double) CLOCKS_PER_SEC;
cout << Time_in_Seconds<<endl;
*/


	//  Set EKF_Data
	    set_EKF_Data( EKF_Data[0]  , // X_Position
	                EKF_Data[1]  , // Y_Position
	                EKF_Data[2]  , // EKF Matrix [0,0]
	                EKF_Data[3]  , // EKF Matrix [0,1]
	                EKF_Data[4]  , // EKF Matrix [1,0]
	                EKF_Data[5]  , // EKF Matrix [1,1]

				   sp         );


	// SEND THE COMMAND
	    api.update_setpoint(sp);



        usleep(75*1000); //sleep for x milliseconds -->(x*1000)
	    printf("\n");

    } // end the for loop which rewrite the EKF_Data variable with the EKF parameters from the EKF.txt file

	// --------------------------------------------------------------------------
	//   STOP OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.disable_offboard_control();




	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------

	return;
}





// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}






// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}


