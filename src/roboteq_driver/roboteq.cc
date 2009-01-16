/***************************************************************

 Intelligent Ground Vehicle Competition 2009
 Pennsylvania State University - Robotics Club
 Learn more at www.psurobotics.org
 Protected by the GNU General Public License

 ***************************************************************/
/*
 Desc: Driver for RoboteQ AX2550 & AX2850
 Author: Pablo Rivera
 Modified by: Rich Mattes
 Date: May 2008
 */

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_roboteqplugin roboteqplugin
 * @brief Motor control driver for roboteqplugin AX2850

Provides position2d interface to the roboteqplugin AX2850 motor controller
http://www.roboteqplugin.com/ax2850-folder.html

This driver uses the configuration file options
max_rot_spd and max_trans_spd to scale commands sent to the controller.
These values can be determined by testing with RC -- or closed loop
could be implemented by integrating dead-reckoning devices.

The driver has been modified by psurobotics to return robot geometry information (specified in
config file), as well as odometry information from the encoder card.

@par Compile-time dependencies

- none

@par Provides

- @ref interface_position2d

@par Requires

- None

@par Configuration requests

- none

@par Configuration file options

- devicepath (string)
  - Default: none
  - The serial port to be used.

- baud (integer)
  - Default: 9600
  - The baud rate to be used.

- max_rot_spd (float)
  - Default: none
  - maximum rotational speed (in rad/sec) that would be achieved
	by vehicle if full power where applied to that channel.

- max_trans_spd (float)
  - Default: none
  - maximum translational speed (in meters/sec) that would be achieved
	by vehicle if full power where applied to that channel.

- motor_power (int)
  - Default: 0
  - Sets initial motor power state: 0 for disabled, 1 for enabled.

- robot_dimensions [float length, float width]
  - Default: [0.0 0.0]
  - Sets robot size in meters

@par Example

@verbatim
driver
(
  name "roboteqplugin"
  provides ["position2d:0"] 
  devicepath "/dev/ttyS0"
  baud 9600
  max_trans_spd 6.0
  max_rot_spd 4.0
  motor_power 1
  robot_dimensions [1.5 1.0]
)
@endverbatim

@author Pablo Rivera rivera@cse.unr.edu, modified by Rich Mattes rjm5066@psu.edu

 */
/** @} */


#include <time.h>
#include <assert.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/ioctl.h> // ioctl
#include <unistd.h> // close(2),fcntl(2),getpid(2),usleep(3),execvp(3),fork(2)
#include <netdb.h> // for gethostbyname(3) 
#include <netinet/in.h>  // for struct sockaddr_in, htons(3) 
#include <sys/types.h>  // for socket(2) 
#include <sys/socket.h>  // for socket(2) 
#include <signal.h>  // for kill(2) 
#include <fcntl.h>  // for fcntl(2) 
#include <string.h>  // for strncpy(3),memcpy(3) 
#include <stdlib.h>  // for atexit(3),atoi(3) 
#include <pthread.h>  // for pthread stuff 
#include <sys/poll.h> // for poll and poll_fd
#include <math.h>
#include <ctype.h> // For char/string manipulation: tolower(...)

#include <libplayercore/playercore.h>

// settings
#define SERIAL_BUFF_SIZE		128
#define MAX_MOTOR_SPEED			127
#define roboteqplugin_CON_TIMEOUT		10      // seconds to time-out on setting RS-232 mode
#define roboteqplugin_DEFAULT_BAUD	9600 
#define WHEEL_CIRCUMFERENCE .4084 // given in meters
#define ROBOT_BASE .5 // given in meters

// *************************************
// some assumptions made by this driver:

// roboteqplugin is in "mixed mode" where
// channel 1 is translation
// channel 2 is rotation

// roboteqplugin is set to be in RC mode by default

// the robot is a skid-steer vehicle where
// left wheel(s) are on one output,
// right wheel(s) on the other.
// directionality is implied by the following
// macros (FORWARD,REVERSE,LEFT,RIGHT)
// so outputs may need to be switched

// Tankbot motors face opposite ways, so
// these values were adjusted accordingly

// *************************************

#define FORWARD "!b"
#define REVERSE "!B"
#define LEFT "!A"
#define RIGHT "!a"
#define RIGHT_ENCODER "?Q4\r"
#define LEFT_ENCODER "?Q5\r"
#define RPM_ENCODER "?Z\r"

// Maximum tries to read before giving up
#define GIVEUP_MAX 100

///////////////////////////////////////////////////////////////////////////

class roboteqplugin:public Driver
{
private:
	int roboteqplugin_fd;
	char serialin_buff[SERIAL_BUFF_SIZE];
	char serialout_buff[SERIAL_BUFF_SIZE];
	const char* devicepath;
	int roboteqplugin_baud;
	double speed_scaling_factor, rot_scaling_factor;
	int motorPower;
	int motorPowerConfigfile;
	float motorRot;
	float motorTrans;
	float robotLength;
	float robotWidth;

	int FormMotorCmd(char* cmd_str, short trans_command, short rot_command);
	int ProcessEncoderData();
	int readVelocity(char command [6], int *RPM_right, int *RPM_left);
	int readEncoder(char command [6], int *encoder_value);

	// Converts a given string to a double value (A while number)
	bool hextodec(char *string, int *output);
	bool twohextodec(char *string, int *output);

	bool ReadHexFromSerial(char *command, int *output, bool IgnoreEcho = false);
	bool isHex(char input);

	// current data
	player_position2d_data_t data;
	player_devaddr_t position2d_id;
	player_position2d_data_t posdata;

public:
	roboteqplugin( ConfigFile* cf, int section);

	virtual int ProcessMessage(QueuePointer &resp_queue, 
			player_msghdr * hdr, void * data);
	virtual int Setup();
	virtual int Shutdown();
	virtual void Main();
};


///////////////////////////////////////////////////////////////////////////
// initialization function
Driver* roboteqplugin_Init( ConfigFile* cf, int section)
{
	return((Driver*)(new roboteqplugin( cf, section)));
}


///////////////////////////////////////////////////////////////////////////
// a driver registration function
void roboteqplugin_Register(DriverTable* table)
{
	table->AddDriver("roboteqplugin",  roboteqplugin_Init);
}
////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.

/* need the extern to avoid C++ name-mangling  */
extern "C"  // Mod plugin
{
	int player_driver_init(DriverTable* table)  // Mod plugin
	{
		roboteqplugin_Register(table);  // Mod plugin
		return(0);  // Mod plugin
	}
}

///////////////////////////////////////////////////////////////////////////
roboteqplugin::roboteqplugin( ConfigFile* cf, int section) : Driver(cf, section)
{  
	memset (&this->position2d_id, 0, sizeof (player_devaddr_t));

	// Outgoing position 2d interface
	if(cf->ReadDeviceAddr(&(this->position2d_id), section, "provides",
			PLAYER_POSITION2D_CODE, -1, NULL) == 0){
		if(this->AddInterface(this->position2d_id) != 0){
			this->SetError(-1);
			return;		
		}   }

	double max_trans_spd, max_rot_spd;

	//Required parameters   
	if(!(this->devicepath = (char*)cf->ReadString(section, "devicepath", NULL))){
		PLAYER_ERROR("must specify devicepath");
		this->SetError(-1);
		return;
	}
	if(!(max_trans_spd = (double)cf->ReadFloat(section, "max_trans_spd", 0))){
		PLAYER_ERROR("must specify maximum translational speed");
		this->SetError(-1);
		return;
	}
	if(!(max_rot_spd = (double)cf->ReadFloat(section, "max_rot_spd", 0))){
		PLAYER_ERROR("must specify maximum rotational speed");
		this->SetError(-1);
		return;
	}

	//Optional Parameters
	motorPowerConfigfile = cf->ReadInt(section, "motor_power", 0);

	robotLength = cf->ReadTupleLength(section, "robot_dimensions", 0, 0.0);
	robotWidth = cf->ReadTupleLength (section, "robot_dimensions", 1, 0.0);

	fprintf(stderr, "roboteqplugin: using a max translational speed of %f m/s\n\tand max rotational speed of %f rad/s to scale motor commands\n", max_trans_spd, max_rot_spd);

	speed_scaling_factor = ((double)(MAX_MOTOR_SPEED / max_trans_spd));
	rot_scaling_factor = ((double)(MAX_MOTOR_SPEED / max_rot_spd));

	roboteqplugin_baud = cf->ReadInt(section, "baud", roboteqplugin_DEFAULT_BAUD);

	memset(&data,0,sizeof(data));
	roboteqplugin_fd = -1;

	return;
}

///////////////////////////////////////////////////////////////////////////
int
roboteqplugin::Setup()
{
	int ret, i;

	// Set Up Serial Port
	PLAYER_MSG1(0, "Configuring roboteqplugin serial port at %s..\n", devicepath);
	roboteqplugin_fd = open(devicepath, O_RDWR|O_NDELAY);
	if (roboteqplugin_fd == -1){
		fputs("Unable to configure serial port for roboteqplugin!", stderr);
		return 0; 
	}else{
		struct termios options;

		tcgetattr(roboteqplugin_fd, &options);

		// default is 9600 unless otherwise specified

		if (roboteqplugin_baud == 4800){
			cfsetispeed(&options, B4800);
			cfsetospeed(&options, B4800);
		}
		else if (roboteqplugin_baud == 19200){
			cfsetispeed(&options, B19200);
			cfsetospeed(&options, B19200);
		}
		else if (roboteqplugin_baud == 38400){
			cfsetispeed(&options, B38400);
			cfsetospeed(&options, B38400);
		}
		else{
			cfsetispeed(&options, B9600);
			cfsetospeed(&options, B9600);
		}

		// set to 7bit even parity, no flow control
		options.c_cflag |= (CLOCAL | CREAD);
		options.c_cflag |= PARENB;
		options.c_cflag &= ~PARODD;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS7;
		options.c_cflag &= ~CRTSCTS;

		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    // non-canonical

		tcsetattr(roboteqplugin_fd, TCSANOW, &options);
		ioctl(roboteqplugin_fd, TCIOFLUSH, 2);
	}


	// initialize roboteqplugin to RS-232 mode
	strcpy(serialout_buff, "\r");
	for (i=0; i<10; i++){ 
		write(roboteqplugin_fd, serialout_buff, 1);
		tcdrain(roboteqplugin_fd);
		usleep(25000);
	}

	// check response from roboteqplugin
	bzero(serialin_buff, SERIAL_BUFF_SIZE); 
	ret = read(roboteqplugin_fd, serialin_buff, SERIAL_BUFF_SIZE);
	int beg_time = time(NULL);
	bool mode_changed = true;
	while (! strchr(serialin_buff, 'W')){
		if ((time(NULL) - beg_time)>roboteqplugin_CON_TIMEOUT){
			mode_changed = false;
			break;
		}
		bzero(serialin_buff, SERIAL_BUFF_SIZE); 
		ret = read(roboteqplugin_fd, serialin_buff, SERIAL_BUFF_SIZE);
	}
	if (!mode_changed)
		fputs("Failed to set roboteqplugin to RS-232 mode!\n", stderr);
	else
		fputs("Successfully initialized roboteqplugin connection.\n", stderr);

	fputs("Done.\n", stderr); 

	this->motorPower = this->motorPowerConfigfile;

	// now spawn reading thread 
	StartThread();

	return(0);
}


///////////////////////////////////////////////////////////////////////////
int roboteqplugin::Shutdown()
{
	int ret;

	StopThread();

	// return roboteqplugin to RC mode
	strcpy(serialout_buff, "^00 00\r");
	write(roboteqplugin_fd, serialout_buff, 7);
	tcdrain(roboteqplugin_fd);
	usleep(25000);
	strcpy(serialout_buff, "%rrrrrr\r");
	write(roboteqplugin_fd, serialout_buff, 8);
	tcdrain(roboteqplugin_fd);
	usleep(25000);

	// check response from roboteqplugin
	bzero(serialin_buff, SERIAL_BUFF_SIZE); 
	ret = read(roboteqplugin_fd, serialin_buff, SERIAL_BUFF_SIZE);
	int beg_time = time(NULL);
	while (! strchr(serialin_buff, 'W')){
		if ((time(NULL) - beg_time)>roboteqplugin_CON_TIMEOUT){
			// no 'W's for roboteqplugin_CON_TIMEOUT seconds
			//		means we're probably in RC mode again

			// 07-09-07 
			// this test may need to change since the reset
			// appears to fail quite often. is it really
			// failing or is the test bad?
			return 0; 
		}
		bzero(serialin_buff, SERIAL_BUFF_SIZE); 
		ret = read(roboteqplugin_fd, serialin_buff, SERIAL_BUFF_SIZE);
	}
	fputs("Unable to reset roboteqplugin to RC mode!", stderr);

	close(roboteqplugin_fd);

	return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Process an incoming message
////////////////////////////////////////////////////////////////////////////////
int roboteqplugin::ProcessMessage (QueuePointer &resp_queue, player_msghdr * hdr, void * data)
{
	assert(hdr);
	//	assert(data);
	/*
    fprintf(stderr, "ProcessMessage: type=%d subtype=%d\n", 
            hdr->type, hdr->subtype);
	 */      
	if (Message::MatchMessage(hdr, PLAYER_POSITION2D_REQ_MOTOR_POWER, 
			PLAYER_POSITION2D_CMD_VEL, position2d_id)){         
		assert(hdr->size == sizeof(player_position2d_cmd_vel_t));

		player_position2d_cmd_vel_t & command 
		= *reinterpret_cast<player_position2d_cmd_vel_t *> (data);

		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// convert from the generic position interface 
		// to the roboteqplugin-specific command
		// assumes "Mixed Mode" -- 
		// channel 1 : FW/BW speed
		// channel 2 : rotation
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		float vel_xtrans = command.vel.px;
		//float vel_ytrans = command.vel.py;
		float vel_yawspd = command.vel.pa;

		//fprintf(stderr, "ProcessMessage: trans=%f, steer=%f\n", vel_trans, vel_turret);
		if (motorPower)
		{

			// scale and translate to roboteqplugin command
			vel_xtrans = (double)vel_xtrans * speed_scaling_factor;
			vel_yawspd = (double)vel_yawspd * rot_scaling_factor;
			FormMotorCmd(serialout_buff, (short)vel_xtrans, (short)vel_yawspd);

			// write motor cmd
			write(roboteqplugin_fd, serialout_buff, strlen(serialout_buff)+1);
			tcdrain(roboteqplugin_fd);
		}
		else
			printf("Cannot set speed, motors not enabled!\n");

		return 0;
	}

	//Motor Power Message
	if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_MOTOR_POWER, this->position2d_id))
	{
		/* motor state change request
		 *   1 = enable motors (any non-zero number will work)
		 *   0 = disable motors (default)
		 */
		if(hdr->size != sizeof(player_position2d_power_config_t))
		{
			PLAYER_WARN("Arg to motor state change request wrong size; ignoring");
			return(-1);
		}
		player_position2d_power_config_t* power_config = (player_position2d_power_config_t*)data;
		this->motorPower = power_config->state;

		if (this->motorPower == 0)
		{
			printf ("Disabling Motors...\n");
			FormMotorCmd(serialout_buff, 0, 0);

			// write motor cmd
			write(roboteqplugin_fd, serialout_buff, strlen(serialout_buff)+1);
			tcdrain(roboteqplugin_fd);
		}
		else if (this->motorPower != 0)
		{
			printf ("Motors enabled!\n");
		}

		this->Publish(this->position2d_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_MOTOR_POWER);
		return 0;
	}

	//Get Geometry Message
	if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,PLAYER_POSITION2D_REQ_GET_GEOM,position2d_id))
	{	
		/* Return the robot geometry. */
		if(hdr->size != 0)
		{
			PLAYER_WARN("Arg get robot geom is wrong size; ignoring");
			return(-1);
		}

		player_position2d_geom_t geom;

		// Use a default pose
		geom.pose.px = 0.0;
		geom.pose.py = 0.0;
		geom.pose.pyaw = 0.0;

		// Use the dimensions of the vehicle specified in the config file
		geom.size.sl = this->robotLength;
		geom.size.sw = this->robotWidth;

		// Publish the reply to the request
		Publish(position2d_id, resp_queue,PLAYER_MSGTYPE_RESP_ACK,PLAYER_POSITION2D_REQ_GET_GEOM,(void*)&geom, sizeof(geom), NULL);
		return 0;
	}
	return -1;
}

///////////////////////////////////////////////////////////////////////////
// Main driver thread runs here.
///////////////////////////////////////////////////////////////////////////
void roboteqplugin::Main()
{

	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL);

	for(;;){
		ProcessMessages();
		pthread_testcancel();
		ProcessEncoderData();
		//usleep(10);
		sleep(1);
	}

	pthread_exit(NULL);

	return;
}

///////////////////////////////////////////////////////////////////////////
int roboteqplugin::FormMotorCmd(char* cmd_str, short trans_command, short rot_command)
{
	char speed[8];
	char heading[8];

	//Check to make sure desired speed falls within safe boundaries
	if (trans_command > MAX_MOTOR_SPEED) trans_command = MAX_MOTOR_SPEED;
	else if (trans_command < -MAX_MOTOR_SPEED) trans_command = -MAX_MOTOR_SPEED;
	if (rot_command > MAX_MOTOR_SPEED) rot_command = MAX_MOTOR_SPEED;
	else if (rot_command < -MAX_MOTOR_SPEED) rot_command = -MAX_MOTOR_SPEED;

	//Update global variables with new speeds
	motorTrans = trans_command;
	motorRot = rot_command;

	//Add correct command prefix for wanted direction
	if (trans_command > 0)
		strcpy(speed, FORWARD);
	else strcpy(speed, REVERSE);
	if (rot_command > 0)
		strcpy(heading, LEFT);
	else strcpy(heading, RIGHT);

	//Form the rest of the motor cmd string
	strcpy(cmd_str, speed);
	snprintf(cmd_str+2, 4, "%.2x", abs(trans_command)); // start at char 3
	strcat(cmd_str, "\r");	
	strcat(cmd_str, heading);
	snprintf(cmd_str + strlen(cmd_str), 4, "%.2x", abs(rot_command));
	strcat(cmd_str, "\r");

	return 0;
}

int roboteqplugin::ProcessEncoderData()
{
	int left_encoder = 0;
	int right_encoder = 0;
	int left_RPM = 0;
	int right_RPM = 0;

	readEncoder((char*)LEFT_ENCODER, &left_encoder);
	readEncoder((char*)RIGHT_ENCODER,&right_encoder);
	readVelocity((char*)RPM_ENCODER, &right_RPM, &left_RPM);

	printf("left encoder value:%d   right encoder value: %d \n",left_encoder,right_encoder);fflush(stdout);
	printf("left RPM value:%d   right RPM value: %d \n",left_RPM,right_RPM);fflush(stdout);
	// Then do math to convert differential stuff to x and theta and fill in px, pa, vx, va
	double distance = ((((double)left_encoder + (double)right_encoder)/(2.0 * 2048.0)) *WHEEL_CIRCUMFERENCE);
	double thetaPos = ((double)left_encoder - (double)right_encoder) / (ROBOT_BASE * 2048.0);

	posdata.pos.px += distance * cos(thetaPos);
	posdata.pos.py += distance * sin(thetaPos);
	posdata.pos.pa += thetaPos;

	double position_time=0;

	double velocity = ((left_RPM + right_RPM) *WHEEL_CIRCUMFERENCE)/(60.0* 2.0);
	double thetaVel = (left_RPM - right_RPM) / ROBOT_BASE; 
	posdata.vel.px = velocity * cos(thetaVel);
	posdata.vel.py = velocity * sin(thetaVel);
	posdata.vel.pa = thetaVel;

	posdata.stall=0;

	Publish(device_addr, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE, 
			(unsigned char*) &posdata, sizeof(posdata), &position_time);
	return 0;
}

int roboteqplugin::readEncoder(char command[6],int *encoder_value)
{
	int out;
	if(ReadHexFromSerial(command, &out))
	{
		printf("readEncoder::Input Error: ReadHexFromSerial returned true!\n");
		fflush(stdout);
	}

		*encoder_value = out;

		return 0;
}

int roboteqplugin::readVelocity(char command[6], int *RPM_right, int *RPM_left)
{
	int out;
	// reads the first velocity value from the encoder
	if(ReadHexFromSerial(command, &out))
		{printf("readVeloocity::Input Error: ReadHexFromSerial returned true !\n");fflush(stdout);}
	*RPM_right = out;
	// reads the second encoder value from the encoder // TODO fix the naming of the function
	//if(ReadVelHexFromSerial(command, &out))
	if(ReadHexFromSerial(command, &out, true))
		{printf("readVeloocity::Input Error: ReadHexFromSerial returned true !\n");fflush(stdout);}
	*RPM_left = out;
	return 0;
}

// Takes char input and returns true if hex
bool roboteqplugin::isHex(char input)
{
	// Valid set of chars
	char valid[] = "0123456789abcdef";

	// To lower
	input = tolower(input);

	// Validate the character
	bool checked = false;
	for(int j = 0; j < (int)strlen(valid) && !checked; j++)
		if(input == valid[j])
			checked = true;

	// Check if checked, this was false !!!
	if(checked == true)
		return true;
	return false; // Not hex
}

// Input is a hexadecimal string
// Outpout, through a pointer, is the decimal value
// If it returns true, there was an error, ignore output
bool roboteqplugin::hextodec(char* inputData, int *output)
{
	// The final data
	unsigned int data = 0;

	// The hex base for the position
	unsigned int base = 1;

	// Save the string length
	int inputRead = (int)strlen(inputData);
	inputData[inputRead] = '\0';
	//printf("  input data: %s  ",inputData);

	// For each char in the hex string, from right to left
	for(int i = inputRead - 1; i >= 0; i--)
	{
		// Look at the current hex char, and cast it to a lower char if needed
		unsigned int temp = 0;
		if( !isHex(inputData[i]) )
			return true; // Error

		// If the char is a digit, cast from ASCII to decimal
		if(inputData[i] >= '0' && inputData[i] <= '9')
			temp = inputData[i] - (int)'0';

		// Else if a letter, cast from ASCII to decimal
		else if(inputData[i] >= 'a' && inputData[i] <= 'z')
			temp = inputData[i] - (int)'a' + 10;

		// Add the data by the multiple and grow it
		data += (temp * base);
		base *= 16;
	}

	// Post answer and print data
	*output = (int)data;
	printf("hextodec: %s => (int)%d, (uint)%u\n", inputData, *output, data);

	// Return no error
	return false;
}

// Input: Command string, and expected packets returned
// If IgnoreEcho is true, though it is defaulted to false, it will ignore the needed echo from the motor controller
// Output: returns bool true on failure, false on success and a pointer to an array of interegers
// NOTE: Please allocate the space yourself for the output pointer
bool roboteqplugin::ReadHexFromSerial(char *command, int *output, bool IgnoreEcho)
{
	// Send out command
	write(roboteqplugin_fd, command, strlen(command));
	tcdrain(roboteqplugin_fd);

	// Start index of command string
	int commandindex = 0;
	int giveup = 0;

	// Wait for command to be echoed (If we are waiting for an echo)
	while(true && !IgnoreEcho)
	{
		// Read single character
		int dataread = 0;
		int bytesread = read(roboteqplugin_fd, &dataread, 1);

		//if(dataread!=0)
		//printf("Does %d == %d??",dataread,command[commandindex]);fflush(stdout);
		// If giveup reaches too high, we give up
		if(giveup > GIVEUP_MAX)
		{
			printf("ReadHexFromSerial::Give up max reached\n");fflush(stdout);
			return true;
		}

		// If < 0, error    i commented this out
		//if(bytesread < 0)
		//{
		//	printf("ReadHexFromSerial::Bytesread < 0\n");fflush(stdout);
		//	return true;
		//}

		// Nothing read...
		else if(bytesread == 0)
		{
			// Do nothing...
			giveup++;
		}
		// If there is data, and the data matches		
		else if(bytesread > 0 && (dataread == command[commandindex]))
		{
			// Grow the commandindex
			commandindex++;
			if(commandindex > (int)strlen(command) - 2)
				break;
		}
	}
	
	// Read off any non-visible characters
	int dataread;
	do
	{
		// Read in a byte
		int bytesread = read(roboteqplugin_fd, &dataread, 1);

		// if error with read, return error
		//if(bytesread < 0)
		//{
		//	printf("ReadHexFromSerial:: bytesread<0 \n");fflush(stdout);
		//	return true;
		//}
		// If read in nothing, try again
		// was an else if !!!
		if(bytesread == 0)
			giveup++;

		// If we have failed too many times, give up
		if(giveup > GIVEUP_MAX)
		{
			printf("ReadHexFromSerial:: give up is greater than giveup max \n");fflush(stdout);
			return true;
		}
	}
	while(!isHex(dataread));
	
	//printf("found hex value->%d \n",dataread);fflush(stdout);
	// Create the input buffer
	char buffer[16];
	buffer[0] = (char)dataread;
	int bufferindex = 1;

	// Read in data
	do
	{
		// Read in a byte
		int bytesread = read(roboteqplugin_fd, &buffer[bufferindex], 1);
		//printf("%s ",buffer);
		// if error with read, return error
		//if(bytesread < 0)
		//{
		//	printf("ReadHexFromSerial::Bytesread<0 \n");fflush(stdout);
		//	return true;
		//}
		// If read in nothing, try again, was else if!!!
		if(bytesread == 0)
			giveup++;
		// Else, data read, increment index of buffer
		else
			bufferindex++;

		// If we have failed too many times, give up
		if(giveup > GIVEUP_MAX)
		{
			printf("ReadHexFromSerial::Give up is greater than giveup max \n");fflush(stdout);
			return true;
		}
	}
	while(isHex(buffer[bufferindex]));
	
	//printf("buffer filled \n");fflush(stdout);
	// Cap off the hex string
	buffer[bufferindex] = '\0';

	//printf("ReadHexFromSerial::buffer: %s----->",buffer);fflush(stdout);
	// Convert this packet to a decimal
	hextodec(buffer, output);

	//printf("output: %d \n",output);fflush(stdout);

	// Return false for no error
	return false;
}
