/***************************************************************

 Intelligent Ground Vehicle Competition 2009
 Pennsylvania State University - Robotics Club
 Learn more at www.psurobotics.org
 Protected by the GNU General Public License

 ***************************************************************/
/*
 Desc: Driver for RoboteQ AX2550 & AX2850
 Author: Pablo Rivera
 Modified by: Rich Mattes, Jeremy Bridon, Zack Correll
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
	bool HexToInt(char *String, int *DataOut);
	
	bool ReadHexFromSerial(char *command, int *output, bool IgnoreEcho = false);
	char IsHex(char Value);

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