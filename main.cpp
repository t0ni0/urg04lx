/** This example is public domain. */


/**
 * @file
 *   @brief The serial interface process
 *
 *   This process connects any external MAVLink UART device and prints data
 *
 *   @author Lorenz Meier, <lm@inf.ethz.ch>
 *
 */
// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

#include "ring_buffer.h"

using std::string;
using namespace std;

struct timeval tv;		  ///< System time

// Settings
int sysid = 42;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int compid = 110;
int serial_compid = 0;
bool silent = false;              ///< Wether console output should be enabled
bool verbose = false;             ///< Enable verbose output
bool debug = false;               ///< Enable debug functions and output
int fd;

/**
 *
 *
 * Returns the file descriptor on success or -1 on error.
 */

int open_port(const char* port)
{
	int fd; /* File descriptor for the port */
	
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		/* Could not open the port. */
		return(-1);
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
	}
	
	return (fd);
}

bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	//struct termios options;
	
	struct termios  config;
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}
	//
	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	                    INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	                     ONOCR | OFILL | OPOST);

	#ifdef OLCUC 
  		config.c_oflag &= ~OLCUC; 
	#endif

  	#ifdef ONOEOT
  		config.c_oflag &= ~ONOEOT;
  	#endif

	//
	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	//
	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	//
	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0
	
	// Get the current options for the port
	//tcgetattr(fd, &options);
	
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, 460800) < 0 || cfsetospeed(&config, 460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, 921600) < 0 || cfsetospeed(&config, 921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;
			
			break;
	}
	
	//
	// Finally, apply the configuration
	//
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}
	return true;
}

void close_port(int fd)
{
	close(fd);
}

typedef enum {
	URG_COMMUNICATION_3_BYTE,
	URG_COMMUNICATION_2_BYTE,
} urg_range_data_byte_t;

bool scanRange(urg_range_data_byte_t comRange, 
		int startStep, int endStep, int clusterCount, int scanInterval,
		int numScans, int _fd){
	const char scanStartChar = 'M';
	const int SCAN_CMD_SIZE = 22;
	char buffer[SCAN_CMD_SIZE];
	char scanFormatChar;
	//if(comRange == URG_COMMUNICATION_3_BYTE)
		scanFormatChar = 'D';
	//else
	//	scanFormatChar = 'S';

	//min range = 10, max range = 750
	//add range padded with 0's
	int writeSize = snprintf(buffer, SCAN_CMD_SIZE, "%c%c%04d%04d%02d%01d%02d;LOL\n", scanStartChar, 
		scanFormatChar, startStep, endStep, clusterCount, scanInterval, numScans);
	
	int writtenSize = write(_fd, buffer, SCAN_CMD_SIZE);
	printf("Sent: %s", buffer);
	if(writeSize != writtenSize)
		return false;
	else
		return true;
}

int verifyStatus(uint8_t* statusArray){
	if(statusArray[0] == '0' && statusArray[1] == '0' && statusArray[2] == 'P')
		return 0;
	else if(statusArray[0] == '9' && statusArray[1] == '9' && statusArray[2] == 'b')
		return 1;
	else if(statusArray[0] == '0' && statusArray[1] != '0'){
		return 100;
	}
}

int byteDecode3(uint8_t* bytes)
{
	const uint8_t mask = 0x3f;	//keep first 6 bits
	bytes[0] -= 0x30;
	bytes[1] -= 0x30;
	bytes[2] -= 0x30;

	bytes[0] &= mask;
	bytes[1] &= mask;
	bytes[2] &= mask;

	int32_t result = (bytes[0] << 12) | (bytes[1] << 6) | bytes[2];
	return result;
}

int main(int argc, char **argv) {
	/* default values for arguments */
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 115200;
	const char *commandline_usage = "\tusage: %s -d <devicename> -b <baudrate> [-v/--verbose] [--debug]\n\t\tdefault: -d %s -b %i\n";

	/* read program arguments */
	int i;

	for (i = 1; i < argc; i++) { /* argv[0] is "mavlink" */
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf(commandline_usage, argv[0], uart_name, baudrate);
			return 0;
		}

		/* UART device ID */
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf(commandline_usage, argv[0], uart_name, baudrate);
				return 0;
			}
		}

		/* baud rate */
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf(commandline_usage, argv[0], uart_name, baudrate);
				return 0;
			}
		}

		/* terminating MAVLink is allowed - yes/no */
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}

		if (strcmp(argv[i], "--debug") == 0) {
			debug = true;
		}
	}

	// SETUP SERIAL PORT

	// Exit if opening port failed
	// Open the serial port.
	if (!silent) printf("Trying to connect to %s.. ", uart_name);
	fflush(stdout);

	fd = open_port(uart_name);
	if (fd == -1)
	{
		if (!silent) printf("failure, could not open port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("success.\n");
	}
	if (!silent) printf("Trying to configure %s.. ", uart_name);
	bool setup = setup_port(fd, baudrate, 8, 1, false, false);
	if (!setup)
	{
		if (!silent) printf("failure, could not configure port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("success.\n");
	}

	int noErrors = 0;
	if (fd == -1 || fd == 0)
	{
		if (!silent) fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
	}

	if(fd < 0)
	{
		exit(noErrors);
	}

	/*************************************************************
		Start out shit n shit
	**************************************************************/
	char buf = 0;
	write(fd, "SCIP2.0\n", 8);
	int lfCount = 0;
	while(lfCount < 3){
		read(fd, &buf, 1);
		printf("%X ", buf);
		if(buf == '\n' | buf == '\r')
			lfCount++;
	}

	lfCount = 0;
	ring_buffer_t rbuf;
	char _rbuf[256];
	ring_initialize(&rbuf, _rbuf, 8);
	printf("\nscanning\n");
	const int _nbrScans = 1;
	const int _nbrCluster = 34;
	scanRange(URG_COMMUNICATION_3_BYTE, 130, 639, _nbrCluster , 5, _nbrScans, fd);
	//scanRange(URG_COMMUNICATION_3_BYTE, 384, 384 , 1, 1, 1, fd);
	fflush(stdout);
	//read off all bytes until status bytes
	while(lfCount < 2){
		if(read(fd, &buf, 1) > 0){
			printf("%c ", buf);
			fflush(stdout);
			if(buf == '\n')
				lfCount++;
		}
	}

	//read off status bytes
	int bytesRead = 0;
	uint8_t status[3];
	while(bytesRead < 3){
		if(read(fd, &buf, 1) > 0){
			bytesRead++;
			status[bytesRead] = buf;
		}
	}
	fflush(stdout);

	int err;
	if(verifyStatus(status) >1){
		//not the message 00P status
		int consecutiveLF = 0;
		while(consecutiveLF < 2){
			if(read(fd, &buf, 1) > 0){
				if(buf == '\n') consecutiveLF++;
				else consecutiveLF = 0;
			}
		}
	} 
	else {
		//message is good, pop off bytes until we get to the data
		lfCount = 0;
		while(lfCount < 2){
			if(read(fd, &buf, 1) > 0){
				if(buf == '\n')
					lfCount++;
			}
		}

		uint8_t point[3];
		bytesRead = 0;
		bool keepGoing = true;
		int dataCount = 1;
		while(keepGoing){
			while(bytesRead < 3){
				if(read(fd, &buf, 1) > 0){
						point[bytesRead] = buf;
						bytesRead++;
				}
				
				if(point[0] != '\n' && point[1] != '\n' && point[2] != '\n'){
					printf("%d Decoded: %d\n", dataCount++, byteDecode3(point));
					fflush(stdout);
				}
				else{
					keepGoing = false;
					printf("End data\n");
					break;
				}
			}
			bytesRead = 0;
		}
	}
	close_port(fd);

	printf("\nProgram terminated\n");
	return 0;
}
