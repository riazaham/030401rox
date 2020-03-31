
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <ncurses.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"
#include "unistd.h"

#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B9600

int exitFlag=0;
int mode = 0;

char d = 'a';
char command = 'x';
char prevcommand = 'x';
char finalcommand = 'x';
int count = 0;
int dircount = 0;
int state = 0;
int commandflag = 0;
int ok_flag = 1;

sem_t _xmitSema;

void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			printf("ERROR: Bad Magic Number\n\r");
			break;

		case PACKET_CHECKSUM_BAD:
			printf("ERROR: Bad checksum\n\r");
			break;

		default:
			printf("ERROR: UNKNOWN ERROR\n\r");
	}
}

void handleStatus(TPacket *packet)
{
	printf("\n\r ------- ALEX STATUS REPORT ------- \n\r\n\r");
	printf("Left Forward Ticks:\t\t%d\n\r", packet->params[0]);
	printf("Right Forward Ticks:\t\t%d\n\r", packet->params[1]);
	printf("Left Reverse Ticks:\t\t%d\n\r", packet->params[2]);
	printf("Right Reverse Ticks:\t\t%d\n\r", packet->params[3]);
	printf("Left Forward Ticks Turns:\t%d\n\r", packet->params[4]);
	printf("Right Forward Ticks Turns:\t%d\n\r", packet->params[5]);
	printf("Left Reverse Ticks Turns:\t%d\n\r", packet->params[6]);
	printf("Right Reverse Ticks Turns:\t%d\n\r", packet->params[7]);
	printf("Forward Distance:\t\t%d\n\r", packet->params[8]);
	printf("Reverse Distance:\t\t%d\n\r", packet->params[9]);
	printf("\n\r---------------------------------------\n\r\n\r");
}

void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			ok_flag = 1;
			printf("Command OK\n\r");
			break;

		case RESP_STATUS:
			handleStatus(packet);
			break;

		default:
			printf("Arduino is confused\n\r");
	}
}

void handleErrorResponse(TPacket *packet)
{
	// The error code is returned in command
	switch(packet->command)
	{
		case RESP_BAD_PACKET:
			printf("Arduino received bad magic number\n\r");
			break;

		case RESP_BAD_CHECKSUM:
			printf("Arduino received bad checksum\n\r");
			break;

		case RESP_BAD_COMMAND:
			printf("Arduino received bad command\n\r");
			break;

		case RESP_BAD_RESPONSE:
			printf("Arduino received unexpected response\n\r");
			break;

		default:
			printf("Arduino reports a weird error\n\r");
	}
}

void handleMessage(TPacket *packet)
{
	printf("Message from Alex: %s\n\r", packet->data);
}

void handlePacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
			// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
			handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
			handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
			handleMessage(packet);
			break;
	}
}

void sendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len);
	ok_flag = 0;
}

void *receiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handlePacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					printf("PACKET ERROR\n\r");
					handleError(result);
				}
		}
	}
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

void getParams(TPacket *commandPacket)
{
	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n\r");
	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n\r");
	scanf("%d %d", &commandPacket->params[0], &commandPacket->params[1]);
	flushInput();
}

void getParamsAuto(TPacket *commandPacket)
{
	switch(commandPacket->command){
		case COMMAND_FORWARD:
			commandPacket->params[0] = 0;
			commandPacket->params[1] = 50;
			break;
		case COMMAND_REVERSE:
			commandPacket->params[0] = 0;
			commandPacket->params[1] = 50;
			break;
		case COMMAND_TURN_LEFT:
			commandPacket->params[0] = 0;
			commandPacket->params[1] = 90;
			break;
		case COMMAND_TURN_RIGHT: 
			commandPacket->params[0] = 0;
			commandPacket->params[1] = 70;
			break;
	}
}

void sendCommand(char command)
{
	TPacket commandPacket;

	commandPacket.packetType = PACKET_TYPE_COMMAND;

	switch(command)
	{
		case 'p':
		case 'P':
			mode = (mode == 1)?0:1;
			break;

		case 'w':
		case 'W':
			getParamsAuto(&commandPacket);
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
			break;

		case 'a':
		case 'A':
			getParamsAuto(&commandPacket);
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;

		case 's':
		case 'S':
			getParamsAuto(&commandPacket);
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket);
			break;

		case 'd':
		case 'D':
			getParamsAuto(&commandPacket);
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;


		case 'f':
		case 'F':
			getParams(&commandPacket);
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
			break;

		case 'b':
		case 'B':
			getParams(&commandPacket);
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket);
			break;

		case 'l':
		case 'L':
			getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;

		case 'r':
		case 'R':
			getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;

		case 'x':
		case 'X':
			commandPacket.command = COMMAND_STOP;
			sendPacket(&commandPacket);
			break;

		case 'c':
		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			commandPacket.params[0] = 0;
			sendPacket(&commandPacket);
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			sendPacket(&commandPacket);
			break;

		case 'q':
		case 'Q':
			exitFlag=1;
			break;

		default:
			printf("Bad command\n\r");

	}
}


///////////////////////////////////////////////
/*void* command_toggle_thread(void* p){
  char prev = 'x';
  while(1){

  command = d;
  prev = d;
  dircount = 0;
  }
  else dircount++;
  }
  else command = 'x';
  }
  }*/


void* change_detect_thread(void* p){
	if(command == -1) command = prevcommand;
	/*clock_t time;
	  int count = 0;
	  int curr;
	  int i = 0;
	  int j = 0;
	  while(1){
	  if(j == 1) command = d;
	  else if(i %3 == 0) command = 'x';

	  if (state) {
	//printw("Key pressed! It was: %d\n", getch());
	getch();
	i = 0;
	j = 1;
	refresh();
	} else {
	count++;
	if(count <= 2) usleep(200000);

	else{
	count = 0;
	i++;
	j = 0;
	refresh();
	usleep(100000);
	}
	}
	}


	time = clock();
	prev = count;
	while(clock() - time < 300000);
	if(count == prev){
	command = 'x';
	count = 0;
	} 
	} */


	}

void* movement_change_thread(void* p){
	char prev = 'x';	
	while(1){
		if(command != prev){
			//	if(command == 'x') printw("Easy Mode (w=forward, s=reverse, a=turn left, d=turn right, x = stop, c=clear stats, g=get stats, q=exit)\n");
			prev = command;
			finalcommand = command;
			commandflag = 1;
			if (ok_flag){
			       	sendCommand(finalcommand);
				
			}
			
			commandflag = 0;

		}
	}
}


int kbhit(void)
{
	d = getch();
	//printw("d is %c, %d\n",d, d); 
	if (d != (char)255) {
		ungetch(d);
		return 1;
	} else {
		return 0;
	}
}
///////////////////////////////////////////////////////////////



int main()
{
	int start = 0;
	int i= 0, j = 0;
	int _count = 0;
	// Connect to the Arduino
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);

	// Sleep for two seconds
	printf("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n\r");
	sleep(2);
	printf("DONE\n\r");

	// Spawn receiver thread
	pthread_t recv;
	pthread_t commandthread[3];
	pthread_create(&recv, NULL, receiveThread, NULL);
	//pthread_create(&commandthread[0], NULL, command_toggle_thread, NULL);
	//pthread_create(&commandthread[1], NULL, change_detect_thread, NULL);
	pthread_create(&commandthread[2], NULL, movement_change_thread, NULL);

	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	sendPacket(&helloPacket);
        ok_flag = 1;

	while(!exitFlag)
	{
		char ch;
		switch (mode){
			case 0: endwin();
				start = 0;
				printf("Command (p=toggle to easy mode, f=forward, b=reverse, l=turn left, r=turn right, x=stop, c=clear stats, g=get stats q=exit)\n\r");
				printf("Easy Mode (w=forward, s=reverse, a=turn left, d=turn right, x = stop, c=clear stats, g=get stats, q=exit)\n\r");
				scanf("%c", &ch);
				flushInput();
				sendCommand(ch);
				break;

			case 1: clear();
				//printf("The final command is %c\n\r", finalcommand);
				if(!start){
					initscr();
					cbreak();
					noecho();
					nodelay(stdscr, TRUE);
					scrollok(stdscr, TRUE);
				}
				start = 1;

				if(j == 1){
					//printw("character of d is %c\n", d);
					//printw("Number of d is %d\n", d);
					command = (d == -1 || d == (char)255)? prevcommand:d;
					prevcommand = command;
					if(_count <= 1) usleep(400000);
					//printw("button\n");

				} 
				else if(i%3 == 0){
					command = 'x';
					prevcommand = command;
					//printw("end\n");

				}
				if(ok_flag) printw("Ready!\n");
				else printw("Busy!\n");
				printw("command is %c\n", finalcommand);

				if (kbhit()) {
					getch();
					i = 0;
					j = 1;
					_count++;
					refresh();
				} 
				
				else {
					_count = 0;
					i++;
					j = 0;
					refresh();
					usleep(70000);
				}
				break;
		}
	}
	// Purge extraneous characters from input stream


	endwin();
	printf("Closing connection to Arduino.\n\r");
	endSerial();
}
