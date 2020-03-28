
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

#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B9600

int exitFlag=0;
int mode = 0;
int ok_flag = 1;

char d = 'a';
char command = 'x';
char finalcommand = 'x';
int count = 0;

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

	if(ok_flag) serialWrite(buffer, len);
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
				ok_flag = 1;
				counter=0;
				handlePacket(&packet);
			}
			else 
				ok_flag = 0;
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
			commandPacket->params[1] = 50;
		break;
		case COMMAND_TURN_RIGHT: 
			commandPacket->params[0] = 0;
			commandPacket->params[1] = 50;
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
void* command_toggle_thread(void* p){
	while(1){
		if(count){
			command = d;
		}
		else command = 'x';
	}
}


void* change_detect_thread(void* p){
	clock_t time;
	int prev;
	int curr;
	int i = 0;
	while(1){
		if(count == 1){
			time = clock();
			prev = count;
			while(clock() - time < 9000000);
		}
		time = clock();
	
		while(clock() - time < 300000); //300000
		if(count == prev){
			command = 'x';
			count = 0;
		} 
	} 
	
}

void* movement_change_thread(void* p){
	char prev = 'x';	
	while(1){
		if(command != prev){
			if(command == 'x') printf("Easy Mode (w=forward, s=reverse, a=turn left, d=turn right, x = stop, c=clear stats, g=get stats, q=exit)\n\r");
			prev = command;
			finalcommand = command;
			sendCommand(finalcommand);
			printf("command is %c\n\r", finalcommand);
		}
	}
}
///////////////////////////////////////////////////////////////



int main()
{
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
    pthread_create(&commandthread[0], NULL, command_toggle_thread, NULL);
	pthread_create(&commandthread[1], NULL, change_detect_thread, NULL);
	pthread_create(&commandthread[2], NULL, movement_change_thread, NULL);

	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	sendPacket(&helloPacket);


	while(!exitFlag)
	{
		char ch;
		switch (mode){
			case 0: printf("Command (p=toggle to easy mode, f=forward, b=reverse, l=turn left, r=turn right, x=stop, c=clear stats, g=get stats q=exit)\n\r");
					printf("Easy Mode (w=forward, s=reverse, a=turn left, d=turn right, x = stop, c=clear stats, g=get stats, q=exit)\n\r");
					scanf("%c", &ch);
					flushInput();
					sendCommand(ch);
					break;

			case 1: clear();
					//printf("The final command is %c\n\r", finalcommand);
					
					initscr();
					cbreak();
					noecho();		
					d = getchar();
					count++;
					
					endwin();
					break;
		}
		// Purge extraneous characters from input stream
		
	}
	endwin();
	printf("Closing connection to Arduino.\n\r");
	endSerial();
}
