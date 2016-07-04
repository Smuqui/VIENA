/**
 * \mainpage
 * main.c
 *
 * Copyright 2016 Bruno Tibério <bruno@bruno-PC>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 *
 */

//#define _GNU_SOURCE

#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "gps.h"

#define PORT 8000

/** \fn char * createSocketServer(int port)
 *	\brief Create a berkley socket to accept start and stop orders
 *
 *  The function creates a socket server in order for daemonized process
 *  to accept the orders to start reading sensors or stop it.
 *
 *	\param port Listen port
 *	\return a char pointer with current working directory
 *
 */

char * createSocketServer(int port){

	char * workingDir;
	char *buffer=(char*)calloc(6,sizeof(char));
	int SocketFD = -1, nbytes = 1;
	bool sucess;
	struct sockaddr_in stSockAddr;
	Gps gps1;
	// Starting to create a berkley socket
	SocketFD = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(-1 == SocketFD){
		fprintf(stderr,"can not create socket\n");
		exit(EXIT_FAILURE);
	}

	memset(&stSockAddr, 0, sizeof(stSockAddr));

	stSockAddr.sin_family = AF_INET;
	stSockAddr.sin_port = htons(port);
	stSockAddr.sin_addr.s_addr = INADDR_ANY;

	if(-1 == bind(SocketFD,(struct sockaddr *)&stSockAddr, sizeof(stSockAddr))){
		fprintf(stderr,"error bind failed: %s:%d\n", __FILE__, __LINE__);
		close(SocketFD);
		exit(EXIT_FAILURE);
	}

	if(-1 == listen(SocketFD, 1)){
		fprintf(stderr,"error listen failed %s:%d\n", __FILE__, __LINE__);
		close(SocketFD);
		exit(EXIT_FAILURE);
	}
	// waiting for new connections
	for(;;){
		int ConnectFD = accept(SocketFD, NULL, NULL);

		if(0 > ConnectFD){
			fprintf(stderr,"error accept failed %s:%d\n", __FILE__, __LINE__);
			close(SocketFD);
			exit(EXIT_FAILURE);
		}
		nbytes=write(ConnectFD,"Welcome to server daemon\n",sizeof("Welcome to server daemon\n"));
		if(nbytes<=0){
			fprintf(stderr,"Error: not able to send message: %s:%d\n", __FILE__, __LINE__);
			exit(-1);
		}
		nbytes=read(ConnectFD,buffer,5);
		if(nbytes<0){
			fprintf(stderr,"Error: not able to receive message %s:%d\n", __FILE__, __LINE__);
			exit(-1);
		}
		if(strcmp(buffer,"start")==0){
			memset(buffer,'\0',5);
			write(ConnectFD,"initiating device... (could take 10sec)\n",
					sizeof("initiating device... (could take 10sec)\n"));
			sucess=gps1.begin();
			if(sucess){
				write(ConnectFD,"Sucess\n",sizeof("Sucess\n"));
				gps1.askLog(241);
			}else{
				write(ConnectFD,
					"Not able to communicate correctly with device\n",
					sizeof("Not able to communicate correctly with device\n"));
			}
		}else{
			if(strcmp(buffer,"stop\r")==0){
				memset(buffer,'\0',5);
				// clear all logs
				sucess=gps1.sendUnlogall();
				if(sucess){
					write(ConnectFD,"Sucess\n",sizeof("Sucess\n"));
				}else{
					write(ConnectFD,
							"Not able to communicate correctly with device\n",
							sizeof("Not able to communicate correctly with device\n"));
				}
				// close files and port
				fclose(gps1.getLogFd());
				close(gps1.getPortFd());
				nbytes=write(ConnectFD,"Exiting Server\n",sizeof("Exiting server\n"));
				sleep(1);
				shutdown(ConnectFD, SHUT_RDWR);
				close(ConnectFD);
				break;
			}else{
				nbytes = write(ConnectFD,
						"[Server]: Unkown Comand\n",
						sizeof("[Server]: Unkown Comand\n"));
				nbytes = write(ConnectFD,
						"[Server]: Valid options are:\nstart\nstop\n\n",
						sizeof("[Server]: Valid options are:\nstart\nstop\n\n"));
			}
		}
		shutdown(ConnectFD, SHUT_RDWR);
		close(ConnectFD);

	}
	close(SocketFD);

	return(workingDir);
}

/** \fn int main(int argc, char **argv)
 * \brief Creates a deamon process for headless server
 *
 * The objective is to create a daemon process in order to become a headless
 * server in linux.
 * After daemonize the program, a berckley TCP socket server is created to
 * accept start and stop commands.
 *
 * The creation of daemonized process was done following the tutorial in
 * <a href=http://www.netzmafia.de/skripten/unix/linux-daemon-howto.html>"Linux Daemon Writing HOWTO"</a>
 *
 * \todo Currently not handling any arguments
 *
 */

int main(int argc, char **argv)
{
	struct rlimit resourceLimit = { 0 };
	int status = -1;
	uint i=0;
	// Stores the current working directory
	char *wd=NULL;
	// stores the command to move output logs to current directory
	char *wd2=NULL;

	status = fork();
	switch (status){
	case -1:
		fprintf(stderr,"Error on first fork(): %s:%d\n", __FILE__, __LINE__);
		exit(1);
	case 0: /* child process */
		break;
	default: /* parent process */
		return(0);
	}

	/*
	 * New child process
	 */
	resourceLimit.rlim_max = 0;
	status = getrlimit(RLIMIT_NOFILE,
			&resourceLimit);
	if (-1 == status) /* shouldn't happen */
	{
		fprintf(stderr,"Error inside first child process: getrlimit(): %s:%d\n", __FILE__, __LINE__);
		exit(1);
	}
	if (0 == resourceLimit.rlim_max)
	{
		fprintf(stderr,"Max number of open file descriptors is 0!!!: %s:%d\n", __FILE__, __LINE__);
		exit(1);
	}

	for (i = 0; i < resourceLimit.rlim_max; i++)
	{
		(void) close(i);
	}
	status = setsid();
	if (-1 == status)
	{
		fprintf(stderr,"Error setting new ID: setsid(): %s:%d\n", __FILE__, __LINE__);
		exit(1);
	}

	// creating new child process

	status = fork();
	switch (status)
	{
	case -1:
		fprintf(stderr,"Error on second fork(): %s:%d\n", __FILE__, __LINE__);
		exit(1);
	case 0: /* (second) child process */
		break;
	default: /* parent process */
		return(0);
	}
	/*
	 * now we are in a new session and process
	 * group than process that started the
	 * daemon. We also have no controlling
	 * terminal */
	umask(0);
	if ( open("/dev/null", O_RDWR)==-1) /* stdin */
		fprintf(stderr,"Error setting stdin: %s:%d\n", __FILE__, __LINE__);

	freopen("stdout.log","w", stdout);  /*stdout*/
	freopen("stderr.log","w", stderr);  /* stderr */

	wd=(char*)createSocketServer(PORT);
	fcloseall();
	//	if(wd!=NULL){
	//		wd2=(char*)calloc(strlen("mv *.log ")+strlen(wd)+1,sizeof(char));
	//		wd2=strcpy(wd2,"mv *.log ");
	//		wd2=strcat(wd2,wd);
	//		system(wd2);
	//	}
	return(0);
}

