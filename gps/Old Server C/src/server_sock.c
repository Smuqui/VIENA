#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>
#include "serv_struct.h"


int camera_activated=0;

char * serv_sock(void){


	proc_info *gps1=NULL;
	status_info *info=NULL;
	struct sockaddr_in stSockAddr;
	int SocketFD = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	char *buffer=calloc(6,sizeof(char));
	int nbytes=0;
	int close_prog=1;
	int status;
	logFile = fopen("stdlog.log","w");

	if(-1 == SocketFD){
		perror("can not create socket");
		exit(EXIT_FAILURE);
	}

	memset(&stSockAddr, 0, sizeof(stSockAddr));

	stSockAddr.sin_family = AF_INET;
	stSockAddr.sin_port = htons(8000);
	stSockAddr.sin_addr.s_addr = INADDR_ANY;

	if(-1 == bind(SocketFD,(struct sockaddr *)&stSockAddr, sizeof(stSockAddr))){
		perror("error bind failed");
		close(SocketFD);
		exit(EXIT_FAILURE);
	}

	if(-1 == listen(SocketFD, 1)){
		perror("error listen failed");
		close(SocketFD);
		exit(EXIT_FAILURE);
	}

	for(;;){
		int ConnectFD = accept(SocketFD, NULL, NULL);

		if(0 > ConnectFD){
			perror("error accept failed");
			close(SocketFD);
			exit(EXIT_FAILURE);
		}
		nbytes=write(ConnectFD,"Welcome to server daemon\n",sizeof("Welcome to server daemon\n"));
		if(nbytes<=0){
			fprintf(logFile,"Error: not able to send message: server_sock.c:59\n");
			exit(-1);
		}
		nbytes=read(ConnectFD,buffer,5);
		if(nbytes<0){
			fprintf(logFile,"Error: not able to receive message: server_sock.c:61\n");
			exit(-1);
		}
		if(strcmp(buffer,"start")==0){
			memset(buffer,'\0',5);
			info=(status_info *) calloc(1,sizeof(status_info));
			gps1=(proc_info*) calloc(1,sizeof(proc_info)); // gps1 process info
			info=start(gps1,info);
		}else{
			if(strcmp(buffer,"stop\r")==0){
				memset(buffer,'\0',5);
				write(gps1->com_f2c[1],&close_prog,sizeof(int));
				waitpid(gps1->pid,&status,0);
				fprintf(logFile,"Process gps1 terminated. Exited status:%d\n",status);
				fflush(logFile);
				close(gps1->com_f2c[0]);
				close(gps1->com_f2c[1]);

				nbytes=write(ConnectFD,"Exiting Server\n",sizeof("Exiting server\n"));
				sleep(1);
				shutdown(ConnectFD, SHUT_RDWR);
				close(ConnectFD);
				break;
			}else{
				nbytes=write(ConnectFD,"Unkown Comand\n",sizeof("Unkown Comand\n"));
			}
		}

		shutdown(ConnectFD, SHUT_RDWR);
		close(ConnectFD);
	}
	close(SocketFD);
	return (info->work_dir);
}
