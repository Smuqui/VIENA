/*
  Para compilar: gcc *.c -o <nome> -g -ggdb `pkg-config opencv --cflags --libs`-std=c99 -ldl -lrt -lm -pthread -Wall -pedantic
*/
#define _GNU_SOURCE
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>



char* serv_sock(void);

int main(int argc, char **argv)
{
  struct rlimit resourceLimit = { 0 };
  int status = -1;
  int i=0;
  char *wd=NULL;
  char *wd2=NULL;
    
  
  status = fork();
  switch (status)
    {
    case -1:
      perror("fork()");
      exit(1);
    case 0: /* child process */
      break;
    default: /* parent process */
      exit(0);
    }
  /*
   * child process
   */
  resourceLimit.rlim_max = 0;
  status = getrlimit(RLIMIT_NOFILE,
		     &resourceLimit);
  if (-1 == status) /* shouldn't happen */
    {
      perror("getrlimit()");
      exit(1);
    }
  if (0 == resourceLimit.rlim_max)
    {
      printf("Max number of open file descriptors is 0!!\n");
      exit(1);
    }

  for (i = 0; i < resourceLimit.rlim_max; i++)
    {
      (void) close(i);
    }
  status = setsid();
  if (-1 == status)
    {
      perror("setsid()");
      exit(1);
    }
  status = fork();
  switch (status)
    {
    case -1:
      perror("fork()");
      exit(1);
    case 0: /* (second) child process */
      break;
    default: /* parent process */
      exit(0);
    }
  /*
   * now we are in a new session and process
   * group than process that started the
   * daemon. We also have no controlling
   * terminal */
  umask(0);
  if ( open("/dev/null", O_RDWR)==-1) /* stdin */
    perror("Error setting stdin: server_daemon.c:85\n");

  //freopen("stdout.log","w", stdout);	/*stdout*/
  //freopen("stderr.log","w", stderr);  /* stderr */
    
  wd=serv_sock();
  fcloseall();
  if(wd!=NULL){
    wd2=(char*)calloc(strlen("mv *.log ")+strlen(wd)+1,sizeof(char));
    wd2=strcpy(wd2,"mv *.log ");
    wd2=strcat(wd2,wd);
    system(wd2);
  }
  return (0);
}
