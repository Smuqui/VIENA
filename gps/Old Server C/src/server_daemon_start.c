#define _XOPEN_SOURCE 500   /* to make use of usleep which is considered obsulete*/
#define _SVID_SOURCE
#define GPS_NUMBER 1
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include <unistd.h> /* UNIX standard function definitions*/
#include <fcntl.h> /*File control definitions*/
#include <errno.h> /*Error number definitions*/
#include <termios.h> /*POSIX terminal control definitionss*/
#include <time.h>	 /*time calls*/
#include <sys/select.h>
#include <semaphore.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <math.h>
#include "serv_struct.h"


int control_loop;


void *control_wait(void * prog){
  /*******************************************************************
   * Thread to control the exit of subprocesses when the user gives the
   * order to do so.
   * It will alaways be at no activity until the order is given
   * ****************************************************************/
  int i;
  proc_info *prog_aux=(proc_info*)prog;
  read(prog_aux->com_f2c[0],&i,sizeof(int));
  sem_wait(&prog_aux->control);
  control_loop=1;
  sem_post(&prog_aux->control);
  return NULL;
}

void gps (proc_info *prog, char * dir){
  /*********************************************************************
   * Adquires velocity and position information from the gps system
   * HW info:
   * Receptor - Novatel Flexpak G2L-3151W
   * Antenna - Novatel Pinwheel 
   *********************************************************************/
  
  
  
  FILE *fd_log, *fd_pos, *fd_pos1;
  pthread_t control_thread;
  int i=1;
  int fd=0;
  int fd1=0;
  int n_bytes=-2;
  int flags=0;
  int first_time=0;
  int debug=0;
  char aux;
  struct termios options;
  struct timespec ti;
  char *msg_unlogall=(char*)calloc(sizeof("unlogall thisport\r\n"),sizeof(char));
  char portname[]="/dev/ttyUSB0\0";
  char portname1[]="/dev/ttyUSB1\0";
  
  char *buffer_read=(char *)calloc(512,sizeof(char));
  char *buffer_read1=(char *)calloc(512,sizeof(char));
  
  chdir(dir);
  
  if(sem_init(&prog->control,0,1)!=0){
    fprintf(logFile,"@GPS:Erro a iniciar semáforo\n");
    exit(1);
  }
  
  /*********************************************************************
   * Configuration of the port to communicate with the GPS1 receptor
   * ******************************************************************/	
  
  strcpy(msg_unlogall,"unlogall\r\n");
  
  fd=open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0) {
    fprintf(logFile,"@GPS:Não abriu porta série\n");
    exit(-1); 
  }      
  flags=fcntl(fd,F_GETFL);
  flags |= O_NONBLOCK;
  
  fcntl(fd, F_SETFL, flags); /*read is not blocked */
  
  
  tcgetattr(fd,&options);
  /*********************************************************************
   *  set baud rate 9600
   ********************************************************************/
  cfsetispeed(&options, B9600); 
  cfsetospeed(&options, B9600);
  
  
  /*********************************************************************
   *  set no parity, 8 data bits, 1 stop bit
   ********************************************************************/
  options.c_cflag &= ~PARENB; 
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;	
  options.c_cflag |= CS8;
  options.c_lflag &=~(ICANON | ECHO | ECHOE);
  tcsetattr(fd,TCSANOW,&options);
  
  fprintf(logFile,"sending sync sequence GPS1\n");
  fflush(logFile);
  write(fd,"\0",1);
  usleep(1E4);
  write(fd,"\0",1);
  usleep(1E4);
  write(fd,"\r",1);
  usleep(1E4);
  write(fd,"\n",1);
  usleep(2E4);
  
  write(fd,msg_unlogall,strlen(msg_unlogall));
  sleep(1);
  
  n_bytes=read(fd,buffer_read,511);
  if(debug){
    fprintf(logFile,"DEBUG:server_daemon.c:215 - %s\n",buffer_read);
  }
  
  if (n_bytes<1){
    fprintf(logFile,"waiting GPS1 boot time\n");
    fflush(logFile);
    sleep(10);
  }		
  fflush(logFile);
  write(fd,"\0",1);
  usleep(1E4);
  write(fd,"\0",1);
  usleep(1E4);
  write(fd,"\r",1);
  usleep(1E4);
  write(fd,"\n",1);
  usleep(2E4);    
  
  fprintf(logFile,"....ready\n");
  fflush(logFile);
  
  
  write(fd,msg_unlogall,strlen(msg_unlogall));
  
  /*********************************************************************
   * 
   *  change the baudrate
   * 
   ********************************************************************/
  
  usleep(1E4);
  write(fd,"com com1 115200 N 8 1 N\r\n",strlen("com com1 115200 N 8 1 N\r\n"));
  usleep(1E4);
  
  
  if(ioctl( fd, TCFLSH, TCIOFLUSH)==0)
	  fprintf(logFile,"buffers cleared GPS1\n");
  
  close(fd);
  
  fprintf(logFile,"changing baudrate to 115200\n");
  
  fd=open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0) {
    fprintf(logFile,"@GPS1: Não abriu porta série\n");
    exit(-1); 
  }      
  
  
  
  tcgetattr(fd,&options);
  
  /*********************************************************************
   *  set baud rates 115200
   ********************************************************************/
  cfsetispeed(&options, B115200); 
  cfsetospeed(&options, B115200);
  
  /*********************************************************************
   *  set no parity, 8 data bits, 1 stop bit
   ********************************************************************/
  options.c_cflag &= ~PARENB; 
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;	
  options.c_cflag |= CS8;
  
  
  options.c_lflag &=~(ICANON | ECHO | ECHOE);
  
  
  tcsetattr(fd,TCSANOW,&options);
  
  fcntl(fd, F_SETFL, 0); /*read is blocked till get data*/
  
  fprintf(logFile,"GPS1: done\n\n");
  fflush(logFile);
  /*********************************************************************
   *  End of port configuration for GPS1.
   *  Start for GPS2
   *********************************************************************/
  if(GPS_NUMBER==2){
    fd1=open(portname1, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd1 < 0) {
      fprintf(logFile,"@GPS2:Não abriu porta série\n");
      exit(-1); 
    }      
    flags=fcntl(fd1,F_GETFL);
    flags |= O_NONBLOCK;
    
    fcntl(fd1, F_SETFL, flags); /*read is not blocked */
    
    
    tcgetattr(fd1,&options);
    /*********************************************************************
     *  set baud rate 9600
     ********************************************************************/
    cfsetispeed(&options, B9600); 
    cfsetospeed(&options, B9600);
    
    
    /*********************************************************************
     *  set no parity, 8 data bits, 1 stop bit
     ********************************************************************/
    options.c_cflag &= ~PARENB; 
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;	
    options.c_cflag |= CS8;
    options.c_lflag &=~(ICANON | ECHO | ECHOE);
    tcsetattr(fd1,TCSANOW,&options);
    
    fprintf(logFile,"sending sync sequence GPS2\n");
    fflush(logFile);
    write(fd1,"\0",1);
    usleep(1E4);
    write(fd1,"\0",1);
    usleep(1E4);
    write(fd1,"\r",1);
    usleep(1E4);
    write(fd1,"\n",1);
    usleep(2E4);
    
    write(fd1,msg_unlogall,strlen(msg_unlogall));
    sleep(1);
    
    n_bytes=read(fd1,buffer_read,511);
    if(debug){
      fprintf(logFile,"DEBUG:server_daemon.c:215 - %s\n",buffer_read);
    }
    
    if (n_bytes<1){
      fprintf(logFile,"waiting gps2 boot time\n");
      fflush(logFile);
      sleep(10);
    }		
    fflush(logFile);
    write(fd1,"\0",1);
    usleep(1E4);
    write(fd1,"\0",1);
    usleep(1E4);
    write(fd1,"\r",1);
    usleep(1E4);
    write(fd1,"\n",1);
    usleep(2E4);    
    
    fprintf(logFile,"....ready\n");
    fflush(logFile);
    
    
    write(fd1,msg_unlogall,strlen(msg_unlogall));
    
    /*********************************************************************
     * 
     *  change the baudrate
     * 
     ********************************************************************/
    
    usleep(1E4);
    write(fd1,"com com1 115200 N 8 1 N\r\n",strlen("com com1 115200 N 8 1 N\r\n"));
    usleep(1E4);
    
    
    if(ioctl( fd1, TCFLSH, TCIOFLUSH)==0)
    	fprintf(logFile,"buffers cleared\n");
    
    close(fd1);
    
    fprintf(logFile,"changing baudrate to 115200\n");
    
    fd1=open(portname1, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd1 < 0) {
      fprintf(logFile,"@GPS2: Não abriu porta série\n");
      exit(-1); 
    }      
    
    
    
    tcgetattr(fd1,&options);
    
    /*********************************************************************
     *  set baud rates 115200
     ********************************************************************/
    cfsetispeed(&options, B115200); 
    cfsetospeed(&options, B115200);
    
    /*********************************************************************
     *  set no parity, 8 data bits, 1 stop bit
     ********************************************************************/
    options.c_cflag &= ~PARENB; 
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;	
    options.c_cflag |= CS8;
    
    
    options.c_lflag &=~(ICANON | ECHO | ECHOE);
    
    
    tcsetattr(fd1,TCSANOW,&options);
    
    fcntl(fd1, F_SETFL, 0); /*read is blocked till get data*/
    
    fprintf(logFile,"GPS2: done\n\n");
    fflush(logFile);
    
    /*********************************************************************
     *  End of port configuration GPS2
     ********************************************************************/
    
  }
  /*********************************************************************
   *  use log file for debug?
   ********************************************************************/
  if(debug){
    fd_log=fopen("data_log.txt","w");
    if(fd_log==NULL){
      fprintf(logFile,"erro a abrir data_log\n");
    }
  }
  
  /*********************************************************************
   *  open log files GPS1
   ********************************************************************/
  
  fd_pos=fopen("pos_log1.txt","w+");
  if(fd_pos==NULL){
    fprintf(logFile,"erro a abrir pos_log\n");
    exit(-1);
  }
  fprintf(fd_pos,"Time;SolStatusP;X;Y;Z;stdX;stdY;stdZ;SolStatusV;Vx;Vy;Vz;stdVx;stdVy;stdVz;Vlatency\n");
  
  fflush(fd_pos);
  if(GPS_NUMBER==2){
    /*********************************************************************
     *  open log files GPS2
     ********************************************************************/
    fd_pos1=fopen("pos_log2.txt","w+");
    if(fd_pos1==NULL){
      fprintf(logFile,"erro a abrir pos_log\n");
      exit(-1);
    }
    fprintf(fd_pos1,"Time;SolStatusP;X;Y;Z;stdX;stdY;stdZ;SolStatusV;Vx;Vy;Vz;stdVx;stdVy;stdVz;Vlatency\n");
    
    fflush(fd_pos1);
  }
  
  pthread_create(&control_thread,NULL,control_wait,(void*)prog);
  /*********************************************************************
   *  ask for data GPS1
   ********************************************************************/
  write(fd,"log com1 bestxyz ontime 0.1\r\n",strlen("log com1 bestxyz ontime 0.1\r\n"));
  usleep(1E4);
  
  if(GPS_NUMBER==2){
    /*********************************************************************
     *  ask for data GPS2
     ********************************************************************/
    write(fd1,"log com1 bestxyz ontime 0.1\r\n",strlen("log com1 bestxyz ontime 0.1\r\n"));
    usleep(1E4);
  }
  
  while(1){
    sem_wait(&prog->control);
    if(control_loop){
      pthread_join(control_thread,NULL);
      write(fd,"unlogall thisport\r\n",sizeof("unlogall thisport\r\n"));
      sleep(1);
      write(fd,"com com1 9600 N 8 1 N\r\n",strlen("com com1 9600 N 8 1 N\r\n"));
      usleep(1);
      if(ioctl(fd, TCFLSH, TCIOFLUSH)==0)
	fprintf(logFile,"\nbuffers GPS1 cleared\n");
      if(GPS_NUMBER==2){
	write(fd1,"unlogall thisport\r\n",sizeof("unlogall thisport\r\n"));
	sleep(1);
	write(fd1,"com com1 9600 N 8 1 N\r\n",strlen("com com1 9600 N 8 1 N\r\n"));
	usleep(1);
	if(ioctl(fd1, TCFLSH, TCIOFLUSH)==0)
	  fprintf(logFile,"\nbuffers GPS2 cleared\n");
      }
      break;
    }
    /*********************************************************************
     *  Get data GPS1
     ********************************************************************/
    i=0;
    n_bytes=-2;
    while(1){
      n_bytes=read(fd,&aux,1);
      if (n_bytes>0){
	buffer_read[i]=aux;
	i++;
	if(aux==']')
	  break;
      }else{
	if(n_bytes==-1)
	  fprintf(logFile,"@GPS1: Erro na leitura da porta: n_bytes=-1\n");
      }
    }
    if(GPS_NUMBER==2){
      /*********************************************************************
       *  Get data GPS2
       ********************************************************************/
      i=0;
      n_bytes=-2;
      while(1){
	n_bytes=read(fd1,&aux,1);
	if (n_bytes>0){
	  buffer_read1[i]=aux;
	  i++;
	  if(aux==']')
	    break;
	}else{
	  if(n_bytes==-1)
	    fprintf(logFile,"@GPS2: Erro na leitura da porta: n_bytes=-1\n");
	}
      }
    }
    if(strlen(buffer_read)<20){
      fprintf(logFile,"%s\n",buffer_read);
    }else{
      if(first_time==0){
	first_time=1;
	clock_gettime(CLOCK_MONOTONIC,&ti);
	write(prog->com_c2f[1],&ti,sizeof(ti));
      }
    
      process_data(buffer_read,fd_pos);
      if(debug){
	fprintf(fd_log,"resposta:%s",buffer_read);
      }
      fflush(logFile);
    }
    if(GPS_NUMBER==2){
      if(strlen(buffer_read1)<20){
	fprintf(logFile,"%s\n",buffer_read);
      }else{
	process_data(buffer_read1,fd_pos1);
	if(debug){
	  fprintf(fd_log,"resposta:%s",buffer_read1);
	}
	fflush(logFile);
      }
    }		    
    
    memset(buffer_read,'\0',strlen(buffer_read));
    if(GPS_NUMBER==2){
      memset(buffer_read1,'\0',strlen(buffer_read1));
    }
    sem_post(&prog->control);
  }
  
  sem_post(&prog->control);
  fclose(fd_pos);
  if(GPS_NUMBER==2)
    fclose(fd_pos1);
  if(debug)
    fclose(fd_log);
  close(fd);
  if(GPS_NUMBER==2)
    close(fd1);
  
  return;
}



status_info* start( proc_info* prog1, status_info* inf){
  FILE *fd_time;
  
  
  struct timespec t1;
  double  delay_gps1;
  
  struct dirent **namelist;
  int n,status=0;
  int retval=-1;
  char *dirname=calloc(sizeof("./data/testxx/"),sizeof(char));
  char *aux_dir=calloc(sizeof("./data/testxx/")+15,sizeof(char));
  char *tmp=calloc(4,sizeof(char));

  /* create or verify if data dir exists*/

  n = scandir(".", &namelist, 0, alphasort);
  if (n < 0)
    perror("scandir error occurred");
  else {
    while (n--) {
      if(strcmp("data", namelist[n]->d_name)==0)
	retval=1;
      free(namelist[n]);
    }
  }
  if(retval<0){
    retval=mkdir("./data/",S_IRWXU|S_IRWXG);
    if (retval<0){
      fprintf(logFile,"Erro na criação de directorio");
      exit(1);
    }
  }
  /* create one or verify what is the last test data folder*/

  retval=-1;
  n=scandir("./data/",&namelist,0,alphasort);
  if (n < 0)
    perror("scandir error occurred");
  else {
    while (n--) {
      if(retval<0)
	if(strstr(namelist[n]->d_name,"test")!=NULL){
	  sscanf(namelist[n]->d_name,"test%d",&retval);
	  sprintf(tmp,"%.2d/",retval+1);
	  dirname=strcat(dirname,"./data/test");
	  dirname=strcat(dirname,tmp);
	  retval=mkdir(dirname,S_IRWXU|S_IRWXG);
	  if (retval<0){
	    fprintf(logFile,"Erro na criação de directorio:%s",dirname);
	    exit(1);
	  }
	}
      free(namelist[n]);
    }
  }
  if(retval<0){
    retval=mkdir("./data/test00",S_IRWXU|S_IRWXG);
    strcpy(dirname,"./data/test00/");
    if (retval<0){
      fprintf(logFile,"Erro na criação de directorio");
      exit(1);
    }
  }
  /*********************************************************************
   *  End of directory creation/verification			       *
   * ******************************************************************/
  inf->work_dir=(char *) calloc (strlen(dirname)+1,sizeof(char));
  strcpy(inf->work_dir,dirname);
  
  aux_dir=strcat(aux_dir,dirname);
  aux_dir=strcat(aux_dir,"delay.txt");
  fd_time=fopen(aux_dir,"w+");
  
 
  if(pipe(prog1->com_f2c)!=0||(pipe(prog1->com_c2f)!=0)){
    fprintf(stderr, "Erro na criação de pipe de comunicação de GPS1\n");
    exit(-1);
  }
  
  if((prog1->pid=fork())==-1){
    fprintf(logFile, "Erro na criação de processos: GPS1\n");
    exit(-1);
  }
  if(prog1->pid==0){
    close(prog1->com_f2c[1]);
    close(prog1->com_c2f[0]);
    gps(prog1,dirname);
    exit(0);
  }else{
    close(prog1->com_f2c[0]);
    close(prog1->com_c2f[1]);
    read(prog1->com_c2f[0],&t1,sizeof(t1));
    delay_gps1=t1.tv_sec+t1.tv_nsec*1E-9;
    fprintf(fd_time,"GPS1:%lf\n",delay_gps1);
    
  }
   
  return (inf);
  
}
