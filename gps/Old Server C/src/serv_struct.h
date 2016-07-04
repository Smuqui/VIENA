#include <semaphore.h>
#include <stdio.h>

typedef struct proc_inf {
  pid_t pid; /*pid of child process*/
  int com_f2c[2];/*com pipe from father to child process*/
  int com_c2f[2];/*com pipe from child to father process*/
  sem_t control; /*semaphore for control*/
}proc_info;

typedef struct status_inf{
  char *work_dir;
  int camera;
}status_info;

FILE * logFile;

status_info * start(proc_info*, status_info *);
void process_data(char * , FILE *);
