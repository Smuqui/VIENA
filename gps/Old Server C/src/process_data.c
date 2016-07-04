#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>


float time_start;
/***********************************************************************
 * A mensagem enviada do gps é subdividida nos vários campos e são 
 * retirados aqueles que têm interesse. 
 **********************************************************************/
void process_data(char * buffer, FILE *fd_pos){
	char *aux;
	char *pch;
	char *aux_buffer;
	int i=0;
	int obs_status=0;
	float aux_time=0;

	aux_buffer =(char *)calloc(strlen(buffer)+1,sizeof(char));
	strcpy(aux_buffer,buffer);
	aux=strstr(buffer,"<BESTXYZ");/*verifica se o cabeçalho é o esperado*/

	if(aux!=NULL){
		/*--------------------------------------------------------------
		 *  log bestxyz message
		 * -----------------------------------------------------------*/
		pch = strtok (aux_buffer," \n");
		while (pch != NULL){
			if (i==6){
				if(time_start==0)
					time_start=atof(pch);
				aux_time=atof(pch)-time_start;
				fprintf(fd_pos,"%.1f;",aux_time);
			}

			if (i==13||i==14||i==15||i==16||i==17||i==18||i==21||i==22||i==23||i==24||i==25||i==26)
				fprintf(fd_pos,"%s;",pch);
			if(i==11||i==19){
				obs_status=strcmp(pch,"INSUFFICIENT_OBS");
				if(obs_status==0){
					fprintf(fd_pos,"0;");
				}else{
					fprintf(fd_pos,"1;");
				}
			}
			if(i==28) /*último campo leva \n para passar para a linha seguinte*/
				fprintf(fd_pos,"%s\n",pch);
			pch = strtok (NULL," \n");
			i++;
		}


	}


	free(aux_buffer);
	return;
}

