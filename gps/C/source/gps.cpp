/**
 * \file
 * \brief contains the implementation of class definitions for gps()
 *
 * \author Bruno Tibério
 * \date 2016
 * \todo Implent all possible commands
 *
 */

#include "gps.h"

#define CRC32_POLYNOMIAL 0xEDB88320L
#define NUM_TRIES 3
#define DATA_HEADER "Indice,Time,PSolStatus,X,Y,Z,stdX,stdY,stdZ,\
	VSolStatus,VX,VY,VZ,stdVX,stdVY,stdVZ,VLatency,SolAge,SolSatNumber\n"
/**
 * \brief Calculate a CRC value to be used by CRC calculation functions.
 *
 * extracted from device firmware manual
 */
unsigned int CRC32Value(int i)
{
	int j;
	unsigned int ulCRC;
	ulCRC = i;
	for ( j = 8 ; j > 0; j-- )
	{
		if ( ulCRC & 1 )
			ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}
	return (ulCRC);
}
/**
 * \brief Calculates the CRC-32 of a block of data all at once
 *
 * extracted from device firmware manual
 */
unsigned int CalculateBlockCRC32(
		unsigned int ulCount, /* Number of bytes in the data block */
		unsigned char *ucBuffer ) /* Data block */
{
	unsigned int ulTemp1;
	unsigned int ulTemp2;
	unsigned int ulCRC = 0;
	while ( ulCount-- != 0 )
	{
		ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return( ulCRC );
}



Gps::Gps() {
	this->_comPort = 0;
	this->isOpen = 0;
	this->_baudRate = 9600;
	this->serialPortFD = -1;
	this->openError = false;
	this->errorFlags = 0;
	this->logFD = NULL;
	this->logData = NULL;
	this->sizeHeader = sizeof(struct _header);
	memset(&this->defaultHeader,0,this->sizeHeader);
	this->defaultHeader.sync[0] = 0xAA;
	this->defaultHeader.sync[1] = 0x44;
	this->defaultHeader.sync[2] = 0x12; // always
	this->defaultHeader.headerLength = 0x1C; // 28 bytes expected
	this->defaultHeader.messageType = 0x02; //original message and binary
	memcpy(&this->currentHeader, &this->defaultHeader,this->sizeHeader);
	this->exitOrder = 0;
	this->threadID = 0;
	pthread_cond_init(&this->gotUnlogResponce, NULL);
	pthread_cond_init(&this->gotLogResponce, NULL);
	this->logRequestStatus = 0;
	this->unlogallRequestStatus = 0;
	this->comRequestStatus = 0;
	this->CurrentRecord = 0;


}


bool Gps::begin(const char * comPort, const char *log, const char *data,const int baudRate){
	this->_comPort = comPort;
	struct termios options;
	int tries=0, bytes_available=0;
	struct timespec tPause;
	speed_t baud;
	bool sucess = 0;

	this->logFD=fopen(log, "w");
	if(this->logFD < 0){
		this->openError = true;
		this->errorFlags |= LOGERROR;
		return (false);
	}
	this->logData=fopen(data, "w");
	if(this->logData < 0){
		this->openError = true;
		this->errorFlags |= LOGERROR;
		return (false);
	}
	fprintf(this->logData,DATA_HEADER);
	fflush(this->logData);
	baud = parseBaudRate(baudRate);


	/*********************************************************************
	 * Configuration of the port to communicate with the GPS receptor
	 ********************************************************************/

	this->serialPortFD=open(comPort, O_RDWR | O_NOCTTY | O_NDELAY);
	if (this->serialPortFD < 0) {
		fprintf(this->logFD,"@GPS:Não abriu porta série\n");
		this->openError = true;
		this->errorFlags |= PORTERROR;
		return(false);
	}
	this->isOpen = true;

	tcgetattr(this->serialPortFD,&options);
	/*********************************************************************
	 *  set baud rate 9600 default GPS value
	 ********************************************************************/
	cfsetispeed(&options, baud);
	cfsetospeed(&options, baud);
	/*********************************************************************
	 *  set no parity, 8 data bits, 1 stop bit
	 ********************************************************************/
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_lflag &=~(ICANON | ECHO | ECHOE |ISIG);
	tcsetattr(this->serialPortFD,TCSANOW,&options);
	tPause.tv_sec = 1;
	tPause.tv_nsec = 0.5*1E9;
	//reset port to default config using 2 breaks and pauses
	//1 break, 1.5 sec pause, 1 break and at least 0.25sec pause
	tcsendbreak(this->serialPortFD,0);
	while(nanosleep(&tPause,NULL)!= 0);
	tcsendbreak(this->serialPortFD,0);
	sleep(2); //required to make flush work, for some reason
	tcflush(this->serialPortFD,TCIOFLUSH); // clear buffer
	fcntl(this->serialPortFD, F_SETFL, 0); // block until data comes in
	pthread_mutex_init(&this->lock, NULL);
	pthread_create(&this->threadID, NULL, &Gps::Help_processResponses, this);
	do{
		sucess = sendUnlogall();
		if (!sucess){
			fprintf(this->logFD,".");
			fflush(this->logFD);
			tries++;
			sleep(2);
		}
	}while(sucess == false && tries < NUM_TRIES);

	if( tries == NUM_TRIES){
		fprintf(this->logFD,
				"Unable to communicate with device after %d tries\n", tries);
		fprintf(this->logFD,"Current bytes available: %d\n", bytes_available);
		return(false);
	}
	fprintf(this->logFD,"done\n\n");
//	sleep(2); //required to make flush work, for some reason
//	tcflush(this->serialPortFD,TCIOFLUSH); // clear buffer
	this->_baudRate = baud;
	fprintf(this->logFD,"....ready\n");
	fflush(this->logFD);
	return (sucess);
}

/**
 *
 * checks if the desired baudrate value is valid and accepted by the gps device.
 * If the value is not valid, B9600 is returned.
 *
 * The valid values are:
 *
 * |Baud  |Note                 |
 * |:----:|:--------------------|
 * |300	  |                     |
 * |600   |						|
 * |900   |	Tag B900 is not defined in linux termios.h. Using B9600 instead.
 * |1200  |						|
 * |2400  |						|
 * |4800  |						|
 * |9600  |						|
 * |19200 |						|
 * |38400 |						|
 * |57600 |						|
 * |115200|						|
 * |230400| Tag B230400 is not defined in linux termios.h. Using B9600 instead.
 *
 * \param baud the desired baudrate value
 * \return the corresponding macro for the given input.
 */
speed_t Gps::parseBaudRate(const int baud){
	switch(baud){
	case 300: return (B300);
	case 600: return (B600);
	case 900:
		printf("BaudRate tag B900 is not defined in termios.h\nUsing B9600 instead\n");
		return(B9600);
	case 1200: return(B1200);
	case 2400: return(B2400);
	case 4800: return(B4800);
	case 9600: return(B9600);
	case 19200: return(B19200);
	case 38400: return(B38400);
	case 57600: return(B57600);
	case 115200: return(B115200);
	case 230400:
		printf("BaudRate tag B230400 is not defined in termios.h\nUsing B9600 instead\n");
		return(B9600);
	default:
		return(B9600);
	}
}

/**
 *
 * send command unlogall to gps device. Message is sent in binary mode
 * On sucess clears all logs on all ports even held logs
 *
	unlogall message is defined as:

	|Field| ID         | N  Bytes | Description           |
	|:---:|:----------:|:--------:|:---------------------:|
	|1    | header     | H = 28   | Header of message     |
	|2    | port       | ENUM = 4 | identification of port|
	|3    | Held       | ENUM = 4 | can only be 0 or 1. Clear logs with hold flag or not?|
	|CRC32|            | UL = 4   | calculated crc32 value over all message|

 * See: OEMStar Firmware Reference Manual Rev 6 page 161
 */

bool Gps::sendUnlogall(){

	// check if port is open first
	if(!this->isOpen){
		fprintf(this->logFD, "Device port is not open:%s\n", this->_comPort);
		return(false);
	}

	// Variables
	GPSheader *myHeader = & this->currentHeader;
	struct _unlogall buffer;
	int n_bytes = sizeof(struct _unlogall);
	int returnVal = 0;

	// set options on header for unlogall command
	myHeader->messageID = UNLOGALL; //unlogall messageID
	myHeader->messageLength = 8; //message length in bytes;
	myHeader->portAddress = THISPORT; //thisport

	// clean memory
	memset(&buffer,0,n_bytes);
	// copy header
	memcpy(&buffer.header,myHeader,this->sizeHeader);
	// set options for message body
	buffer.port = ALL_PORTS; //allports
	buffer.held = 1; // remove hold logs
	// calculate CRC32 value
	buffer.crc32=CalculateBlockCRC32(28+8, (unsigned char*) &buffer);

#ifdef DEBUG
	DEBUG_MSG("DEBUG: Sending unlogall: ");
	unsigned char *aux = (unsigned char*) &buffer;
	for(int i = 0; i<40; i++)
		DEBUG_MSG("%02x ", aux[i]);
	DEBUG_MSG("\n");
	fflush(this->logFD);
#endif
	pthread_mutex_lock(&this->lock);
	// send to device
	if(write(this->serialPortFD,&buffer,n_bytes)!=n_bytes){
		fprintf(this->logFD,
				"Number of bytes writen to device  not as expected:\n"
				"%s:%d\n", __FILE__, __LINE__);
		fflush(this->logFD);
		//pthread_mutex_unlock(&lock);
		return(false);
	}

	pthread_cond_wait(&this->gotUnlogResponce,&this->lock );
	// wait for log response
	returnVal = this->unlogallRequestStatus;
	pthread_mutex_unlock(&this->lock);
	if(returnVal == OK) //ok
		return(true);
	return (false);
}

/**
 *
 * The com request command is defined as:
 *
 *|Field| ID         | N  Bytes | Description           		   |
 *|:---:|:----------:|:--------:|:---------------------------------|
 *|1    | Com header | H = 28   | Header of message     		   |
 *|2    | port       | ENUM = 4 | identification of port		   |
 *|3    | baud		 | Ulong = 4| Communication baud rate (bps)	   |
 *|4    | parity	 | ENUM = 4 | Parity 						   |
 *|5	| databits	 | Ulong = 4| Number of data bits (default = 8)|
 *|6	| stopbits	 | Ulong = 4| Number of stop bits (default = 1)|
 *|7	| handshake	 | ENUM = 4 | Handshaking					   |
 *|8	| echo		 | ENUM = 4 | No echo (default)(must be 0 or 1)|
 *|9	| break		 | ENUM = 4 | Enable break detection (default),(must be 0 or 1)|
 *
 * Total byte size = header + 32 = 60 bytes
 *
 * COM Serial Port Identifiers (field 2):
 *
 * 	|Binary |ASCII		|Description			|
 * 	|:-----:|:---------:|:----------------------|
 * 	|1		| COM1 		|COM port 1				|
 * 	|2 		| COM2 		|COM port 2				|
 * 	|6		| THISPORT	|The current COM port	|
 * 	|8		| ALL		|All COM ports			|
 * 	|9		| XCOM1		|Virtual COM1 port		|
 * 	|10		| XCOM2		|Virtual COM2 port		|
 * 	|13		| USB1		|USB port 1				|
 * 	|14		| USB2 		|USB port 2				|
 * 	|15		| USB3 		|USB port 3				|
 *	|17		| XCOM3		|Virtual COM3 port		|
 *
 *	Parity(field 4):
 *
 *	|Binary |ASCII		|Description			|
 * 	|:-----:|:---------:|:----------------------|
 * 	|0		| N			| No parity (default)	|
 * 	|1		| E			| Even parity			|
 * 	|2		| O			| Odd parity			|
 *
 * 	Handshaking (field 7):
 *
 * 	|Binary |ASCII		|Description					|
 * 	|:-----:|:---------:|:------------------------------|
 * 	|0		|N			|No handshaking (default)		|
 * 	|1		|XON		|XON/XOFF software handshaking	|
 * 	|2		|CTS		|CTS/RTS hardware handshaking	|
 *
 *See: OEMStar Firmware Reference Manual Rev 6 page 56
 * \bug not being able to get a correct response parsing.
 * Using only default settings for now.
 */

bool Gps::setComSettings(const uint32_t baud,
		const uint32_t port,
		const uint32_t parity,
		const uint32_t databits,
		const uint32_t stopbits,
		const uint32_t handshake,
		const uint32_t echo,
		const uint32_t _break){

	// check if port is open first
	if(!this->isOpen){
		fprintf(this->logFD, "Device port is not open:%s\n", this->_comPort);
		return(false);
	}

	//variables
	/// \todo check correct values for fields
	struct _comSettings com;
	int n_bytes = sizeof(com);
	GPSResponse myResponse;
	struct termios options;

	// clean memory
	memset(&com,0,n_bytes);
	memset(&myResponse,0,sizeof(myResponse));

	// copy default header structure;
	memcpy(&com.header, &this->defaultHeader,this->sizeHeader);

	com.header.messageID = COM;
	com.header.messageLength = 32; //32bytes
	com.header.portAddress = THISPORT;
	com.port = port;
	com.parity = parity;
	com.databits = databits;
	com.stopbits = stopbits;
	com.handshake = handshake;
	com.echo = echo;
	com._break = _break;
	com.baud = baud;
	com.crc32 = CalculateBlockCRC32(n_bytes-4, (unsigned char*) &com);

#if DEBUG
	DEBUG_MSG("DEBUG: Sending COM command: ");
	unsigned char *aux = (unsigned char*) &com;
	for(int i = 0; i<n_bytes; i++)
		DEBUG_MSG("%02x ", aux[i]);
	DEBUG_MSG("\n");
	fflush(this->logFD);
#endif

	// send to device
	if(write(this->serialPortFD,&com,n_bytes)!=n_bytes){
		DEBUG_MSG("Number of bytes writen to device  not as expected:\n"
				"%s:%d\n", __FILE__, __LINE__);
		return(false);
	}

	/********************************************************************
	 *  Get the response message
	 *******************************************************************/
	//get response header
	/// \todo check sync header first
	//returnVal = this->comRequestStatus;
	//if(returnVal != OK) //ok
	//	return(false);

	/*********************************************************************
	 *  change the baudrate
	 *******************************************************************/
	sleep(1);
	if(ioctl( this->serialPortFD, TCFLSH, TCIOFLUSH)==0)
		fprintf(this->logFD,"buffers cleared\n");

	close(this->serialPortFD);
	this->isOpen = false;
	sleep(2);
	fprintf(this->logFD,"changing baudrate to %d\n",baud);
	fflush(this->logFD);
	this->serialPortFD=open(this->_comPort, O_RDWR | O_NOCTTY | O_NDELAY);
	if (this->serialPortFD < 0) {
		fprintf(this->logFD,"@GPS: Não abriu porta série\n");
		this->openError = true;
		this->errorFlags |= PORTERROR;
		return(false);
	}
	this->isOpen = true;

	tcgetattr(this->serialPortFD,&options);

	/********************************************************************
	 *  set new baudrate
	 *******************************************************************/
	cfsetispeed(&options, parseBaudRate(baud));
	cfsetospeed(&options, parseBaudRate(baud));

	/********************************************************************
	 * set no parity, 8 data bits, 1 stop bit
	 *******************************************************************/
	/// \todo handle diferent values
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_lflag &=~(ICANON | ECHO | ECHOE);

	tcsetattr(this->serialPortFD,TCSANOW,&options);
	fcntl(this->serialPortFD, F_SETFL, 0); // block until data comes in
	fprintf(this->logFD,"done\n\n");
	fflush(this->logFD);
	this->_baudRate = parseBaudRate(baud);
	return(true);
}

int Gps::getBaudRate(){
	switch(this->_baudRate){
	case B300: return (300);
	case B600: return (600);
	case B1200: return(1200);
	case B2400: return(2400);
	case B4800: return(4800);
	case B9600: return(9600);
	case B19200: return(19200);
	case B38400: return(38400);
	case B57600: return(57600);
	case B115200: return(115200);
	default:
		printf("ERROR: undefined baudrate\n");
		return(-1);
	}
}

/**
 *
 * The log request command is defined as:
 *
 *|Field| ID         | N  Bytes 	| Description           		   	|
 *|:---:|:----------:|:------------:|:----------------------------------|
 *|1    | Log header | H = 28   	| Header of message     		   	|
 *|2    | port       | ENUM = 4		| identification of port		   	|
 *|3    | message	 | Ushort = 2	| Message ID of log to output	   	|
 *|4    | messageType| char = 1 	| Message type (Binary)			   	|
 *|5	| RESERVED	 | char = 1		|								   	|
 *|6	| trigger	 | ENUM = 4		| message trigger				   	|
 *|7	| period	 | double = 8	| Log period (for ONTIME in secs)  	|
 *|8	| offset	 | double = 8 	| Offset for period (ONTIME in secs	|
 *|9	| hold		 | ENUM = 4 	| Hold log						   	|
 *|10	| crc32		 | Ulong = 4	| crc32 value						|
 *
 * Total byte size = header + 32 = 60 bytes
 *
 * Log trigger Identifiers (field 6):
 *
 * 	|Binary |ASCII		|Description															|
 * 	|:-----:|:---------:|:----------------------------------------------------------------------|
 * 	|0		| ONNEW 	|when the message is updated (not necessarily changed)					|
 * 	|1 		| ONCHANGED	|Current message and then continue to output when the message is changed|
 * 	|2		| ONTIME	|Output on a time interval												|
 * 	|3		| ONNEXT	|Output only the next message											|
 * 	|4		| ONCE		|Output only the current message										|
 * 	|5		| ONMARK	|Output when a pulse is detected on the mark 1 input					|
 */
bool Gps::askLog(const uint16_t messageID, const uint32_t port,
		uint32_t trigger, double period, double offset){

	// check if port is open first
	if(!this->isOpen){
		fprintf(this->logFD, "Device port is not open:%s\n", this->_comPort);
		return(false);
	}

	struct _log logRequest;
	int n_bytes = sizeof(logRequest);
	int returnVal = 0;
	GPSResponse myResponse;
	// clean memory
	memset(&logRequest,0,n_bytes);
	memset(&myResponse,0,sizeof(myResponse));

	// copy default header structure;
	memcpy(&logRequest.header, &this->defaultHeader,this->sizeHeader);
	// set header for log request
	logRequest.header.messageID = LOG;
	logRequest.header.messageLength = 32; //32bytes
	logRequest.header.portAddress = THISPORT;
	// set log message data
	logRequest.port = port;
	logRequest.messageID = messageID;
	logRequest.messageType = BINARY;
	logRequest.trigger = trigger;
	logRequest.period = period;
	logRequest.offset = offset;
	logRequest.hold = NOHOLD;
	logRequest.crc32 = CalculateBlockCRC32(n_bytes-4, (unsigned char*) &logRequest);

#if DEBUG
	DEBUG_MSG("DEBUG: Sending Log command: ");
	unsigned char *aux = (unsigned char*) &logRequest;
	for(int i = 0; i<n_bytes; i++)
		DEBUG_MSG("%02x ", aux[i]);
	DEBUG_MSG("\n");
	fflush(this->logFD);
#endif
	pthread_mutex_lock(&this->lock);
	// send to device
	if(write(this->serialPortFD,&logRequest,n_bytes)!=n_bytes){
		DEBUG_MSG("Number of bytes writen to device  not as expected:\n"
				"%s:%d\n", __FILE__, __LINE__);
		//pthread_mutex_unlock(&lock);
		return(false);
	}
	pthread_cond_wait(&this->gotLogResponce,&this->lock );
	// Get Status of log Request
	returnVal = this->logRequestStatus;
	pthread_mutex_unlock(&this->lock);
	if(returnVal == OK)
		return(true);
	return(false);
}

void* Gps::processResponses(){
	// check if port is open first
	if(!this->isOpen){
		fprintf(this->logFD, "Device port is not open:%s\n", this->_comPort);
		pthread_exit(NULL);
	}

	// Variables
	GPSheader myHeader =  this->currentHeader;
	GPSResponse myResponse;
	struct _bestposxyz bestxyzLog;
	int serialPortFD = this->serialPortFD;
	int retval;
	uint8_t newByte = 0;
	uint8_t syncBytes[2];
	uint8_t header_buffer[28];
#if DEBUG
	unsigned char *aux = NULL;
#endif

	while(1){
		memset(&header_buffer,0,28);
		newByte = 0;
		read(serialPortFD,&newByte,1);
		pthread_mutex_lock(&this->lock);
		if(newByte == 0xAA){
			read(serialPortFD,&syncBytes, 2);
			if (syncBytes[0] == 0x44 && syncBytes[1] == 0x12){
				// got a valid sync vector
				header_buffer[0]= 0xAA;
				header_buffer[1]= 0x44;
				header_buffer[2]= 0x12;
				read(serialPortFD, &header_buffer[3], 25);
				memset(&myHeader,0, 28);
				memcpy(&myHeader,&header_buffer, 28);
				switch(myHeader.messageID){
				case LOG:
					memset(&myResponse,0,sizeof(myResponse));
					memcpy(&myResponse.header, &myHeader, 28);
					// get response ID
					read(serialPortFD,&myResponse.responseID,4);
					// get message response
					retval=myResponse.header.messageLength-4; //the responseID also counts
					memset(&myResponse.ascii_responce, 0, 512);
					read(serialPortFD,&myResponse.ascii_responce,retval);
					// get CRC32
					read(serialPortFD,&myResponse.crc32, 4);
					fprintf(this->logFD, "Log request response: %s\n",
							myResponse.ascii_responce);
					fflush(this->logFD);
					//pthread_mutex_lock(&this->lock);
					this->logRequestStatus = myResponse.responseID;
					pthread_cond_signal(&this->gotLogResponce);
					//pthread_mutex_unlock(&this->lock);
#if DEBUG
					DEBUG_MSG("Received Responce for Log Command: %s\n", myResponse.ascii_responce);
#endif
					break;
				case UNLOGALL:
					//unlogall response
					memset(&myResponse,0,sizeof(myResponse));
					memcpy(&myResponse.header, &myHeader, 28);
					// get response ID
					read(serialPortFD,&myResponse.responseID,4);

					// get message response
					retval=myResponse.header.messageLength-4; //the responseID also counts
					memset(&myResponse.ascii_responce, 0, 512);
					read(serialPortFD,&myResponse.ascii_responce,retval);

					// get CRC32
					read(serialPortFD,&myResponse.crc32, 4);
					//bytesOnBuffer-=4;
					fprintf(this->logFD, "Unlogall response: %s\n",
							myResponse.ascii_responce);
					fflush(this->logFD);
					//pthread_mutex_lock(&this->lock);
					this->unlogallRequestStatus = myResponse.responseID;
					pthread_cond_signal(&this->gotUnlogResponce);
					//pthread_mutex_unlock(&this->lock);
#if DEBUG
					DEBUG_MSG("Received Responce for Unlogall Command: %s\n", myResponse.ascii_responce);
#endif
					break;
				case BESTXYZ: // bestxyz log
					memset(&bestxyzLog,0,sizeof(bestxyzLog));
					memcpy(&bestxyzLog.header,&myHeader, 28);
					retval=bestxyzLog.header.messageLength;
					read(this->serialPortFD,&bestxyzLog.pSolStatus,retval);

					read(this->serialPortFD,&bestxyzLog.crc32, 4);

#if DEBUG
					DEBUG_MSG("DEBUG: bestxyz log: ");
					aux = (unsigned char*) &bestxyzLog;
					retval = sizeof(bestxyzLog);
					for(int i = 0; i<retval; i++)
						DEBUG_MSG("%02x ", aux[i]);
					DEBUG_MSG("\n");
					fflush(this->logFD);
#endif
					this->parseBestxyzLog(bestxyzLog);
					break;
				default:
					continue;
				}
			}
		}
		pthread_mutex_unlock(&this->lock);
	}
	pthread_exit(NULL);
}

void Gps::parseBestxyzLog(struct _bestposxyz data){
	struct timespec clock;
	clock_gettime(CLOCK_MONOTONIC,&clock);
	// "Indice,Time,PSolStatus,X,Y,Z,stdX,stdY,stdZ,VSolStatus,VX,VY,VZ,stdVX,stdVY,stdVZ,VLatency,SolAge,SolSatNumber
	fprintf(this->logData, "%5d,%li.%03d,%d,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",
			this->CurrentRecord,
			clock.tv_sec,
			(int)(((double)clock.tv_nsec)/1000000),
			data.pSolStatus,
			data.position[0],
			data.position[1],
			data.position[2],
			data.positionStd[0],
			data.positionStd[1],
			data.positionStd[2],
			data.vSolStatus,
			data.velocity[0],
			data.velocity[1],
			data.velocity[2],
			data.velocityStd[0],
			data.velocityStd[1],
			data.velocityStd[2],
			data.vLatency,
			data.solAge,
			data.numSolSatVs
			);
	fflush(this->logData);
	return;
}

Gps::~Gps() {
	// TODO Auto-generated destructor stub
}

