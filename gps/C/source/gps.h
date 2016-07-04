/** \file
 * \class Gps
 * \brief Declarations for the class gps()
 *
 *
 * **Hardware information:**
 * * Receptor: Novatel Flexpak G2L-3151W
 * * Antenna - Novatel Pinwheel
 *
 * \author Bruno Tibério
 * \date 2016
 */

#ifndef GPS_H_
#define GPS_H_
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
#include <sys/select.h>
#include <pthread.h>
#include <time.h>

#define LOGERROR  0x01
#define PORTERROR 0x02
#define OK 1
#define DEBUG 1

//Message Type
#define BINARY 0x00

// MessageID Header defines
#define LOG 1
#define COM 4
#define UNLOGALL 38
#define BESTXYZ 241

// header port defines
#define THISPORT 192

//message port defines
#define THISPORT_ALL 6
#define ALL_PORTS 8

//LOG MessageID
#define BESTXYZ 241

//LOG Trigger defines
#define ONNEW 0
#define ONCHANGED 1
#define ONTIME 2
#define ONNEXT 3
#define ONCE 4
#define ONMARK 5
#define NOHOLD 0
#define HOLD 1


#if DEBUG == 1
#define DEBUG_MSG(...) fprintf(this->logFD,__VA_ARGS__)
#endif

#ifndef DEBUG_MSG
#define DEBUG_MSG(...)
#endif
/**
 * \brief structure for the header of each message
 */

typedef struct _header{
	uint8_t sync[3]; //<sync vector is always
	uint8_t headerLength;
	uint16_t messageID;
	uint8_t messageType;
	uint8_t portAddress;
	uint16_t messageLength;
	uint16_t sequence;
	uint8_t idleTime;
	uint8_t timeStatus;
	uint16_t week;
	int ms;
	uint32_t receiverStatus;
	uint16_t reserved;
	uint16_t swVersion;
}GPSheader;

struct _comSettings{
	struct _header header;
	uint32_t port;
	uint32_t baud;
	uint32_t parity;
	uint32_t databits;
	uint32_t stopbits;
	uint32_t handshake;
	uint32_t echo;
	uint32_t _break;
	uint32_t crc32;
};
/**
 * \brief structure to send log request
 */

struct _log{
	struct _header header;
	uint32_t port;
	uint16_t messageID;
	uint8_t messageType;
	uint8_t reserved;
	uint32_t trigger;
	double period;
	double offset;
	uint32_t hold;
	uint32_t crc32;
};
/**
 * \brief structure for bestposxyz log
 */
struct _bestposxyz{
	struct _header header;
	uint32_t pSolStatus;
	uint32_t posType;
	double position[3];
	float positionStd[3];
	uint32_t vSolStatus;
	uint32_t velType;
	double velocity[3];
	float velocityStd[3];
	char stnID[4];
	float vLatency;
	float diffAge;
	float solAge;
	uint8_t numSatVs;
	uint8_t numSolSatVs;
	uint8_t numGGL1;
	uint8_t reserved[2];
	uint8_t extSolStat;
	uint8_t reserved2;
	uint8_t sigMask;
	uint32_t crc32;
};
/**
 * \brief structure to send unlogall request
 */
struct _unlogall {
	struct _header header;
	uint32_t port;
	uint32_t held;
	uint32_t crc32;
};

/**
 * \brief structure to handle binary response to request commands
 */
typedef struct _response {
	struct _header header;
	uint32_t responseID;
	char ascii_responce[512];
	uint32_t crc32;
}GPSResponse;



class Gps {
	const char* _comPort; /**< system path location of device*/
	bool isOpen, exitOrder; /**< is port open?*/
	speed_t _baudRate; /**< current communication baudRate*/
	int serialPortFD;	/**< File descriptor to the device */
	bool openError; /**< if any error during any process occurs this will be set */
	FILE *logFD, *logData;
	uint8_t errorFlags;
	GPSheader defaultHeader;
	GPSheader currentHeader;
	uint8_t sizeHeader;
	uint32_t CurrentRecord;
	uint32_t logRequestStatus;
	uint32_t unlogallRequestStatus;
	uint32_t comRequestStatus;


public:
	pthread_t threadID;
	pthread_mutex_t lock;
	pthread_cond_t gotLogResponce;
	pthread_cond_t gotUnlogResponce;

	/**
	 * \brief default constructor
	 */
	Gps();

	static void *Help_processResponses(void *context)
	    {

			return((Gps *)context)->processResponses();
	    }

	/**
	 * \brief implements the basic operations for configuring the communication
	 * port of the gps sensor
	 *
	 * \param comPort system path to the port. Default is /dev/ttyUSB0
	 * \param log the name of logfile. Default gps.log
	 * \param baudRate value of desired baudRate. Default 9600.
	 * \return a bool if the configuration was sucessful or not.
	 */
	bool begin(const char *comPort="/dev/ttyUSB0", const char *log="gps.log", const char *data="data.csv", const int baudRate=9600);

	/**
	 * \brief constructs an unlogall command to send for gps
	 */
	bool sendUnlogall();

	/**
	 * \brief Parses the desired baudrate value
	 */
	speed_t parseBaudRate(const int baud);
	/**
	 * \brief change communication configuration settings
	 *
	 *
	 * \return a bool if the configuration was sucessful or not
	 */
	bool setComSettings(const uint32_t baud,
			const uint32_t port=THISPORT,
			const uint32_t parity=0,
			const uint32_t databits=8,
			const uint32_t stopbits=1,
			const uint32_t handshake=0,
			const uint32_t echo=0,
			const uint32_t _break=1);


	bool askLog(const uint16_t messageID,
			const uint32_t port=THISPORT,
			uint32_t trigger=ONCE,
			double period=0,
			double offset=0);

	void *processResponses();
	void parseBestxyzLog(struct _bestposxyz data);

	virtual ~Gps();

	/**
	 * \brief returns current defined communication port.
	 *
	 */
	const char* getComPort() const
	{
		return (_comPort);
	}

	/**
	 * \brief returns the current Error flags value
	 */
	uint8_t getErrorFlags() const
	{
		return(errorFlags);
	}

	/**
	 * \brief return the current system File Descriptor for the device.
	 *
	 * \return int value of system file descriptor for device.
	 */
	int getPortFd() const
	{
		return(serialPortFD);
	}

	/**
	 * \brief check if port is open.
	 *
	 * \return True or False acording to the state of device
	 */
	bool IsOpen() const
	{
		return (isOpen);
	}

	/**
	 * \brief returns the current log file descriptor used by device.
	 *
	 * \return FILE* pointer for the log file
	 */
	FILE* getLogFd() const
	{
		return (logFD);
	}

	/**
	 * \brief check if any error occurred during open device.
	 *
	 * \return True or False accordingly
	 */
	bool OpenError() const
	{
		return (openError);
	}
	int getBaudRate();
};



#endif /* GPS_H_ */
