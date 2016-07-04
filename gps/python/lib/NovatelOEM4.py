import Queue
import binascii
import crcmod
from datetime import datetime
import logging
import os
import serial
import struct
import sys
import threading
from time import sleep


class Gps:
    header_keys = ('sync', 'headerLength', 'messageID', 'messageType',
                   'portAddress', 'messageLength', 'sequence', 'idleTime',
                   'timeStatus', 'week', 'ms', 'receiverStatus', 'reserved',
                   'swVersion')
    current_header = []
    MessageID = {'LOG': 1, 'COM': 4, 'UNLOGALL': 38, 'BESTXYZ': 241}
    Indice = 1

    def __init__(self, sensorName="GPS"):
        self.myPort = ""
        self.isOpen = 0  # is port open?*/
        self.baudRate = 9600  # current communication baudRate*/
        self.openError = 0  # if any error during any process occurs this will be set */
        self.logFile = ""
        self.dataFile = ""
        self.name = sensorName
        self.isSet = False
        self.exitFlag = threading.Event()
        self.orders = Queue.Queue()

    def CRC32Value(self, i):
        '''
        @brief calculate the 32bits CRC of message.
        @param i message to calculate the crc-32.
        @return the CRC value calculated.

        See: OEMStar Firmware Reference Manual Rev 6 page 24 for
        more information.
        '''
        crc = crcmod.mkCrcFun(0x104C11DB7, 0, True, 0)
        return crc(i)

    def getDebugMessage(self, message):
        '''
        @brief create a string which contains all bytes represented
        as hex values

        @param message message to be represented.
        @return the corresponding hex representation of message.
        '''
        debugMessage = (binascii.hexlify(message)).upper()
        debugMessage = [debugMessage[i:i + 2] for i in range(0, len(debugMessage), 2)]
        debugMessage = ' '.join('0x{}'.format(item) for item in debugMessage)
        return debugMessage

    def parseResponces(self):
        self.log.info("Entering Thread logger")
        if(not self.isOpen):
            self.log.warning('Port is not open: {0}'.format(self.myPort))
            self.log.info("Exiting Thread logger")
            return
        MYPORT = self.myPort
        dataFile = self.dataFile
        while(self.exitFlag.isSet() == False):
            header = [0] * 14
            newByte = ord(MYPORT.read(1))
            if newByte == 0xAA:
                header[0] = [0, 0, 0]
                header[0][0] = newByte
                header[0][1] = ord(MYPORT.read(1))
                if header[0][1] == 0x44:
                    header[0][2] = ord(MYPORT.read(1))
                    if header[0][2] == 0x12:
                        # got a valid header sync vector
                        serialBuffer = MYPORT.read(25)
                        header[1:] = struct.unpack('<BHBBHHBBHlLHH',
                                                   serialBuffer)
                        self.current_header = dict(zip(self.header_keys, header))
                        header = self.current_header
                        if header['messageID'] == self.MessageID['LOG']:
                            message = [0] * 3
                            message_keys = ('responseID', 'ascii', 'crc32')
                            serialBuffer = MYPORT.read(4)
                            message[0] = struct.unpack('<I', serialBuffer)
                            message[1] = MYPORT.read(self.current_header['messageLength'] - 4)
                            message[2] = MYPORT.read(4)
                            message[2] = struct.unpack('<L', message[2])
                            message = dict(zip(message_keys, message))
                            self.log.info("LOG response received : {0}".format(message['ascii']))
                            self.orders.put({'order': 'LOG', 'data': message})
                        elif header['messageID'] == self.MessageID['UNLOGALL']:
                            message = [0] * 3
                            message_keys = ('responseID', 'ascii', 'crc32')
                            serialBuffer = MYPORT.read(4)
                            message[0] = struct.unpack('<I', serialBuffer)
                            message[1] = MYPORT.read(self.current_header['messageLength'] - 4)
                            message[2] = MYPORT.read(4)
                            message[2] = struct.unpack('<L', message[2])
                            message = dict(zip(message_keys, message))
                            self.log.info("UNLOGALL response received : {0}".format(message['ascii']))
                            self.orders.put({'order': 'UNLOGALL', 'data': message})
                        elif header['messageID'] == self.MessageID['COM']:
                            message = [0] * 3
                            message_keys = ('responseID', 'ascii', 'crc32')
                            serialBuffer = MYPORT.read(4)
                            message[0] = struct.unpack('<I', serialBuffer)
                            message[1] = MYPORT.read(self.current_header['messageLength'] - 4)
                            message[2] = MYPORT.read(4)
                            message[2] = struct.unpack('<L', message[2])
                            message = dict(zip(message_keys, message))
                            self.log.info("COM response received : {0}".format(message['ascii']))
                            self.orders.put({'order': 'COM', 'data': message})
                        elif header['messageID'] == self.MessageID['BESTXYZ']:
                            message = [0] * 20
                            message_keys = ('pSolStatus', 'posType', 'position', 'positionStd',
                                            'velSolStatus', 'velType', 'velocity', 'velocityStd',
                                            'stnID', 'vLatency', 'diffAge', 'solAge', 'numStasVs',
                                            'numSolSatVs', 'numGGL1', 'reserved', 'extSolStat',
                                            'reserved2', 'sigMask', 'crc32')
                            serialBuffer = MYPORT.read(self.current_header['messageLength'])
                            message[0:2] = struct.unpack('<II', serialBuffer[0:8])
                            message[2] = [0, 0, 0]
                            message[2][0:] = struct.unpack('<ddd', serialBuffer[8:32])
                            message[3] = [0, 0, 0]
                            message[3][0:] = struct.unpack('<fff', serialBuffer[32:44])
                            message[4:6] = struct.unpack('<II', serialBuffer[44:52])
                            message[6] = [0, 0, 0]
                            message[6][0:] = struct.unpack('<ddd', serialBuffer[52:76])
                            message[7] = [0, 0, 0]
                            message[7][0:] = struct.unpack('<fff', serialBuffer[76:88])
                            message[8] = serialBuffer[88:92]
                            message[9:12] = struct.unpack('<fff', serialBuffer[92:104])
                            message[12:15] = struct.unpack('<3B', serialBuffer[104:107])
                            message[15] = [0, 0]
                            message[15][0:] = struct.unpack('<2B', serialBuffer[107:109])
                            message[16:19] = struct.unpack('<3B', serialBuffer[109:112])
                            serialBuffer = MYPORT.read(4)  # crc32
                            message[19] = struct.unpack('<I', serialBuffer)
                            message = dict(zip(message_keys, message))
                            # "Indice,Time,PSolStatus,X,Y,Z,stdX,stdY,stdZ,VSolStatus,VX,VY,VZ,stdVX,stdVY,stdVZ,VLatency,SolAge,SolSatNumber\n"
                            currentTime = datetime.now()
                            myTime = '{0:%Y-%m-%d %H:%M:%S}'.format(currentTime) + '.{0:02.0f}'.format(round(currentTime.microsecond / 10000.0))
                            dataFile.write('{0:5d},{1},{2},{3},{4},{5},'
                                           '{6},{7},{8},{9},{10},{11},'
                                           '{12},{13},{14},{15},{16},'
                                           '{17},{18}\n'.format(self.Indice, myTime,
                                                                message['pSolStatus'],
                                                                message['position'][0],
                                                                message['position'][1],
                                                                message['position'][2],
                                                                message['positionStd'][0],
                                                                message['positionStd'][1],
                                                                message['positionStd'][2],
                                                                message['velSolStatus'],
                                                                message['velocity'][0],
                                                                message['velocity'][1],
                                                                message['velocity'][2],
                                                                message['velocityStd'][0],
                                                                message['velocityStd'][1],
                                                                message['velocityStd'][2],
                                                                message['vLatency'],
                                                                message['solAge'],
                                                                message['numSolSatVs']
                                                                ))
                            self.Indice = self.Indice + 1
                            dataFile.flush()
                        else:
                            # todo error
                            pass
            else:
                self.log.debug("New Byte unexpected: 0x{0:X}".format(newByte))
        self.log.info("Exiting Thread logger")
        return

    def begin(self, comPort="/dev/ttyUSB0",
              logFile="output.log",
              dataFile="out.csv",
              baudRate=9600,
              logLevel=20):
        '''
        @brief Initializes the gps receiver.

        This function resets the current port to factory default and setup the
        gps receiver to be able to acept new commands. Also creates the necessary
        log files used to save data transmited.
        If connection to gps is made, it launchs a thread used to parse messages comming from gps.
        @param comPort  system port where receiver is connected
        @param logFile  output log for the tipical messages from the class.
        @param dataFile data log file used to store incoming requested log commands
        @param baudRate baudrate to configure port. (should always be equal to factory default of
        receiver).
        @param logLevel level at which class message should be printed to logFile.
        @return True or False if the setup has gone as expected or not.

        usage :
        ~~~{.py}
        Gps.begin(comPort="<port>",
            dataFile="<your datafile>",
            logFile="<your logfile>",
            baudRate=9600,
            logLevel=<logging level int>)
        ~~~
        Default values:
        - comPort="/dev/ttyUSB0"
        - dataFile="./data/out.csv"
        - logFile="output.log"
        - baudRate=9600
        - logLevel=20

        loglevels are defined as:

        |Level    | Numeric value  |
        |---------|----------------|
        |CRITICAL |  50            |
        |ERROR    |  40            |
        |WARNING  |  30            |
        |INFO     |  20            |
        |DEBUG    |  10            |
        |NOTSET   |  0             |

        check documentation of [module logging] for more info
        [module logging]: https://docs.python.org/2/library/logging.html

        HW info:
        Receptor - Novatel Flexpak G2L-3151W
        Antenna - Novatel Pinwheel

        '''

        # create a logfile
        self.logFile = logFile
        logging.basicConfig(filename=logFile,
                            level=logLevel,
                            format='[%(asctime)s] [%(threadName)-10s] %(levelname)-8s %(message)s',
                            filemode="w")

        self.log = logging.getLogger(self.name)

        # checking if port exists on system
        if not os.path.exists(comPort):
            self.log.warning('Port is not available: {0}'.format(comPort))
            return False
        else:
            # port exists, open it
            self.myPort = serial.Serial(comPort, baudrate=baudRate)
            if not self.myPort.is_open:
                self.log.warning("Error opening port: {0}".format(comPort))
                self.isOpen = False
                return False
            # reset port settings to default
            self.myPort.break_condition = True
            self.myPort.send_break()
            sleep(1.5)
            self.myPort.send_break()
            sleep(0.25)
            self.baudRate = baudRate
            self.isOpen = True
            # open dataFile to save GPS data
            self.dataFile = open(dataFile, 'w')
            # write header of csv dataFile
            self.dataFile.write("Indice,Time,PSolStatus,X,Y,Z,stdX,stdY,stdZ,VSolStatus,VX,VY,VZ,stdVX,stdVY,stdVZ,VLatency,SolAge,SolSatNumber\n")
            self.dataFile.flush()
            # start thread to handle GPS responces
            self.threadID = threading.Thread(name="Logger", target=self.parseResponces)
            self.threadID.start()
            self.log.info("Started Logger Thread")
            sleep(0.1)
            return True

    def create_header(self, messageID, messageLength, portAddress=192):
        '''
        @brief creates a header object to be passed to receiver.
        @param messageID the corresponding value of identifying the message body.
        @param messageLength size of message in bytes excluding CRC-32bit code.
        @oaram portAddress port from where message request is sent.

        the header is defined as:

        |Field| Value | N Bytes | Description |
        |:---:|:-----:|:-------:|:------------|
        |1    | sync[0]| UChar = 1| Hexadecimal 0xAA.|
        |2    | sync[1]| UChar = 1| Hexadecimal 0x44.|
        |3    | sync[2]| UChar = 1| Hexadecimal 0x12.|
        |4    | headerLength| UChar = 1| Length of the header (should always be 28 unless some firmware update)|
        |5    | messageID| UShort = 2 | This is the Message ID code|
        |6    | messageType | UChar = 1 | message type mask (binary and original message)|
        |7    | portAddress | Uchar = 1 | Corresponding value of port |
        |8    | messageLength| UShort = 2| Length of message body |
        |9    | sequence | UShort = 2 | This is used for multiple related logs.|
        |10   | idleTime | UChar = 1 | The time that the processor is idle in the last second between successive logs with the same Message ID|
        |11   | timeStatus| Enum = 1 | Indicates the quality of the GPS time |
        |12   | week | UShort = 2 | GPS week number. |
        |13   | ms   | int = 4    | Milliseconds from the beginning of the GPS week.|
        |14   | receiverStatus | Ulong = 4 | 32 bits representing the status of various hardware and software components of the receiver|
        |15   | reserved    | UShort = 2 ||
        |16   | swVersion | UShort = 2| receiver software build number. |


        portAddress=192 (equal to thisport)

        '''
        header = [0] * 16
        # First 3 start bytes are always.
        header[0] = 0xAA
        header[1] = 0x44
        header[2] = 0x12
        # header Length 1C
        header[3] = 0x1C
        # messageID
        header[4] = messageID
        # messageType binary and original responce = b000xxxxx = 0x02?
        header[5] = 0x02
        # port address
        header[6] = portAddress
        # message length in bytes
        header[7] = messageLength
        return header

    def sendUnlogall(self):
        '''
        @brief Send command unlogall to gps device.

        Message is sent in binary mode.
        On sucess clears all logs on all ports even held logs.

        @return True or False if the request has gone as expected or not.

        unlogall message is defined as:

        | Field| value      | N  Bytes | Description                   |
        |:----:|------------|:--------:|-------------------------------|
        |1     | header     | H = 28   | Header of message             |
        |2     | port       | ENUM = 4 | identification of port        |
        |3     | Held       | ENUM = 4 | can only be 0 or 1. Clear logs|
        |      |            |          | with hold flag or not?        |
        | CRC32|            | UL = 4   |                               |

        See: OEMStar Firmware Reference Manual Rev 6 page 161
        '''
        if self.isOpen:
            MYPORT = self.myPort

            messageSize = 8  # 2 * ENUM
            header = self.create_header(messageID=38,
                                        messageLength=messageSize)
            myMessage = struct.pack('<BBBBHBBHHBBHlLHH', *header)
            myMessage = myMessage + struct.pack('<LL', 8, 1)
            crc_value = self.CRC32Value(myMessage)
            finalMessage = myMessage + struct.pack('<L', crc_value)

            # print messages to logFile
            self.log.info("Requested unlogall")
            self.log.debug('Message sent to GPS: {0}'.format(self.getDebugMessage(finalMessage)))
            MYPORT.write(finalMessage)
            MYPORT.flush()

            # wait for data on queue with response
            message = self.orders.get()
            if message['order'] == 'UNLOGALL':
                message = message['data']
                self.log.info("Unlogall response received : {0}".format(message['ascii']))
                if message['responseID'][0] == 1:
                    return True
                else:
                    return False
            else:
                self.log.warning("Unexpected responce type: {0}".format(message['order']))
        else:
            self.log.info("Port not open. Couldn't request unlogall command")
            return False

    def setCom(self, baud, port=6, parity=0, databits=8, stopbits=1,
               handshake=0, echo=0, breakCond=1):
        '''
         The com request command is defined as:

        |Field| ID         | N  Bytes | Description                      |
        |:---:|:----------:|:--------:|:---------------------------------|
        |1    | Com header | H = 28   | Header of message                |
        |2    | port       | ENUM = 4 | identification of port           |
        |3    | baud       | Ulong = 4| Communication baud rate (bps)    |
        |4    | parity     | ENUM = 4 | Parity                           |
        |5    | databits   | Ulong = 4| Number of data bits (default = 8)|
        |6    | stopbits   | Ulong = 4| Number of stop bits (default = 1)|
        |7    | handshake  | ENUM = 4 | Handshaking                      |
        |8    | echo       | ENUM = 4 | No echo (default)(must be 0 or 1)|
        |9    | break      | ENUM = 4 | Enable break detection (default 0),(must be 0 or 1)|

        Total byte size = header + 32 = 60 bytes

        COM Serial Port Identifiers (field 2):

        |Binary |ASCII      |Description            |
        |:-----:|:---------:|:----------------------|
        |1      | COM1         |COM port 1          |
        |2      | COM2         |COM port 2          |
        |6      | THISPORT     |The current COM port|
        |8      | ALL          |All COM ports       |
        |9      | XCOM1        |Virtual COM1 port   |
        |10     | XCOM2        |Virtual COM2 port   |
        |13     | USB1         |USB port 1          |
        |14     | USB2         |USB port 2          |
        |15     | USB3         |USB port 3          |
        |17     | XCOM3        |Virtual COM3 port   |

        Parity(field 4):

        |Binary |ASCII        |Description          |
        |:-----:|:---------:|:----------------------|
        |0      | N         | No parity (default)   |
        |1      | E         | Even parity           |
        |2      | O         | Odd parity            |

        Handshaking (field 7):

        |Binary |ASCII    |Description                    |
        |:-----:|:-------:|:------------------------------|
        |0      |N        |No handshaking (default)       |
        |1      |XON      |XON/XOFF software handshaking  |
        |2      |CTS      |CTS/RTS hardware handshaking   |

        See: OEMStar Firmware Reference Manual Rev 6 page 56
        '''
        if self.isOpen:
            MYPORT = self.myPort
            messageSize = 32  # 32bytes length
            header = self.create_header(messageID=4, messageLength=messageSize)
            myMessage = struct.pack('<BBBBHBBHHBBHlLHH', *header)
            myMessage = myMessage + struct.pack('<8L', port, baud, parity, databits,
                                                stopbits, handshake, echo, breakCond)
            crc_value = self.CRC32Value(myMessage)
            finalMessage = myMessage + struct.pack('<L', crc_value)
            self.log.info("Requested Com command")
            self.log.debug('Message sent to GPS: {0}'.format(self.getDebugMessage(finalMessage)))
            MYPORT.write(finalMessage)
            MYPORT.flush()
            self.log.info("waiting for port settings to change")
            sleep(1)
            portOptions = MYPORT.get_settings()
            # change port settings
            portOptions['baudrate'] = baud
            # auxiliar vector
            parity_vect = [serial.PARITY_NONE, serial.PARITY_EVEN, serial.PARITY_ODD]
            portOptions['parity'] = parity_vect[parity]
            portOptions['bytesize'] = databits
            portOptions['stopbits0'] = stopbits
            if handshake == 0:
                portOptions['xonxoff'] = False
                portOptions['rtscts'] = False
            elif handshake == 1:
                portOptions['xonxoff'] = True
                portOptions['rtscts'] = False
            elif handshake == 2:
                portOptions['xonxoff'] = False
                portOptions['rtscts'] = True
            MYPORT.apply_settings(portOptions)
            MYPORT.reset_input_buffer()
            MYPORT.reset_output_buffer()
            if MYPORT.is_open:
                self.log.info("changed port settings: {0}".format(MYPORT.portstr))
                return True
            else:
                self.log.warning("Not able to change port settings: {0}".format(MYPORT.portstr))
                return False
        else:
            self.log.info("Port not open. Couldn't request COM command")
            return False

    def askLog(self, logID='BESTXYZ', port=192, trigger=4, period=0, offset=0, hold=0):
        '''
        The log request command is defined as:

        |Field| ID         | N  Bytes     | Description                       |
        |:---:|:----------:|:------------:|:----------------------------------|
        |1    | Com header | H = 28       | Header of message                 |
        |2    | port       | ENUM = 4     | identification of port            |
        |3    | message    | Ushort = 2   | Message ID of log to output       |
        |4    | messageType| char = 1     | Message type (Binary)             |
        |5    | RESERVED   | char = 1     |                                   |
        |6    | trigger    | ENUM = 4     | message trigger                   |
        |7    | period     | double = 8   | Log period (for ONTIME in secs)   |
        |8    | offset     | double = 8   | Offset for period (ONTIME in secs |
        |9    | hold       | ENUM = 4     | Hold log                          |
        |10   | crc32      | Ulong = 4    | crc32 value                       |

         Total byte size = header + 32 = 60 bytes

         Log trigger Identifiers (field 6):

        |Binary |ASCII      |Description                                                            |
        |:-----:|:---------:|:----------------------------------------------------------------------|
        |0      | ONNEW     |when the message is updated (not necessarily changed)                  |
        |1      | ONCHANGED |Current message and then continue to output when the message is changed|
        |2      | ONTIME    |Output on a time interval                                              |
        |3      | ONNEXT    |Output only the next message                                           |
        |4      | ONCE      |Output only the current message                                        |
        |5      | ONMARK    |Output when a pulse is detected on the mark 1 input                    |

        '''
        if self.isOpen:
            MYPORT = self.myPort
            messageSize = 32
            header = self.create_header(messageID=1, messageLength=messageSize)
            myMessage = struct.pack('<BBBBHBBHHBBHlLHH', *header)
            myMessage = myMessage + struct.pack('<LHBBLddL', port, self.MessageID[logID],
                                                0, 0, trigger, period, offset, hold)
            crc_value = self.CRC32Value(myMessage)
            finalMessage = myMessage + struct.pack('<L', crc_value)
            self.log.info("Requested Log command")
            self.log.debug('Message sent to GPS: {0}'.format(self.getDebugMessage(finalMessage)))
            MYPORT.write(finalMessage)
            MYPORT.flush()
            # wait for responce
            message = self.orders.get()
            if message['order'] == 'LOG':
                message = message['data']
                self.log.info("LOG response received : {0}".format(message['ascii']))
                if message['responseID'][0] == 1:
                    return True
                else:
                    return False
            else:
                self.log.warning("Unexpected responce type: {0}".format(message['order']))
        else:
            self.log.info("Port not open. Couldn't request LOG command")
            return False

    def shutdown(self):
        self.sendUnlogall()
        self.exitFlag.set()
        self.sendUnlogall()
        self.threadID.join()
        # reset port settings to default
        self.myPort.break_condition = True
        self.myPort.send_break()
        sleep(1.5)
        self.myPort.send_break()
        sleep(0.25)
        self.myPort.close()
        self.isOpen = False
        self.log.info("Shuting down")
        self.dataFile.close()
        logging.shutdown()
        return True


def main():
    '''
    Set of test to run to see if class behaves as expected.

    Creates a Gps class object and execute the following commands on gps receiver:
    - begin: on on default port or given port by argv[1].
    - sendUnlogall
    - setCom(baud=115200): changes baudrate to 115200bps
    - askLog(trigger=2, period=0.1): ask for log *bestxyz* with trigger `ONTIME` and period `0.1`
    - wait for 10 seconds
    - shutdown: safely disconnects from gps receiver
    '''
    gps = Gps()
    # check if arguments with port are passed
    num_args = len(sys.argv)
    if num_args > 1:
        # print("argument = {0}".format(sys.argv[1]))
        if(gps.begin(comPort=sys.argv[1]) != 1):
            print("Not able to begin device properly... check logfile")
            return
    else:
        if (gps.begin() != 1):
            print("Not able to begin device properly... check logfile")
            return
    if(gps.sendUnlogall() != 1):
        print("Unlogall command failed... check logfile")
        gps.myPort.close()
        return
    gps.setCom(baud=115200)
    gps.askLog(trigger=2, period=0.1)
    sleep(10)
    gps.shutdown()
    return

if __name__ == '__main__':
    main()
