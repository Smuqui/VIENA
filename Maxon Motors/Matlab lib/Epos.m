classdef Epos < handle
	%% Class for Maxton EPOS 70/10 motor control
	% to be added
	
	properties
		portObj;
		nodeID;
		currFrame;
		errors;
		connected;
		baudRate;
		baudRateMap = containers.Map({9600,14400,19200,38400,57600,115200},[0,1,2,3,4,5]);
		debug_flag;
		ResponseCodes = containers.Map({'E_OK', 'E_FAIL', 'E_ANS'}, hex2dec({'4F','46', '0'}))
		OPCode = containers.Map({'READ','WRITE'}, hex2dec({'10','11'}));
		objectIndex = containers.Map(...
			{'DeviceType','ErrorRegister','ErrorHistory','COB_ID_SYNC',...
			'ManufacturerDeviceName','GuardTime','LifeTimeFactor','Store',...
			'RestoreDefaultParameters','COB_ID_EMCY','ConsumerHeartbeatTime',...
			'ProducerHeartbeatTime','IdentityObject','VerifyConfiguration',...
			'ServerSDOParameter','ReceivePDO1Parameter','ReceivePDO2Parameter',...
			'ReceivePDO3Parameter','ReceivePDO4Parameter','ReceivePDO1Mapping',...
			'ReceivePDO2Mapping','ReceivePDO3Mapping','ReceivePDO4Mapping',...
			'TransmitPDO1Parameter','TransmitPDO2Parameter','TransmitPDO3Parameter',...
			'TransmitPDO4Parameter','TransmitPDO1Mapping','TransmitPDO2Mapping',...
			'TransmitPDO3Mapping','TransmitPDO4Mapping','NodeID','CANBitrate',...
			'RS232Baudrate','Version','SerialNumber','RS232FrameTimeout',...
			'MiscellaneousConfig','CustomPersistentMemory','EncoderCounterIndexPulse',...
			'HallsensorPattern','CurrentActualValueAveraged','VelocityActualValueAveraged',...
			'CurrentModeSettingValue','PositionModeSettingValue','VelocityModeSettingValue',...
			'ConfigurationDigitalInputs','DigitalInputFuntionalities','PositionMarker',...
			'DigitalOutputFunctionalities','ConfigDigitalOutputs','AnalogInputs',...
			'CurrentThresholdHomingMode','HomePosition','FollowingErrorActualValue',...
			'SensorConfiguration','DigitalPositionInput','Controlword','Statusword',...
			'ModesOperation','ModesOperationDisplay','PositionDemandValue',...
			'PositionActualValue','MaximalFollowingError','PositionWindow',...
			'PositionWindowTime','VelocitySensorActualValue','VelocityDemandValue',...
			'VelocityActualValue','CurrentActualValue','TargetPosition',...
			'HomeOffset','SoftwarePositionLimit','MaximalProfileVelocity',...
			'ProfileVelocity','ProfileAcceleration','ProfileDeceleration',...
			'QuickStopDeceleration','MotionProfileType','PositionNotationIndex',...
			'PositionDimensionIndex','VelocityNotationIndex','VelocityDimentionIndex',...
			'AccelerationNotationIndex','AccelerationDimensionIndex','HomingMethod',...
			'HomingSpeeds','HomingAcceleration','CurrentControlParameterSet',...
			'VelocityControlParameterSet','PositionControlParameterSet',...
			'TargetVelocity','MotorType','MotorData','SupportedDriveModes'},...
			uint16(hex2dec({'1000','1001','1003','1005','1008','100C',...
			'100D','1010','1011','1014','1016','1017','1018','1020',...
			'1200','1400','1402','1403','1600','1601','1602','1603',...
			'1800','1801','1801','1802','1803','1A00','1A01','1A02',...
			'1A03','2000','2001','2002','2003','2004','2005','2008',...
			'200C','2021','2022','2027','2028','2030','2062','206B',...
			'2070','2071','2074','2078','2079','207C','2080','2081',...
			'20F4','2210','2300','6040','6041','6060','6061','6062',...
			'6064','6065','6067','6068','6069','606B','606C','6078',...
			'607A','607C','607D','607F','6081','6083','6084','6085',...
			'6086','6089','608A','608B','608C','608D','608E','6098',...
			'6099','609A','60F6','60F9','60FB','60FF','6402','6410',...
			'6502'})));
		motorType = containers.Map({'DC motor', 'Sinusoidal PM BL motor', 'Trapezoidal PM BL motor'},...
			{1, 10, 11});
	end
	methods
		
		function me=Epos(debug_flag)
			% me = Epos (debug_flag)
			% EPOS Constructor
			% If debug flag is active, it reports communications between PC and 
			% Epos device
			
			% check if debug is used
			if exist('debug_flag','var')
				me.debug_flag = debug_flag;
			else
				me.debug_flag = 0;
			end
			
			me.connected = false;
			me.nodeID=uint8(1); % zero sends to all
			me.portObj = [];
			me.baudRate = 115200;
			format hex;
		end
		
		function [] = delete(me)
			me.disconnect();
		end
		%{
        ****************************************************************
        @fn begin(devname, Baud)
        @brief connects to Epos
        
        Establish the connection to EPOS via RS232 connection
        Sets connected if configuration was sucessfull or not.

        NOTE: it changes format to hex for easier visualization of message
        transaction.
        
        @param devname Portname for the device (example: '/dev/ttyUSB0')
        @param Baud        [optional] baudrate for the communication (default
        115200)
        *****************************************************************
		%}
		function begin(me, devname, Baud)
			if ~exist('Baud', 'var')
				Baud = 115200;
			end
			me.portObj = serial(devname,'BaudRate',Baud, 'Databits', 8, 'Parity', 'none', 'StopBits', 1, 'InputBufferSize', 1024, 'OutputBufferSize', 1024);
			me.portObj.TimeOut = 1;
			
			fopen(me.portObj);
			
			if strcmp(me.portObj.Status,'open')
				fprintf('%s is open\n', devname);
				me.connected = true;
				flushinput(me.portObj);
				flushoutput(me.portObj);
			else
				fprintf('%s not found or in use', devname);
				me.connected = false;
			end
			format hex;
		end
		
		%{
        *************************************************************
         @fn disconnect()
         @brief closes epos port and sets format to short (default matlab)
        *************************************************************
		%}
		function disconnect(me)
			if(me.connected)
				fclose(me.portObj);
				me.connected = false;
			end
			format short;
		end
		
		%{
        *************************************************************
            basic I/O functions
        *************************************************************
		%}
		%{
        *************************************************************
         @fn writeBYTE(myByte)
         @brief send a byte to epos
         @param myByte byte to be sent to epos
         @retval  OK         a boolean if write was sucessfull or not
        *************************************************************
		%}
		
		function [OK] = writeBYTE(me, myByte)
			% write single byte to EPOS
			if ~me.connected
				fprintf('[Epos.writeBYTE]: Port "%s" is not open\n', me.portObj.Port);
				OK = 0;
				return;
			end
			nBytes = me.portObj.ValuesSent;
			if me.debug_flag
				fprintf('[Epos.writeBYTE] >> Sending byte value: 0x%02X\n', myByte);
			end
			fwrite(me.portObj, uint8(myByte), 'uint8');
			if(me.portObj.ValuesSent-nBytes)~=1
				OK = 0;
				return;
			else
				OK =1;
				return;
			end
		end
		
		%{
        *************************************************************
         @fn writeWORD(myWord)
         @brief send a Word (2 bytes) to epos
         @param myWord byte to be sent to epos
         @retval  OK          a boolean if write was sucessfull or not
        *************************************************************
		%}
		function [OK]= writeWORD(me, myWord)
			%  write a single WORD to EPOS
			if ~me.connected
				fprintf('[Epos.writeWORD]: Port "%s" is not open\n', me.portObj.Port);
				OK = 0;
				return;
			end
			nBytes = me.portObj.ValuesSent;
			if me.debug_flag
				fprintf('[Epos.writeWORD] >> Sending WORD value: 0x%04X\n', uint16(myWord));
			end
			myWord = typecast(myWord,'uint8');
			% send lowByte
			fwrite(me.portObj, myWord(1), 'uint8');
			if me.debug_flag
				fprintf('[Epos.writeWORD] >> Sending low Byte value: 0x%02X\n', myWord(1));
			end
			% send HighByte
			fwrite(me.portObj, myWord(2), 'uint8');
			if me.debug_flag
				fprintf('[Epos.writeWORD] >> Sending High Byte value: 0x%02X\n', myWord(2));
			end
			if(me.portObj.ValuesSent-nBytes)~=2
				OK = 0;
				return;
			else
				OK =1;
				return;
			end
		end
		
		%{
        *************************************************************
         @fn readBYTE()
         @brief read a byte from epos
         @retval myByte byte read from epos
         @retval OK         a boolean if write was sucessfull or not
        *************************************************************
		%}
		function [myByte, OK] = readBYTE(me)
			% read single byte to EPOS
			if ~me.connected
				fprintf('[Epos.readBYTE]: Port "%s" is not open\n', me.portObj.Port);
				OK = 0;
				return;
			end
			[myByte, nBytes] = fread(me.portObj, 1,'uint8');
			myByte = uint8(myByte);
			if me.debug_flag
				if nBytes
					fprintf('[EPOS.readBYTE]<< Reading byte value: 0x%02X\n', myByte);
				else
					fprintf('[EPOS.readBYTE]<< Failed to readBYTE: timeout\n');
				end
			end
			if(nBytes)~=1
				OK = 0;
				return;
			else
				OK =1;
				return;
			end
		end
		
		%{
        *************************************************************
         @fn readWORD()
         @brief read a word (2 Bytes) from epos
         @retval myWord word read from epos
         @retval OK         a boolean if write was sucessfull or not
        *************************************************************
		%}
		function [myWord, OK] = readWORD(me)
			myWord =[];
			if ~me.connected
				fprintf('[Epos.readWORD]: Port "%s" is not open\n', me.portObj.Port);
				OK = 0;
				return;
			end
			[myWord, nBytes] = fread(me.portObj, 1,'uint16');
			myWord = uint16(myWord);
			myWord = swapbytes(myWord);
			if me.debug_flag
				if nBytes
					fprintf('[Epos.readWORD] << reading word value: 0x%04X\n',myWord);
				else
					fprintf('[EPOS.readWORD]<< Failed to readWORD: timeout\n');
				end
			end
			if(nBytes)~=1
				OK = 0;
				return;
			else
				OK =1;
				return;
			end
		end
		%{
        **************************************************************************

                                         End of low level I/O functions
        
       **************************************************************************
		%}
		
		%{
		************************************************************************

			Set of basic functions for comunication

		************************************************************************
		%}

		function [Answer, NumWords] = readAnswer(me)
			% Read an answer from a request
			% 
			Answer = [];
			NumWords = 0;
			[newByte, OK] = me.readBYTE();
			if OK
				if (newByte ~= me.ResponseCodes('E_ANS'))
					fprintf('[Epos.readAnswer]: Epos sent 0x%02X while was expecting a Answer frame start "0x%02X"\n'...
						,newByte, me.ResponseCodes('E_ANS'));
					return;
				else
					%Always ready, send ok in advance!
					me.writeBYTE(me.ResponseCodes('E_OK'));
					% get len-1
					[len_1, OK] = me.readBYTE();
					if (OK)
						NumWords = len_1+3;
						Answer = uint16(zeros(1,NumWords));
						Answer(1) = typecast([newByte len_1], 'uint16');
						%read len_1 +1 data words + crc = NumWords-1
						for index = 1:1:NumWords-1;
							[Answer(index+1), ~] = me.readWORD();
						end
						crcMatch = me.CRCCheck(Answer);
						if crcMatch
							me.writeBYTE(me.ResponseCodes('E_OK'));
						else
							me.writeBYTE(me.ResponseCodes('E_FAIL'));
						end
					end
				end
			end
		end
		
		%{
        *************************************************************
        @fn CRCcalc(DataArray, CRCnumberOfWords)
        @brief calculate 16 bit CRC checksum
        
        CRCcalc calculates the CRC of frame message, wich is made of:
        [header][DATA][CRC = 0]
        For correct crc calculation, the last word (CRC field) must be zero.
        
        @param DataArray frame to be checked
        @param CRCnumberOfWords number of words (word = 2 bytes) present in frame
        @revalt CRC_OK a boolean if crc is match or not
         *************************************************************
		%}

		function [ CRC ] = CRCcalc(~, DataArray, CRCnumberOfWords)
			%Calculates the 16 bit CRC checksum according to communication guide for
			%EPOS
			%Arguments are the DataArray that is being sent (opCode to Data) and the
			%number of words
			
			CRC = uint16(0);
			for i=1:CRCnumberOfWords
				%shifter = uint16(hex2dec('8000'));
				shifter = uint16(32768);
				c = DataArray(i);
				
				while(shifter)
					%carry = bitand(CRC, uint16(myhex2dec('8000')), 'uint16');
					carry = bitand(CRC, uint16(32768), 'uint16');
					CRC = bitshift(CRC,1,'uint16');
					if(bitand(c, shifter, 'uint16'))
						CRC = CRC +  1;
					end
					if(carry)
						%CRC = bitxor(CRC,uint16(myhex2dec('1021')),'uint16');
						CRC = bitxor(CRC,uint16(4129),'uint16');
					end
					shifter = bitshift(shifter,-1,'uint16');
				end
			end
		end
		
		%{
        *************************************************************
        @fn CRCCheck(DataArray)
        @brief check if crc is correct
        
        CRCCecheck extracts the CRC received on message (last word of
        array) replaces it to zero and calculates the new crc over all
        array. After it compares value received with the new one
        calculated.
        
        @param DataArray frame to be checked
        @revalt CRC_OK a boolean if crc is match or not
         *************************************************************
		%}
		function [CRC_OK] = CRCCheck(me, DataArray)
			numWords = length(DataArray);
			DataArray = swapbytes(DataArray);
			crcReceived = uint16(DataArray(numWords));
			DataArray(numWords) = 0;
			newCrc = me.CRCcalc(DataArray,numWords);
			
			if newCrc == crcReceived
				CRC_OK = true;
			else
				CRC_OK = false;
			end
		end

		%{
        *************************************************************
        @fn sendCom(DataArray, numWords)
        @brief send command to EPOS
        
        Send command to EPOS, taking care of all necessary 'ack' and 
        checksum tests.
        
        @param DataArray frame to be sent.
		@param numWords number of words present in the frame
        @revalt OK boolean if all went ok or not
         *************************************************************
		%}
		function [OK] = sendCom(me, DataArray, numWords)
			% calculate CRC
			CrcValue = me.CRCcalc(DataArray,numWords);
			DataArray(numWords) = CrcValue;
			% start sending frame
			if me.debug_flag
				fprintf('[Epos.SendCom] >> sending frame to Epos:\n');
				disp(DataArray);
			end
			header = typecast(DataArray(1),'uint8');
			OK = 0;
			retries = 0;
			while(OK == 0)
				retries = retries +1;
				if(retries>5)
					return;
				end
				if(retries>1)
					flushinput(me.portObj);
					pause(0.1);
				end
				% send OpCode to EPOS
				if (~me.writeBYTE(header(2)))
					fprintf('[Epos.SendCom] Error sending byte "OPCode": 0x%02X\n',header(2));
					return;
				end
				% wait for "ready Ack" 'O'
				[responseByte, OK] = me.readBYTE();
				if (OK ~= true)
					fprintf('[Epos.SendCom]: failed to receive ready Ack...retries %d\n', retries);
				end
			end
			if responseByte ~= me.ResponseCodes('E_OK')
				OK = false;
				fprintf(['[Epos.sendCom]: EPOS not ready, reply was 0x%02X\n',responseByte]);
				return;	
			end
			% all ok, send rest of header (len-1)
			if (~me.writeBYTE(header(1)))
				fprintf('[Epos.SendCom] Error sending byte "Len-1": 0x%02X\n',header(1));
				return;
			end	
			% send the rest of words
			for I=2:1:numWords
				me.writeWORD(DataArray(I));
			end
			% wait for "End Ack" 'O'
			[responseByte, OK] = me.readBYTE();
			if responseByte == me.ResponseCodes('E_OK')
				OK = true;
				return;
			end
			OK = false;
			fprintf('[Epos.SendCom]: EPOS CRCError, reply was 0x%02X\n',responseByte);
		end
		
		%{
		@fn readObject(index, subindex)
		@brief reads an object from dictionary 

		Request a read from dictionary object referenced by index and subindex.

		@param index reference of dictionary object index
		@param subindex reference of dictionary object subindex
		@retval Answer message returned by EPOS or empty if unsucessful
		@retval OK boolean if sucessful communication or not
	
		%}
		
		function [Answer, OK] = readObject(me, index, subindex)
			validateattributes(index,{'uint16'},{'scalar'});
			subindex = uint8(subindex);
			header = hex2dec('1001'); % Allways fixed OpCode = 10, len-1 = 1
			frame =uint16(zeros(4,1));
			frame(1) = header;
			frame(2) = index;
			frame(3) = typecast(uint8([subindex me.nodeID]), 'uint16'); 
			
			OK = me.sendCom(frame,4);
			if OK == true
				[Answer, ~] = me.readAnswer();
				if isempty(Answer)
					OK = false;
					return;
				end
				% do not forget to swapbytes!
				Answer = swapbytes(Answer);
				return;
			else
				Answer = [];
				return;
			end
		end
		
		%{
		@fn writeObject(index, subindex, data)
		@brief write an object from dictionary 

		Request a write to dictionary object referenced by index and subindex.

		@param index reference of dictionary object index
		@param subindex reference of dictionary object subindex
		@param data array to be stored in object
		@retval Answer message returned by EPOS or empty if unsucessful
		@retval OK boolean if sucessful communication or not
	
		%}
		function [Answer, OK] = writeObject(me, index, subindex,data)
			validateattributes(index,{'uint16'},{'scalar'});
			header = hex2dec('1103'); % allways fixed OpCode = 11, len-1 3
			frame = uint16(zeros(6,1));
			frame(1) = header;
			frame(2) = index;
			frame(3) = typecast(uint8([subindex me.nodeID]), 'uint16');
			frame(4) = data(1);
			frame(5) = data(2);
			
			OK = me.sendCom(frame,6);
			if OK == true
				[Answer, ~] = me.readAnswer();
				if isempty(Answer)
					OK = false;
					return;
				end
				Answer = swapbytes(Answer);
				return;
			else
				Answer = [];
				return;
			end
		end

		%{
		@fn checkError(E_error)
		@brief check if any error occurred in message received 

		When you send a request to EPOS, the returned response frame, contains a
		data field wich stores information of errors if any. The corresponding 
		message of error explaining it is printed.

		@param error data field from EPOS
		@retval anyError boolean representing if any error happened.
	
		%}
		function [anyError] = checkError(me, E_error)
			anyError = true;
			E_error = typecast(E_error, 'uint32');

			% CANopen defined error codes */
			E_NOERR 	  = hex2dec('00000000');  % Error code: no error
			E_ONOTEX      = hex2dec('06020000');  % Error code: object does not exist
			E_SUBINEX     = hex2dec('06090011');  % Error code: subindex does not exist
			E_OUTMEM      = hex2dec('05040005');  % Error code: out of memory
			E_NOACCES     = hex2dec('06010000');  % Error code: Unsupported access to an object
			E_WRITEONLY   = hex2dec('06010001');  % Error code: Attempt to read a write-only object
			E_READONLY    = hex2dec('06010002');  % Error code: Attempt to write a read-only object
			E_PARAMINCOMP = hex2dec('06040043');  % Error code: general parameter incompatibility 
			E_INTINCOMP   = hex2dec('06040047');  % Error code: general internal incompatibility in the device 
			E_HWERR       = hex2dec('06060000');  % Error code: access failed due to an hardware error
			E_PRAGNEX     = hex2dec('06090030');  % Error code: value range of parameter exeeded
			E_PARHIGH     = hex2dec('06090031');  % Error code: value of parameter written is too high
			E_PARLOW      = hex2dec('06090032');  % Error code: value of parameter written is too low
			E_PARREL      = hex2dec('06090036');  % Error code: maximum value is less than minimum value

			% maxon specific error codes */
			E_NMTSTATE = hex2dec('0f00ffc0'); % Error code: wrong NMT state
			E_RS232    = hex2dec('0f00ffbf'); % Error code: rs232 command illegeal
			E_PASSWD   = hex2dec('0f00ffbe'); % Error code: password incorrect
			E_NSERV    = hex2dec('0f00ffbc'); % Error code: device not in service mode
			E_NODEID   = hex2dec('0f00fb9 '); % Error code: error in Node-ID
			
			switch E_error 
				case E_NOERR
					anyError = false;
				case E_ONOTEX
					fprintf('[EPOS checkError] EPOS responds with error: requested object does not exist!\n');
				case E_SUBINEX
					fprintf('[EPOS checkError] EPOS responds with error: requested subindex does not exist!\n');
				case E_OUTMEM
					fprintf('[EPOS checkError] EPOS responds with error: out of memory!\n');
				case E_NOACCES
					fprintf('[EPOS checkError] EPOS responds with error: unsupported access to an object!\n');
				case E_WRITEONLY
					fprintf('[EPOS checkError] EPOS responds with error: attempt to read a write-only object!\n');
				case E_READONLY
					fprintf('[EPOS checkError] EPOS responds with error: attempt to write a read-only object!\n');
				case E_PARAMINCOMP
					fprintf('[EPOS checkError] EPOS responds with error: general parameter incompatibility!\n');
				case E_INTINCOMP
					fprintf('[EPOS checkError] EPOS responds with error: general internal incompatibility in the device!\n');
				case E_HWERR
					fprintf('[EPOS checkError] EPOS responds with error: access failed due to an HARDWARE ERROR!\n');
				case E_PRAGNEX
					fprintf('[EPOS checkError] EPOS responds with error: value range of parameter exeeded!\n');
				case E_PARHIGH
					fprintf('[EPOS checkError] EPOS responds with error: value of parameter written is too high!\n');
				case E_PARLOW
					fprintf('[EPOS checkError] EPOS responds with error: value of parameter written is too low!\n');
				case E_PARREL
					fprintf('[EPOS checkError] EPOS responds with error: maximum value is less than minimum value!\n');
				case E_NMTSTATE
					fprintf('[EPOS checkError] EPOS responds with error: wrong NMT state!\n');
				case E_RS232
					fprintf('[EPOS checkError] EPOS responds with error: rs232 command illegeal!\n');
				case E_PASSWD
					fprintf('[EPOS checkError] EPOS responds with error: password incorrect!\n');
				case E_NSERV
					fprintf('[EPOS checkError] EPOS responds with error: device not in service mode!\n');
				case E_NODEID
					fprintf('[EPOS checkError] EPOS responds with error: error in Node-ID!\n');
				otherwise
					fprintf('[EPOS checkError] EPOS responds with error: unknown EPOS error code: 0x%08X\n', E_error);
			end
		end
		
		%{
		************************************************************************

			End of basic functions for comunication

		************************************************************************
		%}

		%{
		************************************************************************
			
			High level functions

		************************************************************************
		%}
		
		%{
		@fn [state, ID, ok]=checkEposState()
		@brief check current state of Epos 

		Ask the StatusWord of EPOS and parse it to return the current state of
		EPOS.
		
		|State                 			 | ID  | Statusword [binary] |
		|:------------------------------:|:---:|:-------------------:|
		| Start 	           			 | 0   | x0xx xxx0  x000 0000|
		|Not Ready to Switch On			 | 1   | x0xx xxx1  x000 0000|
		|Switch on disabled	   			 | 2   | x0xx xxx1  x100 0000|
		|ready to switch on    			 | 3   | x0xx xxx1  x010 0001|
		|switched on           			 | 4   | x0xx xxx1  x010 0011|
		|refresh               			 | 5   | x1xx xxx1  x010 0011|
		|measure init          			 | 6   | x1xx xxx1  x011 0011|
		|operation enable      			 | 7   | x0xx xxx1  x011 0111|
		|quick stop active     			 | 8   | x0xx xxx1  x001 0111|
		|fault reaction active (disabled)| 9   | x0xx xxx1  x000 1111|
		|fault reaction active (enabled) | 10  | x0xx xxx1  x001 1111|
		|Fault       					 | 11  | x0xx xxx1  x000 1000|

		see section 8.1.1 of firmware manual for more details.

		@retval state text with current Epos state.
		@retval ID numeric identification of the state
		@retval OK boolean if corrected received status word or not
	
		%}

		function [state, ID, OK] = checkEposState(me)
			[statusWord, OK] = me.readStatusWord();
			if ~OK
				fprintf('[Epos checkEposState]: Failed to read StatusWord\n');
				return;
			end
			% state 'start' (0)
			% statusWord == x0xx xxx0  x000 0000
			bitmask = uint16(bin2dec('0100 0001 0111 1111'));
			if(bitand(bitmask,statusWord) == 0) 
				state = 'start'; 
				ID = 0;
				return;
			end
			% state 'not ready to switch on' (1)
			% statusWord == x0xx xxx1  x000 0000
			bitmask = uint16(bin2dec('0100 0001  0111 1111'));
			if(bitand(bitmask,statusWord) == 256)
				state = 'not ready to switch on';
				ID = 1;
				return;
			end
			% state 'switch on disabled' (2)
			% statusWord == x0xx xxx1  x100 0000
			bitmask = uint16(bin2dec('0100 0001  0111 1111'));
			if(bitand(bitmask,statusWord) == 320)
				state = 'switch on disable';
				ID = 2;
				return;
			end
			% state 'ready to switch on' (3)
			% statusWord == x0xx xxx1  x010 0001
			bitmask = uint16(bin2dec('0100 0001  0111 1111'));
			if(bitand(bitmask,statusWord) == 289)
				state = 'ready to switch on';
				ID = 3;
				return;
			end
			% state 'switched on' (4)
			% statusWord == x0xx xxx1  x010 0011
			bitmask = uint16(bin2dec('0000 0001  0111 1111'));
			if(bitand(bitmask,statusWord) == 291)
				state = 'switched on';
				ID = 4;
				return;
			end
			% state 'refresh' (5)
			% statusWord == x1xx xxx1  x010 0011
			bitmask = uint16(bin2dec('0100 0001  0111 1111'));
			if(bitand(bitmask,statusWord) == 16675)
				state = 'refresh';
				ID = 5;
				return;
			end
			% state 'measure init' (6)
			% statusWord == x1xx xxx1  x011 0011
			bitmask = uint16(bin2dec('0100 0001  0111 1111'));
			if(bitand(bitmask,statusWord) == 16691)
				state = 'measure init';
				ID = 6;
				return;
			end
			% state 'operation enable' (7)
			% statusWord == x0xx xxx1  x011 0111
			bitmask = uint16(bin2dec('0100 0001  0111 1111'));
			if(bitand(bitmask,statusWord) == 311)
				state = 'operation enable';
				ID = 7;
				return;
			end
			% state 'Quick Stop Active' (8)
			% statusWord == x0xx xxx1  x001 0111
			bitmask = uint16(bin2dec('0100 0001  0111 1111'));
			if(bitand(bitmask,statusWord) == 279)
				state = 'quick stop active';
				ID = 8;
				return;
			end
			% state 'fault reaction active (disabled)' (9)
			% statusWord == x0xx xxx1  x000 1111
			bitmask = uint16(bin2dec('0100 0001  0111 1111'));
			if(bitand(bitmask,statusWord) == 271)
				state = 'fault reaction active (disabled)';
				ID = 9;
				return;
			end
			% state 'fault reaction active (enabled)' (10)
			% statusWord == x0xx xxx1  x001 1111
			bitmask = uint16(bin2dec('0100 0001  0111 1111'));
			if(bitand(bitmask,statusWord) == 287)
				state = 'fault reaction active (enable)';
				ID = 10;
				return;
			end
			% state 'fault' (11)
			% statusWord == x0xx xxx1  x000 1000
			bitmask = uint16(bin2dec('0100 0001  0111 1111'));
			if(bitand(bitmask,statusWord) == 264)
				state = 'fault';
				ID = 11;
				return;
			end
		end
		
		function [OK] = changeEposState(me, state)
			data = uint16([0 0]);
			index = me.objectIndex('Controlword');
			subindex = uint8(hex2dec('0'));
			% shoudl I read current control word????
			
			switch state
				case 'shutdown'
					% shutdown, controlword: 0xxx x110
					% set bits
					data(1) = bitor(data(1),bin2dec('00000110'));
					[Answer, OK] = me.writeObject(index, subindex, data);
					if ~OK
						%todo
					end
						
				case 'switch on'
					% switch on, controlword: 0xxx x111
					% set bits
					data(1) = bitor(data(1), bin2dec('00000111'));
					[Answer, OK] = me.writeObject(index, subindex, data);
					if ~OK
						% todo
					end
				case 'disable voltage'
					% disable voltage, controlword 0xxx xx0x
					% already zeros so nothing to change.
					[Answer, OK] = me.writeObject(index, subindex, data);
					if ~OK
						% todo
					end
				case 'quick stop'
					% quick stop, controllword: 0xxx x01x
					% set bits
					data(1) = bitor(data(1), bin2dec('00000010'));
					[Answer, OK] = me.writeObject(index, subindex, data);
					if ~OK
						% todo
					end
				case 'enable operation'
					% enable operation, controlword: 0xxx 1111
					% set bits
					data(1) = bitor(data(1), bin2dec('00001111'));
					[Answer, OK] = me.writeObject(index, subindex, data);
					if ~OK
						% todo
					end
				case 'disable operation'
					% disable operation, controlword: 0xxx 0111
					% already zeros so nothing to change.
					[Answer, OK] = me.writeObject(index, subindex, data);
					if ~OK
						% todo
					end
					
				case 'fault reset'
					% fault reset, controlword: 1xxx xxxx
					% set bits
					data(1) = bitor(data(1), bin2dec('10000000'));
					[Answer, OK] = me.writeObject(index, subindex, data);
					
					if ~OK
						% todo
					else
						E_error = me.checkError(Answer(2:3));
						%check for errors
						% current position
					end
				otherwise
					fprintf('ERROR: demanded state %d is UNKNOWN!\n', state);
					OK = false;
			end
		end
		
		function [Answer, OK] = readStatusWord(me)
			index = me.objectIndex('Statusword');
			subindex = uint8(0);
			[Answer, sucess] = me.readObject(index, subindex);
			if (sucess)
				E_error = me.checkError(Answer(2:3));
				if E_error == 0
					Answer = Answer(4);
					OK = true;
					return
				else
					Answer = 'error';
					OK = false;
				end
			else
				Answer = [];
				OK = false;
			end
		end

		function [] = printStatusWord(me, statusWord)
			statusWord = uint16(statusWord);
			fprintf('[EPOS printStatusWord] meaning of statusWord 0x%04X is\n', statusWord)
			statusWord = dec2bin(statusWord, 16);
			fprintf('Bit 15: position referenced to home position:                  %s\n', statusWord(1));
			fprintf('Bit 14: refresh cycle of power stage:                          %s\n', statusWord(2));
			fprintf('Bit 13: OpMode specific, some error: [Following|Homing]        %s\n', statusWord(3));
			fprintf('Bit 12: OpMode specific: [Set-point ack|Speed|Homing attained] %s\n', statusWord(4));
			fprintf('Bit 11: Internal limit active:                                 %s\n', statusWord(5));
			fprintf('Bit 10: Target reached:                                        %s\n', statusWord(6));
			fprintf('Bit 09: Remote (NMT Slave State Operational):                  %s\n', statusWord(7));
			fprintf('Bit 08: Offset current measured:                               %s\n', statusWord(8));
			fprintf('Bit 07: not used (Warning):                                    %s\n', statusWord(9));
			fprintf('Bit 06: Switch on disable:                                     %s\n', statusWord(10));
			fprintf('Bit 05: Quick stop:                                            %s\n', statusWord(11));
			fprintf('Bit 04: Voltage enabled (power stage on):                      %s\n', statusWord(12));
			fprintf('Bit 03: Fault:                                                 %s\n', statusWord(13));
			fprintf('Bit 02: Operation enable:                                      %s\n', statusWord(14));
			fprintf('Bit 01: Switched on:                                           %s\n', statusWord(15));
			fprintf('Bit 00: Ready to switch on:                                    %s\n', statusWord(16));
		end
		
		function [Answer, OK] = readControlWord(me)
			index = me.objectIndex('Controlword');
			subindex = uint8(0);
			[Answer, sucess] = me.readObject(index, subindex);
			if (sucess)
				E_error = me.checkError(Answer(2:3));
				if E_error == 0
					Answer = Answer(4);
					OK = true;
					return
				else
					Answer = 'error';
					OK = false;
				end
			else
				Answer = [];
				OK = false;
			end
		end
		
		function [] = printControlWord(me, controlWord)
			controlWord = uint16(controlWord);
			fprintf('Epos printControlWord] meaning of controlWord: 0x%04X\n', controlWord);
			% bit 15..11 not in use
  			% bit 10, 9 reserved
			controlWord = dec2bin(controlWord,16);
			fprintf('Bit 08: Halt:                                                                   %s\n', controlWord(8));
			fprintf('Bit 07: Fault reset:                                                            %s\n', controlWord(9));
			fprintf('Bit 06: Operation mode specific:[Abs|rel]                                       %s\n', controlWord(10));
			fprintf('Bit 05: Operation mode specific:[Change set immediately]                        %s\n', controlWord(11));
			fprintf('Bit 04: Operation mode specific:[New set-point|reserved|Homing operation start] %s\n', controlWord(12));
			fprintf('Bit 03: Enable operation:                                                       %s\n', controlWord(13));
			fprintf('Bit 02: Quick stop:                                                             %s\n', controlWord(14));
			fprintf('Bit 01: Enable voltage:                                                         %s\n', controlWord(15));
			fprintf('Bit 00: Switch on:                                                              %s\n', controlWord(16));
		end
		
		
		function [SWversion,sucess] = readSWversion(me)
			if(~me.connected)
				SWversion = 'none';
				sucess = false;
				return;
			else
				index = me.objectIndex('Version');
				[SWversion, sucess] = me.readObject(index,1);
				if (sucess)
					E_error = me.checkError(SWversion(2:3));
					if E_error == 0
						SWversion = SWversion(4);
					else
						SWversion = 'error';
						sucess = false;
					end
				else
					SWversion = 'none';
				end
			end
		end
		
		
		function [devname,sucess] = readDeviceName(me)
			if(~me.connected)
				devname = 'none';
				sucess = false;
				return;
			else
				[devname, sucess] = me.readObject(me.objectIndex('ManufacturerDeviceName'),0);
				if(sucess)
					E_error = me.checkError(devname(2:3));
					if E_error == 0
						devname = typecast(devname(4:5), 'uint8');
						devname = char(devname);
					else
						devname = 'error'
						sucess = false;
					end
				else
					devname = 'none';
					sucess = false;
				end
			end
		end

		function [] = setMotorConfig(me, motorType, currentLimit, maximumSpeed, polePairNumber)
			%
			% set motor type
			index = me.objectIndex('MotorType');
			subindex = uint8(0);
			motorType = uint16([motorType 0]);

			[Answer, OK] = me.writeObject(index, subindex, motorType);
			if ~OK
				% todo
			else
				E_error = me.checkError(Answer(2:3));
				%check for errors
			end
			% set continuous current limit
			% This object represents the maximal permissible continuous current of the motor [mA]
			index = me.objectIndex('MotorData');
			subindex = uint8(1);
			currentLimit = uint16([currentLimit 0]);
			[Answer, OK] = me.writeObject(index, subindex, currentLimit);
			if ~OK
				% todo
			else
				E_error = me.checkError(Answer(2:3));
				%check for errors
			end
			% set output current limit
			% It is recommended to set the output current limit to a value doubles of continuous current limit [mA].
			subindex = uint8(2);
			outputCurrentLimit = uint16([2*currentLimit 0]);
			[Answer, OK] = me.writeObject(index, subindex, outputCurrentLimit);
			if ~OK
				% todo
			else
				E_error = me.checkError(Answer(2:3));
				%check for errors
			end
			% Number of magnetic pole pairs (number of poles / 2) from rotor of a brushless DC motor.
			subindex = uint8(3);
			polePairNumber = uint16([polePairNumber 0]);
			[Answer, OK] = me.writeObject(index, subindex, polePairNumber);
			if ~OK
				% todo
			else
				E_error = me.checkError(Answer(2:3));
				%check for errors
			end
			% To prevent mechanical destroys in current mode it is possible to limit the velocity [rpm].
			subindex = uint8(4);
			maximumSpeed = uint16([maximumSpeed 0]);
			[Answer, OK] = me.writeObject(index, subindex, maximumSpeed);
			if ~OK
				% todo
			else
				E_error = me.checkError(Answer(2:3));
				%check for errors
			end
		end

		function [] = setSensorConfig(me, pulseNumber, sensorType, sensorPolarity)
			% todo validate attributes

			index = me.objectIndex('SensorConfiguration');
			
			%{
				The encoder pulse number should be set to number of counts per revolution of the connected incremental
				encoder.
				Minimal Value: 16 pulse per turn
				Maximal Value: 7500 pulse per turn
			%}

			subindex = uint8(1);
			pulseNumber = uint16([pulseNumber 0]);
			[Answer, OK] = me.writeObject(index, subindex, pulseNumber);
			if ~OK
				% todo
			else
				E_error = me.checkError(Answer(2:3));
				%check for errors
			end
			
			% The position sensor type can be changed with this parameter.

			subindex = uint8(2);
			sensorType = uint16([sensorType 0]);
			[Answer, OK] = me.writeObject(index, subindex, sensorType);
			if ~OK
				% todo
			else
				E_error = me.checkError(Answer(2:3));
				%check for errors
			end

			% With this parameter the position sensor and the hall sensor polarity can be changed.
			subindex = uint8(4);
			sensorPolarity = uint16([sensorPolarity 0]);
			[Answer, OK] = me.writeObject(index, subindex, sensorPolarity);
			if ~OK
				% todo
			else
				E_error = me.checkError(Answer(2:3));
				%check for errors
			end

		end
	end
end