classdef Epos < handle
    %% Class for Maxton EPOS 70/10 motor control
    %
    % Class to control Epos device is based on libepos library originally 
    % developed by Marcus Hauser, found here:
    % https://sourceforge.net/projects/libepos/
    % 
    %     
    % Class is not yet fully tested and is a work in progress.
    %     
    % Title:  Epos.m
    % Author: Bruno Tibério
    % Date:   September 2016
    % email:  bruno.tiberio@tecnico.ulisboa.pt
    %     
    % BSD 2-Clause License
    %     
    % Copyright (c) 2016, Bruno Tibério
    % All rights reserved.
    %     
    % Redistribution and use in source and binary forms, with or without
    % modification, are permitted provided that the following conditions are met:
    %     
    % * Redistributions of source code must retain the above copyright notice, this
    % list of conditions and the following disclaimer.
    %     
    % * Redistributions in binary form must reproduce the above copyright notice,
    % this list of conditions and the following disclaimer in the documentation
    % and/or other materials provided with the distribution.
    %        
    % THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    % AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    % IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    % DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    % FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    % DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    % SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    % CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    % OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    % OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    %
    
    properties
        portObj;
        nodeID;
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
			if me.debug_flag
				format hex;
			end
        end
        
        function [] = delete(me)
            me.disconnect();
        end
        
        %=======================================================================
        %> @fn begin(devname, Baud)
        %> @brief connects to Epos
        %>
        %> Establish the connection to EPOS via RS232 connection
        %> Sets connected if configuration was sucessfull or not.
        %>
        %> NOTE: it changes format to hex for easier visualization of message
        %> transaction.
        %>
        %> @param devname Portname for the device (example: '/dev/ttyUSB0')
        %> @param Baud [optional] baudrate for the communication (default
        %> 115200)
        %=======================================================================
        
        function [OK] = begin(me, devname, Baud)
            % begin(devname, Baud)
			if ~exist('Baud', 'var')
				Baud = 115200;
			end
			% already open?
			if me.connected
				fprintf('[Epos begin] Already connected\n');
				OK = true;
				return;
			end
			% if not
            me.portObj = serial(devname,'BaudRate',Baud, 'Databits', 8, 'Parity', 'none', 'StopBits', 1, 'InputBufferSize', 1024, 'OutputBufferSize', 1024);
            me.portObj.TimeOut = 1;
            
            fopen(me.portObj);
            
            if strcmp(me.portObj.Status,'open')
                fprintf('%s is open\n', devname);
                me.connected = true;
                flushinput(me.portObj);
                flushoutput(me.portObj);
				OK = true;
            else
                fprintf('%s not found or in use', devname);
                me.connected = false;
				OK = false;
			end
        end
        
        
        %=======================================================================
        %> @fn disconnect()
        %> @brief closes epos port and sets format to short (default matlab)
        %=======================================================================
        
        function disconnect(me)
            if(me.connected)
                fclose(me.portObj);
                me.connected = false;
			end
			if me.debug_flag
				format short;
			end
            
        end
        
        %=======================================================================
        %    basic I/O functions
        %=======================================================================
        
        
        %=======================================================================
        %> @fn writeBYTE(myByte)
        %> @brief send a byte to epos
        %>
        %> @param myByte byte to be sent to epos
        %> @retval  OK   a boolean if write was sucessfull or not
        %=======================================================================
        
        
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
        
        
        %=======================================================================
        %> @fn writeWORD(myWord)
        %> @brief send a Word (2 bytes) to epos
        %>
        %> @param myWord byte to be sent to epos
        %> @retval  OK   a boolean if write was sucessfull or not
        %=======================================================================
        
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
        
        
        %=======================================================================
        %> @fn readBYTE()
        %> @brief read a byte from epos
        %>
        %> @retval myByte byte read from epos
        %> @retval OK     a boolean if write was sucessfull or not
        %=======================================================================
        
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
        
        
        %=======================================================================
        %> @fn readWORD()
        %> @brief read a word (2 Bytes) from epos
        %>
        %> @retval myWord word read from epos
        %> @retval OK         a boolean if write was sucessfull or not
        %=======================================================================
        
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
        %=======================================================================

                    End of low level I/O functions
       
        %=======================================================================
        %}
        
        %{
        %=======================================================================

            Set of basic functions for comunication

        %=======================================================================
        %}
        
        function [answer, NumWords] = readAnswer(me)
            % Read an answer from a request
            %
            answer = [];
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
                        answer = uint16(zeros(1,NumWords));
                        answer(1) = typecast([newByte len_1], 'uint16');
                        %read len_1 +1 data words + crc = NumWords-1
                        for index = 1:1:NumWords-1;
                            [answer(index+1), ~] = me.readWORD();
                        end
                        crcMatch = me.CRCCheck(answer);
                        if crcMatch
                            me.writeBYTE(me.ResponseCodes('E_OK'));
                        else
                            me.writeBYTE(me.ResponseCodes('E_FAIL'));
                        end
                    end
                end
            end
        end
        
        
        %=======================================================================
        %> @fn CRCcalc(DataArray, CRCnumberOfWords)
        %> @brief calculate 16 bit CRC checksum
        %>
        %> CRCcalc calculates the CRC of frame message, wich is made of:
        %> [header][DATA][CRC = 0]
        %> For correct crc calculation, the last word (CRC field) must be zero.
        %>
        %> @param DataArray frame to be checked
        %> @param CRCnumberOfWords number of words (word = 2 bytes) present in frame
        %> @revalt CRC_OK a boolean if crc is match or not
        %=======================================================================
        
        
        function [ CRC ] = CRCcalc(~, DataArray, CRCnumberOfWords)
            
            CRC = uint16(0);
            for i=1:CRCnumberOfWords
                %shifter = uint16(hex2dec('8000'));
                shifter = uint16(32768);
                c = DataArray(i);
                
                while(shifter)
                    carry = bitand(CRC, uint16(32768), 'uint16');
                    CRC = bitshift(CRC,1,'uint16');
                    if(bitand(c, shifter, 'uint16'))
                        CRC = CRC +  1;
                    end
                    if(carry)
                        %CRC = bitxor(CRC,uint16(hex2dec('1021')),'uint16');
                        CRC = bitxor(CRC,uint16(4129),'uint16');
                    end
                    shifter = bitshift(shifter,-1,'uint16');
                end
            end
        end
        
        
        %=======================================================================
        %> @fn CRCCheck(DataArray)
        %> @brief check if crc is correct
        %>
        %> CRCCecheck extracts the CRC received on message (last word of
        %> array) replaces it to zero and calculates the new crc over all
        %> array. After it compares value received with the new one
        %> calculated.
        %>
        %> @param DataArray frame to be checked
        %> @revalt CRC_OK a boolean if crc is match or not
        %=======================================================================
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
        
        %=======================================================================
        %> @fn sendCom(DataArray, numWords)
        %> @brief send command to EPOS
        %>
        %> Send command to EPOS, taking care of all necessary 'ack' and
        %> checksum tests.
        %>
        %> @param DataArray frame to be sent.
        %> @param numWords  number of words present in the frame
        %> @revalt OK       boolean if all went ok or not
        %=======================================================================
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
        
        %=======================================================================
        %> @fn readObject(index, subindex)
        %> @brief reads an object from dictionary
        %>
        %> Request a read from dictionary object referenced by index and subindex.
        %>
        %> @param index    reference of dictionary object index
        %> @param subindex reference of dictionary object subindex
        %>
        %> @retval answer message returned by EPOS or empty if unsucessful
        %> @retval OK     boolean if sucessful communication or not
        %=======================================================================
        
        function [answer, OK] = readObject(me, index, subindex)
            validateattributes(index,{'uint16'},{'scalar'});
            subindex = uint8(subindex);
            header = hex2dec('1001'); % Allways fixed OpCode = 10, len-1 = 1
            frame =uint16(zeros(4,1));
            frame(1) = header;
            frame(2) = index;
            frame(3) = typecast(uint8([subindex me.nodeID]), 'uint16');
            
            OK = me.sendCom(frame,4);
            if OK == true
                [answer, ~] = me.readAnswer();
                if isempty(answer)
                    OK = false;
                    return;
                end
                % do not forget to swapbytes!
                answer = swapbytes(answer);
                return;
            else
                answer = [];
                return;
            end
        end
        
        %=======================================================================
        %> @fn writeObject(index, subindex, data)
        %> @brief write an object from dictionary
        %>
        %> Request a write to dictionary object referenced by index and subindex.
        %>
        %> @param index    reference of dictionary object index
        %> @param subindex reference of dictionary object subindex
        %> @param data     array to be stored in object
        %>
        %> @retval answer message returned by EPOS or empty if unsucessful
        %> @retval OK     boolean if sucessful communication or not
        %=======================================================================
        function [answer, OK] = writeObject(me, index, subindex,data)
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
                [answer, ~] = me.readAnswer();
                if isempty(answer)
                    OK = false;
                    return;
                end
                answer = swapbytes(answer);
                return;
            else
                answer = [];
                return;
            end
        end
        
        %=======================================================================
        %> @fn checkError(E_error)
        %> @brief check if any error occurred in message received
        %>
        %> When you send a request to EPOS, the returned response frame, contains a
        %> data field wich stores information of errors if any. The corresponding
        %> message of error explaining it is printed.
        %>
        %> @param error data field from EPOS
        %> @retval anyError boolean representing if any error happened.
        %=======================================================================
        function [anyError] = checkError(me, E_error)
            anyError = true;
            E_error = typecast(E_error, 'uint32');
            
            % CANopen defined error codes */
            E_NOERR       = hex2dec('00000000');  % Error code: no error
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
                    fprintf('[EPOS checkError] EPOS responds with error: value range of parameter exceded!\n');
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
        %=======================================================================

            End of basic functions for comunication

        %=======================================================================
        %}
        
        %{
        %=======================================================================
            
            High level functions

        %=======================================================================
        %}

        function [listErrors, anyError, OK] = checkEposError(me)

            % list of errors
            %E_NOERR = hex2dec('0000'); %not used
            E_GENERIC = hex2dec('1000');
            E_OVERCURRENT = hex2dec('2310');
            E_OVERVOLTAGE = hex2dec('3210');
            E_UNDERVOLTAGE = hex2dec('3220');
            E_OVERTEMPERATURE = hex2dec('4210');
            E_LOW5V = hex2dec('5113');
            E_INTERNALSW = hex2dec('6100');
            E_SWPARAM = hex2dec('6320');
            E_SENSORPOSITION = hex2dec('7320');
            E_CANOVERRUN_LOST = hex2dec('8110');
            E_CANOVERRUN = hex2dec('8111');
            E_CANPASSIVEMODE = hex2dec('8120');
            E_CANLIFEGUARD = hex2dec('8130');
            E_CANTRANSMITCOLLISION = hex2dec('8150');
            E_CANBUSOFF = hex2dec('81FD');
            E_CANRXOVERRUN = hex2dec('81FE');
            E_CANTXOVERRUN = hex2dec('81FF');
            E_CANPDOLENGTH = hex2dec('8210');
            E_FOLLOWING = hex2dec('8611');
            E_HALLSENSOR = hex2dec('FF01');
            E_INDEXPROCESSING = hex2dec('FF02');
            E_ENCODERRESOLUTION = hex2dec('FF03');
            E_HALLSENSORNOTFOUND = hex2dec('FF04');
            E_NEGATIVELIMIT = hex2dec('FF06');
            E_POSITIVELIMIT = hex2dec('FF07');
            E_HALLANGLE = hex2dec('FF08');
            E_SWPOSITIONLIMIT = hex2dec('FF09');
            E_POSITIONSENSORBREACH = hex2dec('FF0A');
            E_SYSTEMOVERLOADED = hex2dec('FF0B');

            % check if there are any errors
            index = me.objectIndex('ErrorHistory');
            subindex = uint8(0);

            [answer, OK] = me.readObject(index, subindex);
			if (OK)
				E_error = me.checkError(answer(2:3));
				if E_error == 0
					anyError = uint8(answer(4));
					OK = true;
				else
					listErrors = 'error';
					anyError = [];
					OK = false;
					return;
				end
			else
				listErrors = [];
				anyError = [];
				OK = false;
			end
			
            % list error(s) occurred
			if anyError == 0
                listErrors = 'No Errors'; % all OK
                return;
            else
				for I=1:anyError
					subindex = uint8(I);
					[answer, OK] = me.readObject(index, subindex);
					if (OK)
						E_error = me.checkError(answer(2:3));
						if E_error == 0
							listErrors(I) = answer(4);
						else
							listErrors(I) = uint16(-1);
						end
					else
						listErrors = uint16(-1);
					end
				end
                % replace error code with text
				temp =cellstr('');
				while(I>0)
					switch listErrors(I)
                        case -1
                            temp(I) = cellstr('Failed to read object');
                        case E_GENERIC
                            temp(I) = cellstr('Unspecific error occurred');
                        case E_OVERCURRENT
                            temp(I) = cellstr('Over Current: short circuit or not enough acceleration current');
                        case E_OVERVOLTAGE
                            temp(I) = cellstr('The power supply voltage is too high');
                        case E_UNDERVOLTAGE
                            temp(I) = cellstr('The supply voltage is too low for operation');
                        case E_OVERTEMPERATURE
                            temp(I) = cellstr('The temperature at the device power stage is too high');
                        case E_LOW5V
                            temp(I) = cellstr('There is a overload on internal generated 5V supply by the hall sensor connector or encoder connector');
                        case E_INTERNALSW
                            temp(I) = cellstr('Internal software error occurred');
                        case E_SWPARAM
                            temp(I) = cellstr('Too high Target position with too low Profile velocity');
                        case E_SENSORPOSITION
                            temp(I) = cellstr('The detected position from position sensor is no longer valid');
                        case E_CANOVERRUN_LOST
                            temp(I) = cellstr('One of the CAN mail boxes had a overflow because of too high communication rate');
                        case E_CANOVERRUN
                            temp(I) = cellstr('The execution of the CAN communication had an overrun because of too high communication rate');
                        case E_CANPASSIVEMODE
                            temp(I) = cellstr('Device changed to CAN passive Mode');
                        case E_CANLIFEGUARD
                            temp(I) = cellstr('The CANopen Life Guarding procedure has failed');
                        case E_CANTRANSMITCOLLISION
                            temp(I) = cellstr('The device has received a bad transmit PDO request');
                        case E_CANBUSOFF
                            temp(I) = cellstr('The CAN Controller has entered CAN bus off state');
                        case E_CANRXOVERRUN
                            temp(I) = cellstr('One of the CAN receive queues had a overrun because of too high communication rate');
                        case E_CANTXOVERRUN
                            temp(I) = cellstr('One of the CAN transmit queues had a overrun because of too high communication rate');
                        case E_CANPDOLENGTH
                            temp(I) = cellstr('The received PDO was not processed due to length error (to short)');
                        case E_FOLLOWING
                            temp(I) = cellstr('The difference between Position demand value and Position actual value is higher then Maximal following error');
                        case E_HALLSENSOR
                            temp(I) = cellstr('The motor hall sensors report an impossible signal combination');
                        case E_INDEXPROCESSING
                            temp(I) = cellstr('The encoder index signal was not found within two turns at start-up');
                        case E_ENCODERRESOLUTION
                            temp(I) = cellstr('The encoder pulses counted between the first two index pulses doesn’t fit to the resolution');
                        case E_HALLSENSORNOTFOUND
                            temp(I) = cellstr('No hall sensor 3 edge found within first motor turn');
                        case E_NEGATIVELIMIT
                            temp(I) = cellstr('The negative limit switch was or is active');
                        case E_POSITIVELIMIT
                            temp(I) = cellstr('The positive limit switch was or is active');
                        case E_HALLANGLE
                            temp(I) = cellstr('The angle difference measured between encoder and hall sensors is too high');
                        case E_SWPOSITIONLIMIT
                            temp(I) = cellstr('Movement commanded or actual position higher than maximal position limit or lower than minimal position limit');
                        case E_POSITIONSENSORBREACH
                            temp(I) = cellstr('The position sensor supervision has detected a bad working condition');
                        case E_SYSTEMOVERLOADED
                            temp(I) = cellstr('The device has not enough free resources to process the new target value');
                        otherwise
                            temp(I) = cellstr('UNKNOWN ERROR CODE');
					end
					I = I-1;
				end
				listErrors = temp;
			end
        end
        
        %=======================================================================
        %> @fn [state, ID, ok]=checkEposState()
        %> @brief check current state of Epos
        %>
        %> Ask the StatusWord of EPOS and parse it to return the current state of
        %> EPOS.
        %>
        %> |State                            | ID  | Statusword [binary] |
        %> |:-------------------------------:|:---:|:-------------------:|
        %> |Start                            | 0   | x0xx xxx0  x000 0000|
        %> |Not Ready to Switch On           | 1   | x0xx xxx1  x000 0000|
        %> |Switch on disabled               | 2   | x0xx xxx1  x100 0000|
        %> |ready to switch on               | 3   | x0xx xxx1  x010 0001|
        %> |switched on                      | 4   | x0xx xxx1  x010 0011|
        %> |refresh                          | 5   | x1xx xxx1  x010 0011|
        %> |measure init                     | 6   | x1xx xxx1  x011 0011|
        %> |operation enable                 | 7   | x0xx xxx1  x011 0111|
        %> |quick stop active                | 8   | x0xx xxx1  x001 0111|
        %> |fault reaction active (disabled) | 9   | x0xx xxx1  x000 1111|
        %> |fault reaction active (enabled)  | 10  | x0xx xxx1  x001 1111|
        %> |Fault                            | 11  | x0xx xxx1  x000 1000|
        %>
        %> see section 8.1.1 of firmware manual for more details.
        %>
        %> @retval state text with current Epos state.
        %> @retval ID numeric identification of the state
        %> @retval OK boolean if corrected received status word or not
        %>
        %=======================================================================
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
        %=======================================================================
        %> @fn [OK] = changeEposState(state)
        %> @brief change Epos state using controlWord object
        %> 
        %> To change Epos state, a write to controlWord object is made.
        %> The bit change in controlWord is made as shown in the following table:
        %> 
        %> |State            | LowByte of Controlword [binary]|
        %> |:---------------:|:------------------------------:|
        %> |shutdown         | 0xxx x110                      |
        %> |switch on        | 0xxx x111                      |
        %> |disable voltage  | 0xxx xx0x                      |
        %> |quick stop       | 0xxx x01x                      |
        %> |disable operation| 0xxx 0111                      |
        %> |enable operation | 0xxx 1111                      |
        %> |fault reset      | 1xxx xxxx                      |
        %> 
        %> see section 8.1.3 of firmware for more information
        %> 
        %> @param state string with state witch we want to switch.
        %>
        %> @retval OK boolean if all went ok and no error was received.
        %=======================================================================
        function [OK] = changeEposState(me, state)

            index = me.objectIndex('Controlword');
            subindex = uint8(hex2dec('0'));
            [controlWord, OK] = me.readControlWord();
			if ~OK
				fprintf('[Epos changeEposState] Failed to read controlword\n');
				return;
			end
            
            switch state
                case 'shutdown'
                    % shutdown, controlword: 0xxx x110
                    % set bits
                    controlWord = bitor(controlWord,bin2dec('0000 0110'));
                    % unset bits
                    controlWord = bitand(controlWord,bin2dec('0111 1110'));
                    [answer, OK] = me.writeObject(index, subindex, [controlWord 0]);
                    if ~OK
                        fprintf('[Epos changeEposState] Failed to write state of epos\n');
                        return;
                    else
                        OK = ~me.checkError(answer(2:3));
                        %check for errors
                        if ~OK
                            fprintf('[Epos changeEposState] Failed to set state of epos\n');
                            return;
                        end
                    end
                case 'switch on'
                    % switch on, controlword: 0xxx x111
                    % set bits
                    controlWord = bitor(controlWord, bin2dec('0000 0111'));
                    % unset bits
                    controlWord = bitand(controlWord, bin2dec('0111 1111'));
                    [answer, OK] = me.writeObject(index, subindex, [controlWord 0]);
                    if ~OK
                        fprintf('[Epos changeEposState] Failed to write state of epos\n');
                        return;
                    else
                        OK = ~me.checkError(answer(2:3));
                        %check for errors
                        if ~OK
                            fprintf('[Epos changeEposState] Failed to set state of epos\n');
                            return;
                        end
                    end
                case 'disable voltage'
                    % disable voltage, controlword 0xxx xx0x
                    % unset bits
                    controlWord = bitand(controlWord, bin2dec('0111 1101'));
                    [answer, OK] = me.writeObject(index, subindex, [controlWord 0]);
                    if ~OK
                        fprintf('[Epos changeEposState] Failed to write state of epos\n');
                        return;
                    else
                        OK = ~me.checkError(answer(2:3));
                        %check for errors
                        if ~OK
                            fprintf('[Epos changeEposState] Failed to set state of epos\n');
                            return;
                        end
                    end
                case 'quick stop'
                    % quick stop, controllword: 0xxx x01x
                    % set bits
                    controlWord = bitor(controlWord, bin2dec('0000 0010'));
                    % unset bits
                    controlWord = bitand(controlWord, bin2dec('0111 1011'));
                    [answer, OK] = me.writeObject(index, subindex, [controlWord 0]);
                    if ~OK
                        fprintf('[Epos changeEposState] Failed to write state of epos\n');
                        return;
                    else
                        OK = ~me.checkError(answer(2:3));
                        %check for errors
                        if ~OK
                            fprintf('[Epos changeEposState] Failed to set state of epos\n');
                            return;
                        end
                    end
                case 'enable operation'
                    % enable operation, controlword: 0xxx 1111
                    % set bits
                    controlWord = bitor(controlWord, bin2dec('0000 1111'));
                    % unset bits
                    controlWord = bitand(controlWord, bin2dec('0111 1111'));
                    [answer, OK] = me.writeObject(index, subindex, [controlWord 0]);
                    if ~OK
                        fprintf('[Epos changeEposState] Failed to write state of epos\n');
                        return;
                    else
                        OK = ~me.checkError(answer(2:3));
                        %check for errors
                        if ~OK
                            fprintf('[Epos changeEposState] Failed to set state of epos\n');
                            return;
                        end
                    end
                case 'disable operation'
                    % disable operation, controlword: 0xxx 0111
                    % set bits
                    controlWord = bitor(controlWord, bin2dec('0000 0111'));
                    % unset bits
                    controlWord = bitand(controlWord, bin2dec('0111 0111'));
                    [answer, OK] = me.writeObject(index, subindex, [controlWord 0]);
                    if ~OK
                        fprintf('[Epos changeEposState] Failed to write state of epos\n');
                        return;
                    else
                        OK = ~me.checkError(answer(2:3));
                        %check for errors
                        if ~OK
                            fprintf('[Epos changeEposState] Failed to set state of epos\n');
                            return;
                        end
                    end
                case 'fault reset'
                    % fault reset, controlword: 1xxx xxxx
                    % set bits
                    controlWord = bitor(controlWord, bin2dec('1000 0000'));
                    [answer, OK] = me.writeObject(index, subindex, [controlWord 0]);
                    if ~OK
                        fprintf('[Epos changeEposState] Failed to write state of epos\n');
                        return;
                    else
                        OK = ~me.checkError(answer(2:3));
                        %check for errors
                        if ~OK
                            fprintf('[Epos changeEposState] Failed to set state of epos\n');
                            return;
                        end
                    end
                otherwise
                    fprintf('[Epos ChangeState] ERROR: demanded state %s is UNKNOWN!\n', state);
                    OK = false;
            end
        end
        
        %=======================================================================
        %> @fn [answer, OK] = readSatusWord()
        %> @brief reads current status word object
        %>
        %> Ask Epos device for the current status word object. If a correct 
        %> request is made, the status word is placed in answer. 
        %>
        %> @retval answer Corresponding status word, 'error' if request was
        %>                sucessful but an error was returned or empty if request
        %>                was not sucessfull.
        %> @retval OK     A boolean if all went ok or not.
        %=======================================================================

        function [answer, OK] = readStatusWord(me)
            index = me.objectIndex('Statusword');
            subindex = uint8(0);
            [answer, sucess] = me.readObject(index, subindex);
            if (sucess)
                E_error = me.checkError(answer(2:3));
                if E_error == 0
                    answer = answer(4);
                    OK = true;
                    return
                else
                    answer = 'error';
                    OK = false;
                end
            else
                answer = [];
                OK = false;
            end
        end
        %=======================================================================
        %> @fn printStatusWord(statusWord)
        %> @brief print the meaning of the status word passed.
        %>
        %> @param statusWord The status word to print the meaning.
        %=======================================================================
        function [] = printStatusWord(me)
			
            [statusWord, OK] = me.readStatusWord();
			if OK
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
			else
				fprintf('[Epos printfStatusWord] ERROR Unable to read status word\n');
			end
        end
        
        %=======================================================================
        %> @fn [answer, OK] = readControlWord()
        %> @brief reads current control word object
        %>
        %> Ask Epos device for the current control word object. If a correct 
        %> request is made, the control word is placed in answer. If not, an answer
        %> will be empty
        %>
        %> @retval answer Corresponding control word, 'error' if request was
        %>                sucessful but an error was returned or empty if request
        %>                was not sucessfull.
        %> @retval OK     A boolean if all went ok or not.
        %=======================================================================
        function [answer, OK] = readControlWord(me)
            index = me.objectIndex('Controlword');
            subindex = uint8(0);
            [answer, sucess] = me.readObject(index, subindex);
            if (sucess)
                E_error = me.checkError(answer(2:3));
                if E_error == 0
                    answer = answer(4);
                    OK = true;
                    return
                else
                    answer = 'error';
                    OK = false;
                end
            else
                answer = [];
                OK = false;
            end
        end
        %=======================================================================
        %> @fn printControlWord(controlWord)
        %> @brief print the meaning of the control word passed.
        %>
        %> @param controlWord The control word to print the meaning.
        %=======================================================================
		function printControlWord(me)
            [controlWord, OK] = me.readControlWord();
			if OK
				fprintf('[Epos printControlWord] meaning of controlWord: 0x%04X\n', controlWord);
				% bit 15..11 not in use
				% bit 10, 9 reserved
				controlWord = dec2bin(controlWord,16);
				fprintf('Bit 08: Halt:                                                                   %s\n', controlWord(8));
				fprintf('Bit 07: Fault reset:                                                            %s\n', controlWord(9));
				fprintf('Bit 06: Operation mode specific:[Abs=0|rel=1]                                   %s\n', controlWord(10));
				fprintf('Bit 05: Operation mode specific:[Change set immediately]                        %s\n', controlWord(11));
				fprintf('Bit 04: Operation mode specific:[New set-point|reserved|Homing operation start] %s\n', controlWord(12));
				fprintf('Bit 03: Enable operation:                                                       %s\n', controlWord(13));
				fprintf('Bit 02: Quick stop:                                                             %s\n', controlWord(14));
				fprintf('Bit 01: Enable voltage:                                                         %s\n', controlWord(15));
				fprintf('Bit 00: Switch on:                                                              %s\n', controlWord(16));
			else
				fprintf('[Epos printControlWord] ERROR Unable to read control word\n');
			end
		end
        
        %=======================================================================
        %> @fn [answer, OK] = readSWversion()
        %> @brief reads Software version object
        %>
        %> Ask Epos device for software version object. If a correct 
        %> request is made, the software version word is placed in answer. If 
        %> not, an answer will be empty
        %>
        %> @retval answer Corresponding software version, 'error' if request was
        %>                sucessful but an error was returned or empty if request
        %>                was not sucessfull.
        %> @retval OK     A boolean if all went ok or not.
        %=======================================================================
        function [SWversion,OK] = readSWversion(me)
            if(~me.connected)
                SWversion = 'none';
                OK = false;
                return;
            else
                index = me.objectIndex('Version');
                [SWversion, OK] = me.readObject(index,1);
                if (OK)
                    E_error = me.checkError(SWversion(2:3));
                    if E_error == 0
                        SWversion = SWversion(4);
                    else
                        SWversion = 'error';
                        OK = false;
                    end
                else
                    SWversion = 'none';
                end
            end
        end
        
        %=======================================================================
        %> @fn [position, OK] = readPositionModeSetting()
        %> @brief reads the setted desired Position 
        %>
        %> Ask Epos device for demand position object. If a correct 
        %> request is made, the position is placed in answer. If 
        %> not, an answer will be empty
        %>
        %> @retval position Corresponding device name, 'error' if request was
        %>                sucessful but an error was returned or empty if request
        %>                was not sucessfull.
        %> @retval OK     A boolean if all went ok or not.
        %=======================================================================
        function [position,OK] = readPositionModeSetting(me)
            if(~me.connected)
                position = [];
                OK = false;
                return;
            else
                [position, OK] = me.readObject(me.objectIndex('PositionModeSettingValue'),0);
                if(OK)
                    OK = ~me.checkError(position(2:3));
                    if OK 
                        position = typecast(position(4:5), 'int32');
                    else
                        position = 'error';
                        OK = false;
                    end
                else
                    position = [];
                    OK = false;
                end
            end
		end
        %=======================================================================
        %> @fn [OK] = setPositionModeSetting()
        %> @brief sets the desired Position 
        %>
        %> Ask Epos device to set position mode setting object. 
        %>
        %> @retval OK     A boolean if all went ok or not.
        %=======================================================================
		function [OK] = setPositionModeSetting(me, position)
            if(position < -2^31 || position > 2^31)
                fprintf('[Epos setPositionModeSetting] Postion out of range\n');
                OK = false;
                return;
            else
                index = me.objectIndex('PositionModeSettingValue');
                subindex = uint8(0);
                data = typecast(int32(position), 'uint16');
                [answer, OK] = me.writeObject(index, subindex, data);
                if ~OK
                    %todo
                else
                    OK = ~me.checkError(answer(2:3));
                    %check for errors
                end
            end
        end
        %=======================================================================
        %> @fn [velocity, OK] = readVelocityModeSetting()
        %> @brief reads the setted desired velocity 
        %>
        %> Ask Epos device for demand velocity object. If a correct 
        %> request is made, the velocity is placed in answer. If 
        %> not, an answer will be empty
        %>
        %> @retval velocity Corresponding device name, 'error' if request was
        %>                sucessful but an error was returned or empty if request
        %>                was not sucessfull.
        %> @retval OK     A boolean if all went ok or not.
        %=======================================================================
        function [velocity,OK] = readVelocityModeSetting(me)
            if(~me.connected)
                velocity = [];
                OK = false;
                return;
            else
                [velocity, OK] = me.readObject(me.objectIndex('VelocityModeSettingValue'),0);
                if(OK)
                    OK = ~me.checkError(velocity(2:3));
                    if OK 
                        velocity = typecast(velocity(4:5), 'int32');
                    else
                        velocity = 'error';
                        OK = false;
                    end
                else
                    velocity = [];
                    OK = false;
                end
            end
        end
        %=======================================================================
        %> @fn [OK] = setVelocityModeSetting()
        %> @brief sets the desired velocity  
        %>
        %> Ask Epos device to set velocity mode setting object. 
        %>
        %> @retval OK     A boolean if all went ok or not.
        %=======================================================================
        function [OK] = setVelocityModeSetting(me, velocity)
            if(velocity < -2^31 || velocity > 2^31)
                fprintf('[Epos setVelocityModeSetting] Velocity out of range\n');
                OK = false;
                return;
            else
                index = me.objectIndex('VelocityModeSettingValue');
                subindex = uint8(0);
                data = typecast(int32(velocity), 'uint16');
                [answer, OK] = me.writeObject(index, subindex, data);
                if ~OK
                    %todo
                else
                    OK = ~me.checkError(answer(2:3));
                    %check for errors
                end
            end
        end
        %=======================================================================
        %> @fn [current, OK] = readCurrentModeSetting()
        %> @brief reads the setted desired current 
        %>
        %> Ask Epos device for demand current object. If a correct 
        %> request is made, the current is placed in answer. If 
        %> not, an answer will be empty
        %>
        %> @retval current Corresponding device name, 'error' if request was
        %>                sucessful but an error was returned or empty if request
        %>                was not sucessfull.
        %> @retval OK     A boolean if all went ok or not.
        %=======================================================================
        function [current,OK] = readCurrentModeSetting(me)
            if(~me.connected)
                current = [];
                OK = false;
                return;
            else
                [current, OK] = me.readObject(me.objectIndex('CurrentModeSettingValue'),0);
                if(OK)
                    OK = ~me.checkError(current(2:3));
                    if OK 
                        current = typecast(current(4), 'int16');
                    else
                        current = 'error';
                        OK = false;
                    end
                else
                    current = [];
                    OK = false;
                end
            end
        end
        %=======================================================================
        %> @fn [OK] = setCurrentModeSetting()
        %> @brief sets the desired current 
        %>
        %> Ask Epos device to set current mode setting object. 
        %>
        %> @retval OK     A boolean if all went ok or not.
        %=======================================================================
        function [OK] = setCurrentModeSetting(me, current)
            if(current < -2^15 || current > 2^15)
                fprintf('[Epos setCurrentModeSetting] Postion out of range\n');
                OK = false;
                return;
            else
                index = me.objectIndex('CurrentModeSettingValue');
                subindex = uint8(0);
                data = typecast(int16(current), 'uint16');
                [answer, OK] = me.writeObject(index, subindex, [data 0]);
                if ~OK
                    %todo
                else
                    OK = ~me.checkError(answer(2:3));
                    %check for errors
                end
            end
        end
        %=======================================================================
        %> @fn [OK] = setOpMode(opMode)
        %> @brief sets the operation mode
        %>
        %> Sets the operation mode of Epos. OpMode is described as:
        %>
        %> | OpMode | Description           |
        %> |:------:|:----------------------|
        %> | 6      | Homing Mode           |
        %> | 3      | Profile Velocity Mode |
        %> | 1      | Profile Position Mode |
        %> | -1     | Position Mode         |
        %> | -2     | Velocity Mode         |
        %> | -3     | Current Mode          |
        %> | -4     | Diagnostic Mode       |
        %> | -5     | MasterEncoder Mode    |
        %> | -6     | Step/Direction Mode   |
        %>
        %> @retval OK     A boolean if all went ok or not.
        %=======================================================================
        function [OK] = setOpMode(me, opMode)
            if(~any(opMode == [6 3 1 -1 -2 -3 -4 -5 -6]))
                fprintf('[Epos setOpMode] Invalid mode of operation: %d\n', opMode);
                OK = false;
                return;
            end
            %
            opMode = typecast([int8(opMode) 0], 'uint16');
            data = uint16([opMode 0]);
            index = me.objectIndex('ModesOperation');
            subindex = uint8(0);
            [answer, OK] = me.writeObject(index, subindex, data);
            if ~OK
                %todo
            else
                OK = ~me.checkError(answer(2:3));
            end
        end
        %=======================================================================
        %> @fn [opMode, OK] = readOpMode()
        %> @brief reads the operation mode
        %>
        %>  
        %> @retval opMode current opMode of EPOS.
        %> @retval OK     A boolean if all went ok or not.
        %=======================================================================
        function [opMode, OK] = readOpMode(me)
            index = me.objectIndex('ModesOperationDisplay');
            subindex = uint8(0);

            [opMode, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(opMode(2:3));
                if OK 
                    opMode = typecast(opMode(4), 'int8');
                    opMode = opMode(1);
                else
                    opMode = 'error';
                    OK = false;
                end
            else
                opMode = [];
                OK = false;
            end
        end
        %=======================================================================
        %> @fn printOpMode()
        %> @brief prints the current operation mode.
        %=======================================================================
		function printOpMode(me)
			[opMode, OK] = me.readOpMode();
			if (OK)
				switch opMode
					case 6
						fprintf('[Epos printOpMode] Homing Mode\n');
					case 3
						fprintf('[Epos printOpMode] Profile Velocity Mode\n');
					case 1
						fprintf('[Epos printOpMode] Profile Position Mode\n');
					case -1
						fprintf('[Epos printOpMode] Position Mode\n');
					case -2
						fprintf('[Epos printOpMode] Velocity Mode\n');
					case -3
						fprintf('[Epos printOpMode] Current Mode\n');
					case -4
						fprintf('[Epos printOpMode] Diagnostic Mode\n');
					case -5
						fprintf('[Epos printOpMode] Master Encoder Mode\n');
					case -6
						fprintf('[Epos printOpMode] Step|Direction Mode\n');
					otherwise
						fprintf('[Epos printOpMode] Unknown mode:%d\n', opMode);
				end
			else
				fprintf('[Epos printOpMode] Failded to get mode of operation\n');
			end
		end

        %=======================================================================
        %> @fn [OK] = setMotorConfig(motorType, currentLimit, maximumSpeed, polePairNumber)
        %> @brief sets the basic configuration for motor.
        %>
        %> Sets the configuration of the motor parameters. The valid motor type is:
        %>
        %> |motorType              | value| Description              |
        %> |:----------------------|:----:|:-------------------------|
        %> |DC motor               | 1    | brushed DC motor         |
        %> |Sinusoidal PM BL motor | 10   | EC motor sinus commutated|
        %> |Trapezoidal PM BL motor| 11   | EC motor block commutated|
        %>
        %> The current limit is the current limit is the maximal permissible 
        %> continuous current of the motor in mA.
        %> Minimum value is 0 and max is hardware dependent.
        %>
        %> The output current limit is recommended to be 2 times the continuous
        %> current limit.
        %>  
        %> The pole pair number refers to the number of magnetic pole pairs 
        %> (number of poles / 2) from rotor of a brushless DC motor.
        %> 
        %> The maximum speed is used to prevent mechanical destroys in current 
        %> mode. It is possible to limit the velocity [rpm]
        %>
        %> Thermal winding not changed, using default 40ms. 
        %>
        %> @param motorType      value of motor type. see table behind.
        %> @param currentLimit   max continuous current limit [mA].
        %> @param maximumSpeed   max allowed speed in current mode [rpm].
        %> @param polePairNumber number of pole pairs for brushless DC motors.
        %>
        %> @retval OK     A boolean if all went ok or not.
        %=======================================================================

        function [OK] = setMotorConfig(me, motorType, currentLimit, maximumSpeed, polePairNumber)
            
            %change to disable state;
            % reset first? 
            if(~me.changeEposState('disable operation'))
                fprintf('[Epos setSensorConfig] Failed to change EPOS to disable\n');
                OK = false;
                return;
            end

            % set motor type
            if (~any(motorType == [1 10 11]))
                fprintf('[Epos setMotorConfig] Error: not a valid motorType %01d.\n', motorType);
                OK = false;
                return;
            end
            index = me.objectIndex('MotorType');
            subindex = uint8(0);
            motorType = uint16([motorType 0]);
            [answer, OK] = me.writeObject(index, subindex, motorType);
            if ~OK
                fprintf('[Epos setMotorConfig] Failed to set MotorType\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
            end
            % set continuous current limit
            %check ranges
            if(currentLimit < 0 || currentLimit > 2^16)
               fprintf('[Epos setMotorConfig] Error: currentLimit out of range\n');
               OK = false;
               return;
            elseif (floor(currentLimit) ~= currentLimit)
                fprintf('[Epos setMotorConfig] Warning: currentLimit should be a integer value\n Using floor\n');
                currentLimit = floor(currentLimit); 
            end
            % This object represents the maximal permissible continuous current of the motor [mA]
            index = me.objectIndex('MotorData');
            subindex = uint8(1);
            currentLimit = uint16([currentLimit 0]);
            [answer, OK] = me.writeObject(index, subindex, currentLimit);
            if ~OK
                fprintf('[Epos setMotorConfig] Failed to set MotorData current limit\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                if ~OK
                    return;
                end
            end
            % set output current limit
            % It is recommended to set the output current limit to a value doubles of continuous current limit [mA].
            subindex = uint8(2);
            outputCurrentLimit = uint16([2*currentLimit 0]);
            [answer, OK] = me.writeObject(index, subindex, outputCurrentLimit);
            if ~OK
                fprintf('[Epos setMotorConfig] Failed to set MotorData output current limit\n');
                return
            else
                OK = ~me.checkError(answer(2:3));
                if ~OK
                    return;
                end
            end
            % Number of magnetic pole pairs (number of poles / 2) from rotor of a brushless DC motor.
            % only needed for brushless DC
            if(motorType > 1)
                if(polePairNumber < 1 || polePairNumber > 255)
                    fprintf('[Epos setMotorConfig] Error: Invalid pole pair number range\n');
                    OK = false;
                    return;
                end
				
				subindex = uint8(3);
				polePairNumber = uint16([uint8(polePairNumber) 0]);
				[answer, OK] = me.writeObject(index, subindex, polePairNumber);
				if ~OK
					fprintf('[Epos setMotorConfig] Failed to set MotorData pole pair number\n');
					return
				else
					OK = ~me.checkError(answer(2:3));
					if ~OK
                        return;
                    end
                end
            end
            % To prevent mechanical destroys in current mode it is possible to limit the velocity [rpm].
            %check ranges
            if(maximumSpeed < 1 || maximumSpeed >= 2^16)
               fprintf('[Epos setMotorConfig] Error: maximum Speed out of range\n');
               OK = false;
               return;
            elseif (floor(maximumSpeed) ~= maximumSpeed)
                fprintf('[Epos setMotorConfig] Warning: maximumSpeed should be a integer value\n Using floor\n');
                maximumSpeed = floor(maximumSpeed); 
            end
            subindex = uint8(4);
            maximumSpeed = uint16([maximumSpeed 0]);
            [answer, OK] = me.writeObject(index, subindex, maximumSpeed);
            if ~OK
                fprintf('[Epos setMotorConfig] Failed to set MotorData maximum speed\n');
                return
            else
                OK = ~me.checkError(answer(2:3));
                if ~OK
                    return;
                end
            end
        end
        %=======================================================================
        %> @fn [motorConfig, OK] = readMotorConfig()
        %> @brief gets the current motor configuration
        %>
        %> Requests from EPOS the current motor type and motor data.
        %> The motorConfig is an struture containing the following information:
        %>
        %> [*] motorType - describes the type of motor.
        %> [*] currentLimit - describes the maximum continuous current limit.
        %> [*] maxCurrentLimit - describes the maximum allowed current limit. 
        %> Usually is set as two times the continuous current limit.
        %> [*] polePairNumber - describes the pole pair number of the rotor of 
        %> the brushless DC motor.
        %> [*] maximumSpeed - describes the maximum allowed speed in current mode.
        %> [*] thermalTimeConstant - describes the thermal time constant of motor 
        %> winding is used to calculate the time how long the maximal output 
        %> current is allowed for the connected motor [100 ms].
        %>
        %> If unable to request the configuration or unsucessfull, an empty
        %> structure is returned. Any error inside any field requests are marked
        %> with 'error'. 
        %>
        %> @retval motorConfig A structure with the current configuration of motor
        %> @retval OK          A boolean if all went as expected or not.         
        %=======================================================================
        function [motorConfig, OK] = readMotorConfig(me)

            % get MotorType object
            index = me.objectIndex('MotorType');
            subindex = uint8(0);
            [motorType, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(motorType(2:3));
                if OK 
                    motorType = motorType(4);
                    switch motorType
                    case 1
                        motorConfig.motorType = 'DC motor';
                    case 10
                        motorConfig.motorType = 'Sinusoidal PM BL motor';
                    case 11
                        motorConfig.motorType = 'Trapezoidal PM BL motor';
                    otherwise
                        motorConfig.motorType = 'Error';
                        OK = false;
                    end
                else
                    motorConfig.motorType = 'Error';
                    OK = false;
                end
            else
                motorConfig.motorType = [];
                OK = false;
                return;
            end
            % This object represents the maximal permissible continuous current of the motor [mA]
            index = me.objectIndex('MotorData');
            subindex = uint8(1);
            [motorData, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(motorData(2:3));
                if OK 
                    motorConfig.currentLimit = motorData(4);
                else
                    motorConfig.currentLimit = 'Error';
                    OK = false;
                end
            else
                motorConfig.currentLimit = [];
                OK = false;
                return;
            end
            % This object represents the maximal current of the motor [mA]
            % is set at 2 * (continuous current limit)
            index = me.objectIndex('MotorData');
            subindex = uint8(2);
            [motorData, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(motorData(2:3));
                if OK 
                    motorConfig.maxCurrentLimit = motorData(4);
                else
                    motorConfig.maxCurrentLimit = 'Error';
                    OK = false;
                end
            else
                motorConfig.maxCurrentLimit = [];
                OK = false;
                return;
            end
            % This object represents pole pair number
            index = me.objectIndex('MotorData');
            subindex = uint8(3);
            [motorData, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(motorData(2:3));
                if OK 
                    motorConfig.polePairNumber = uint8(motorData(4));
                else
                    motorConfig.polePairNumber = 'Error';
                    OK = false;
                end
            else
                motorConfig.polePairNumber = [];
                OK = false;
                return;
            end
            % This object represents the maximal permissible speed in current mode
            index = me.objectIndex('MotorData');
            subindex = uint8(4);
            [motorData, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(motorData(2:3));
                if OK 
                    motorConfig.maximumSpeed = motorData(4);
                else
                    motorConfig.maximumSpeed = 'Error';
                    OK = false;
                end
            else
                motorConfig.maximumSpeed = [];
                OK = false;
                return;
            end
            % The thermal time constant of motor winding is used to calculate 
            % the time how long the maximal output current is allowed for the 
            % connected motor [100 ms]
            subindex = uint8(5);
            [motorData, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(motorData(2:3));
                if OK 
                    motorConfig.thermalTimeConstant = motorData(4);
                else
                    motorConfig.thermalTimeConstant = 'Error';
                    OK = false;
                end
            else
                motorConfig.thermalTimeConstant = [];
                OK = false;
                return;
            end
        end
        
        function printMotorConfig(me)
            [motorConfig, OK] = me.readMotorConfig;
            if (OK)
                fprintf('[Epos printMotorConfig] Current motor configuration is:\n');
                fprintf('Motor Type: %s\n', motorConfig.motorType);
                fprintf('Continuous current limit [mA]: %d\n', motorConfig.currentLimit);
                fprintf('Maximum allowed current limit [mA]: %d\n', motorConfig.maxCurrentLimit);
                fprintf('Pole pair number: %d\n', motorConfig.polePairNumber);
                fprintf('Maximum allowed speed [rpm]: %d\n', motorConfig.maximumSpeed);
                fprintf('Thermal time constant [s*0.1]: %d\n', motorConfig.thermalTimeConstant);
            else
                fprintf('[Epos printMotorConfig] ERROR - Unable to get the motor configuration\n');
            end
        end
        %=======================================================================
        %> @fn [OK] = setSensorConfig(pulseNumber, sensorType, sensorPolatity)
        %> @brief change sensor configuration
        %>
        %> Change the sensor configuration of motor. **Only possible if in disable state**
        %> The encoder pulse number should be set to number of counts per 
        %> revolution of the connected incremental encoder.
        %> range : |16|7500|
        %>
        %> sensor type is described as:
        %>
        %> |value| description                                     |
        %> |:---:|:------------------------------------------------|
        %> |1    | Incremental Encoder with index (3-channel)      |
        %> |2    | Incremental Encoder without index (2-channel)   |
        %> |3    | Hall Sensors (Remark: consider worse resolution)|
        %>
        %> sensor polarity is set by setting the corresponding bit from the word
        %> | Bit | description                                                                      |
        %> |:---:|:---------------------------------------------------------------------------------|
        %> | 15-2| Reserved (0)                                                                     |
        %> | 1   | Hall sensors polarity 0: normal / 1: inverted                                    |
        %> | 0   | Encoder polarity 0: normal / 1: inverted (or encoder mounted on motor shaft side)|
        %>
        %=======================================================================
        function [OK] = setSensorConfig(me, pulseNumber, sensorType, sensorPolarity)
            
            % validate attributes first
            if(pulseNumber<16 || pulseNumber> 7500)
                fprintf('[Epos setSensorConfig] Error pulseNumber out of range\n');
                OK = false;
                return;
            end
            if(~any(sensorType == [1 2 3]))
                fprintf('[Epos setSensorConfig] Error sensorType not valid\n');
                OK = false;
                return;
            end
            if(~any(sensorPolarity == [0 1 2]))
                fprintf('[Epos setSensorConfig] Error sensorPolarity not valid\n');
                OK = false;
                return;
            end
            %change to disable state;
            % reset first?
            if(~me.changeEposState('disable operation'))
                fprintf('[Epos setSensorConfig] Failed to change EPOS to disable\n');
                OK = false;
                return;
            end

            index = me.objectIndex('SensorConfiguration');
            
            
            subindex = uint8(1);
            pulseNumber = uint16([pulseNumber 0]);
            [answer, OK] = me.writeObject(index, subindex, pulseNumber);
            if ~OK
                fprintf('[Epos setSensorConfig] Failed to set pulse number\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                if ~OK
                    return;
                end
            end
            
            % The position sensor type can be changed with this parameter.
            
            subindex = uint8(2);
            sensorType = uint16([sensorType 0]);
            [answer, OK] = me.writeObject(index, subindex, sensorType);
            if ~OK
                fprintf('[Epos setSensorConfig] Failed to set sensor type\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                if ~OK
                    return;
                end
            end
            
            % With this parameter the position sensor and the hall sensor polarity can be changed.
            subindex = uint8(4);
            sensorPolarity = uint16([sensorPolarity 0]);
            [answer, OK] = me.writeObject(index, subindex, sensorPolarity);
            if ~OK
                fprintf('[Epos setSensorConfig] Failed to set sensor polarity\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                if ~OK
                    return;
                end
            end  
        end
        %=======================================================================
        %> @fn [Config, OK] = readSensorConfig()
        %> @brief gets the current sensor configuration
        %>
        %> Requests from EPOS the current sensor configuration.
        %> The sensorConfig is an struture containing the following information:
        %>
        %> [*] sensorType - describes the type of sensor.
        %> [*] pulseNumber - describes the number of pulses per revolution in 
        %> one channel.
        %> [*] sensorPolarity - describes the of each sensor. 
        %>
        %> If unable to request the configuration or unsucessfull, an empty
        %> structure is returned. Any error inside any field requests are marked
        %> with 'error'. 
        %> 
        %> @retval sensorConfig A structure with the current configuration of 
        %>                      the sensor
        %> @retval OK           A boolean if all went as expected or not.         
        %=======================================================================
        function [sensorConfig, OK] = readSensorConfig(me)

            % get pulseNumber
            index = me.objectIndex('SensorConfiguration');
            subindex = uint8(1);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    sensorConfig.pulseNumber = answer(4);
                else
                    sensorConfig.pulseNumber = 'Error';
                    OK = false;
                end
            else
                sensorConfig.pulseNumber = [];
                OK = false;
                return;
            end
            % get sensorType
            index = me.objectIndex('SensorConfiguration');
            subindex = uint8(2);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    sensorType = answer(4);
                    switch sensorType
                    case 1
                        sensorConfig.sensorType = 'Incremental Encoder with index (3-channel)';
                    case 2
                        sensorConfig.sensorType = 'Incremental Encoder without index (2-channel)';
                    case 3
                        sensorConfig.sensorType = 'Hall sensors';
                    otherwise
                        fprintf('[Epos readSensorConfig] Error unknown sensor type\n');
                        OK = false;
                        sensorConfig.sensorType = 'Error';
                    end
                else
                    sensorConfig.sensorType = 'Error';
                    OK = false;
                end
            else
                sensorConfig.sensorType = [];
                OK = false;
                return;
            end
            % get sensorPolarity
            index = me.objectIndex('SensorConfiguration');
            subindex = uint8(4);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    answer = answer(4);
                    switch answer
                    case 0
                        sensorConfig.sensorPolarity.hallSensor = 'normal';
                        sensorConfig.sensorPolarity.encoderSensor = 'normal';
                    case 1 
                        sensorConfig.sensorPolarity.hallSensor = 'normal';
                        sensorConfig.sensorPolarity.encoderSensor = 'inverted';
                    case 2
                        sensorConfig.sensorPolarity.hallSensor = 'inverted';
                        sensorConfig.sensorPolarity.encoderSensor = 'normal';
                    case 3
                        sensorConfig.sensorPolarity.hallSensor = 'inverted';
                        sensorConfig.sensorPolarity.encoderSensor = 'inverted';
                    otherwise
                        sensorConfig.sensorPolarity = 'Error';
                        OK = false;
                        return;
                    end
                else
                    sensorConfig.sensorPolarity = 'Error';
                    OK = false;
                    return;
                end
            else
                sensorConfig.sensorPolarity = [];
                OK = false;
                return;
            end
        end
        function printSensorConfig(me)
            [sensorConfig, OK] = me.readSensorConfig;
            if (OK)
                fprintf('[Epos printSensorConfig] Current sensor configuration is:\n');
                fprintf('Sensor Type: %s\n', sensorConfig.sensorType);
                fprintf('Encoder Pulses per revolution: %d\n', sensorConfig.pulseNumber);
                fprintf('Encoder polarity: %s\n', sensorConfig.sensorPolarity.encoderSensor);
                fprintf('Hall sensor polarity: %s\n', sensorConfig.sensorPolarity.hallSensor);
            else
                fprintf('[Epos MotorConfig] ERROR - Unable to get the motor configuration\n');
            end
        end
        %=======================================================================
        %=======================================================================
        function [currentControlPIgains, OK] = readCurrentControlParam(me)

            %read current regulator P-gain
            index = me.objectIndex('CurrentControlParameterSet');
            subindex = uint8(1);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    currentControlPIgains.pGain = int16(answer(4));
                else
                    currentControlPIgains.pGain = 'Error';
                    OK = false;
                    return;
                end
            else
                currentControlPIgains.pGain = [];
                OK = false;
                return;
            end
            subindex = uint8(2);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    currentControlPIgains.iGain = int16(answer(4));
                else
                    currentControlPIgains.iGain = 'Error';
                    OK = false;
                    return;
                end
            else
                currentControlPIgains.iGain = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [OK] = setCurrentControlParam(me, pGain, iGain)

            % validate attributes first
            if( pGain < 0 || pGain > 2^15-1)
                fprintf('[Epos setCurrentControlParam] pGain is out of range [0 - 32767]\n');
                OK = false;
                return;
            end
            if( iGain < 0 || iGain > 2^15-1)
                fprintf('[Epos setCurrentControlParam] iGain is out of range [0 - 32767]\n');
                OK = false;
                return;
            end
            % set pGain
            index = me.objectIndex('CurrentControlParameterSet');
            subindex = uint8(1);
            pGain = int16(pGain);
            [answer, OK] = me.writeObject(index, subindex, [pGain 0]);
            if ~OK
                fprintf('[Epos setCurrentControlParam] Failed to set pGain\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                if ~OK
                    return;
                end
            end
            % set iGain
            subindex = uint8(2);
            iGain = int16(iGain);
            [answer, OK] = me.writeObject(index, subindex, [iGain 0]);
            if ~OK
                fprintf('[Epos setCurrentControlParam] Failed to set iGain\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                if ~OK
                    return;
                end
            end
        end
        
        function printCurrentControlParam(me)
            [param, OK] = me.readCurrentControlParam();
            if OK
                fprintf('[Epos printCurrentControlParam] Current control gains:\n');
                fprintf('Proportional Gain: %d\n', param.pGain);
                fprintf('Integral Gain: %d\n', param.iGain);
            else
                fprintf('[Epos printCurrentControlParam] ERROR unable to read current mode control parameters\n');
            end
        end
        %=======================================================================
        %
        % Position profile functions
        %
        %=======================================================================

        %=======================================================================
        %> @fn [pos, OK] = readSoftwarePosLimit()
        %> @brief reads the limits of the software position
        %>
        %>  
        %> @retval pos    A structure with fields nimPos and maxPos
        %> @retval OK     A boolean if all went ok or not.
        %=======================================================================
        function [pos, OK] = readSoftwarePosLimit(me)
            index = me.objectIndex('SoftwarePositionLimit');
            subindex = uint8(1);

            [minPos, OK] = me.readObject(index, subindex);
            
            if(OK)
                OK = ~me.checkError(minPos(2:3));
                if OK 
                    pos.minPos = typecast(minPos(4:5), 'int32');
                else
                    pos.minPos = 'error';
                    OK = false;
                    return;
                end
            else
                pos.minPos = [];
                OK = false;
                return;
            end
            
            subindex = uint8(2);
            [maxPos, OK] = me.readObject(index, subindex);
            
            if(OK)
                OK = ~me.checkError(maxPos(2:3));
                if OK 
                    pos.maxPos = typecast(maxPos(4:5), 'int32');
                else
                    pos.maxPos = 'error';
                    OK = false;
                    return;
                end
            else
                OK = false;
                pos.maxPos = [];
                return;
            end
            
        end
        
        %=======================================================================
        %> @fn [OK] = setSoftwarePosLimit(me, minPos, maxPos)
        %> @brief change software position limit
        %>
        %> Position Limits
        %> range : [-2147483648|2147483647]
        %>
        %=======================================================================
        
        function [OK] = setSoftwarePosLimit(me, minPos, maxPos)
            
            % validate attributes first
            if(minPos< -2^31 || minPos> 2^31)
                fprintf('[Epos setSoftwarePosLimit] Error minPos out of range\n');
                OK = false;
                return;
            end
            if(maxPos< -2^31 || maxPos> 2^31)
                fprintf('[Epos setSoftwarePosLimit] Error maxPos out of range\n');
                OK = false;
                return;
            end
            % grant it is a int32
            minPos = int32(minPos);
            maxPos = int32(maxPos);

            index = me.objectIndex('SoftwarePositionLimit');
            
            % set minimum position limit
            subindex = uint8(1);
            minPos = typecast(minPos,'uint16');
            [answer, OK] = me.writeObject(index, subindex, minPos);
            if ~OK
                fprintf('[Epos setSensorConfig] Failed to set software minimum limit position\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
            
            % set maximum position limit
            subindex = uint8(2);
            maxPos = typecast(maxPos,'uint16');
            [answer, OK] = me.writeObject(index, subindex, maxPos);
            if ~OK
                fprintf('[Epos setSensorConfig] Failed to set software maximum limit position\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
        end
        
        function printSoftwarePosLimit(me)
            [pos, OK] = me.readSoftwarePosLimit();
            if OK
                fprintf('[Epos printSoftwarePosLimit] Software Position Limits:\n');
                fprintf('Minimum [qc]: %d\n', pos.minPos);
                fprintf('Maximum [qc]: %d\n', pos.maxPos);
            else
                fprintf('[Epos printSoftwarePosLimit] ERROR Unable to read software position limits\n');
            end
        end

        %=======================================================================
        % This value is used as velocity limit in a position (or velocity) profile move
        %=======================================================================
        function [maxProfileVelocity, OK] = readMaxProfileVelocity(me)
            %
            index = me.objectIndex('MaximalProfileVelocity');
            subindex = uint8(0);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    maxProfileVelocity = typecast(answer(4:5), 'uint32');
                else
                    maxProfileVelocity = 'error';
                    OK = false;
                    return;
                end
            else
                maxProfileVelocity = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        % This value is used as velocity limit in a position (or velocity) profile move
        %=======================================================================
        function [OK] = setMaxProfileVelocity(me, maxProfileVelocity)
            
            % validate attributes
            if(maxProfileVelocity< 1 || maxProfileVelocity > 25000)
                fprintf('[Epos setMaxProfileVelocity] Error maxProfileVelocity out of range\n');
                OK = false;
                return;
            end

            index = me.objectIndex('MaximalProfileVelocity');
            subindex = uint8(0);

            maxProfileVelocity = typecast(uint32(maxProfileVelocity),'uint16');
            [answer, OK] = me.writeObject(index, subindex, maxProfileVelocity);
            if ~OK
                fprintf('[Epos setMaxProfileVelocity] Failed to set maxProfileVelocity \n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end

        end
        %=======================================================================
        % Todo
        % The profile velocity is the velocity normally attained at the end of 
        % the acceleration ramp during a profiled move [Velocity units]
        %=======================================================================
        function [profileVelocity, OK] = readProfileVelocity(me)
            %
            index = me.objectIndex('ProfileVelocity');
            subindex = uint8(0);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    profileVelocity = typecast(answer(4:5), 'uint32');
                else
                    profileVelocity = 'error';
                    OK = false;
                    return;
                end
            else
                profileVelocity = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        % Todo
        % The profile velocity is the velocity normally attained at the end of 
        % the acceleration ramp during a profiled move [Velocity units]
        %=======================================================================
        function [OK] = setProfileVelocity(me, profileVelocity)
            
            % validate attributes
            if(profileVelocity< 1 || profileVelocity > 25000)
                fprintf('[Epos setProfileVelocity] Error profileVelocity out of range\n');
                OK = false;
                return;
            end

            index = me.objectIndex('ProfileVelocity');
            subindex = uint8(0);

            profileVelocity = typecast(uint32(profileVelocity),'uint16');
            [answer, OK] = me.writeObject(index, subindex, profileVelocity);
            if ~OK
                fprintf('[Epos setProfileVelocity] Failed to set profileVelocity \n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end

        end
        %=======================================================================
        %=======================================================================
        function [profileAcceleration, OK] = readProfileAcceleration(me)
            %
            index = me.objectIndex('ProfileAcceleration');
            subindex = uint8(0);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    profileAcceleration = typecast(answer(4:5), 'uint32');
                else
                    profileAcceleration = 'error';
                    OK = false;
                    return;
                end
            else
                profileAcceleration = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [OK] = setProfileAcceleration(me, profileAcceleration)
            
            % validate attributes
            if(profileAcceleration< 1 || profileAcceleration > 2^32-1)
                fprintf('[Epos setProfileAcceleration] Error profileAcceleration out of range\n');
                OK = false;
                return;
            end

            index = me.objectIndex('ProfileAcceleration');
            subindex = uint8(0);

            profileAcceleration = typecast(uint32(profileAcceleration),'uint16');
            [answer, OK] = me.writeObject(index, subindex, profileAcceleration);
            if ~OK
                fprintf('[Epos setProfileAcceleration] Failed to set profileAcceleration \n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end

        end
        %=======================================================================
        %=======================================================================
        function [profileDeceleration, OK] = readProfileDeceleration(me)
            %
            index = me.objectIndex('ProfileDeceleration');
            subindex = uint8(0);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    profileDeceleration = typecast(answer(4:5), 'uint32');
                else
                    profileDeceleration = 'error';
                    OK = false;
                    return;
                end
            else
                profileDeceleration = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [OK] = setProfileDeceleration(me, profileDeceleration)
            
            % validate attributes
            if(profileDeceleration< 1 || profileDeceleration > 2^32-1)
                fprintf('[Epos setProfileDeceleration] Error profileDeceleration out of range\n');
                OK = false;
                return;
            end

            index = me.objectIndex('ProfileDeceleration');
            subindex = uint8(0);

            profileDeceleration = typecast(uint32(profileDeceleration),'uint16');
            [answer, OK] = me.writeObject(index, subindex, profileDeceleration);
            if ~OK
                fprintf('[Epos setProfileDeceleration] Failed to set profileDeceleration \n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end

        end
        %=======================================================================
        %=======================================================================
        function [quickstopDeceleration, OK] = readQuickstopDeceleration(me)
            %
            index = me.objectIndex('QuickStopDeceleration');
            subindex = uint8(0);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    quickstopDeceleration = typecast(answer(4:5), 'uint32');
                else
                    quickstopDeceleration = 'error';
                    OK = false;
                    return;
                end
            else
                quickstopDeceleration = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [OK] = setQuickstopDeceleration(me, quickstopDeceleration)
            
            % validate attributes
            if(quickstopDeceleration< 1 || quickstopDeceleration > 2^32-1)
                fprintf('[Epos setQuickstopDeceleration] Error quickstopDeceleration out of range\n');
                OK = false;
                return;
            end

            index = me.objectIndex('QuickStopDeceleration');
            subindex = uint8(0);

            quickstopDeceleration = typecast(uint32(quickstopDeceleration),'uint16');
            [answer, OK] = me.writeObject(index, subindex, quickstopDeceleration);
            if ~OK
                fprintf('[Epos setQuickstopDeceleration] Failed to set quickstopDeceleration \n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end

        end
        %=======================================================================
        %=======================================================================
        function [motionProfileType, OK] = readMotionProfileType(me)
            %
            index = me.objectIndex('MotionProfileType');
            subindex = uint8(0);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    motionProfileType = typecast(answer(4:5), 'uint32');
                else
                    motionProfileType = 'error';
                    OK = false;
                    return;
                end
            else
                motionProfileType = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [OK] = setMotionProfileType(me, motionProfileType)
            
            % validate attributes
            if(~any([0 1] == motionProfileType))
                fprintf('[Epos setMotionProfileType] Error motionProfileType out of range\n');
                OK = false;
                return;
            end

            index = me.objectIndex('MotionProfileType');
            subindex = uint8(0);

            profileDeceleration = uint16(motionProfileType);
            [answer, OK] = me.writeObject(index, subindex, [profileDeceleration 0]);
            if ~OK
                fprintf('[Epos setMotionProfileType] Failed to set motionProfileType \n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end

        end
        %=======================================================================
        %=======================================================================
        function [positionProfileConfig, OK] = readPositionProfileConfig(me)
            %
            OK = [0 0 0 0 0 0 0 0];
            [positionProfileConfig.maxFollowingError, OK(1)] = me.readMaxFollowingError();
            [positionProfileConfig.softwarePositionLimit, OK(2)] = me.readSoftwarePosLimit();
            [positionProfileConfig.maxProfileVelocity, OK(3)] = me.readMaxProfileVelocity();
            [positionProfileConfig.profileVelocity, OK(4)] = me.readProfileVelocity();
            [positionProfileConfig.profileAcceleration, OK(5)] = me.readProfileAcceleration();
            [positionProfileConfig.profileDeceleration, OK(6)] = me.readProfileDeceleration();
            [positionProfileConfig.quickstopDeceleration, OK(7)] = me.readQuickstopDeceleration();
            [positionProfileConfig.motionProfileType, OK(8)] = me.readMotionProfileType();
            if(any(OK == 0))
                OK = false;
            else
                OK = true;
            end
        end
        %=======================================================================
        %=======================================================================
        function [OK] = setPositionProfileConfig(me, maxFollowingError, minPos, maxPos,...
				maxProfileVelocity, profileVelocity, profileAcceleration, profileDeceleration,...
				quickstopDeceleration, motionProfileType)
			
			OK = me.setMaxFollowingError(maxFollowingError);
			if ~OK
				fprintf('[Epos setMotionProfileConfig] ERROR setting maxFollowingError\n');
				return;
			end
			OK = me.setSoftwarePosLimit(minPos, maxPos);
			if ~OK
				fprintf('[Epos setMotionProfileConfig] ERROR setting SoftwarePositionLimit\n');
				return;
			end
			OK = me.setMaxProfileVelocity(maxProfileVelocity);
			if ~OK
				fprintf('[Epos setMotionProfileConfig] ERROR setting maxProfileVelocity\n');
				return;
			end
			OK = me.setProfileVelocity(profileVelocity);
			if ~OK
				fprintf('[Epos setMotionProfileConfig] ERROR setting profileVelocity\n');
				return;
            end
			OK = me.setProfileAcceleration(profileAcceleration);
			if ~OK
				fprintf('[Epos setMotionProfileConfig] ERROR setting profileAcceleration\n');
				return;
            end
			OK = me.setProfileDeceleration(profileDeceleration);
			if ~OK
				fprintf('[Epos setMotionProfileConfig] ERROR setting profileDeceleration\n');
				return;
            end
			OK = me.setQuickstopDeceleration(quickstopDeceleration);
			if ~OK
				fprintf('[Epos setMotionProfileConfig] ERROR setting quickstopDeceleration\n');
				return;
            end
			OK = me.setMotionProfileType(motionProfileType);
			if ~OK
				fprintf('[Epos setMotionProfileConfig] ERROR setting motionProfileType\n');
				return;
            end
        end
        %=======================================================================
        %=======================================================================

        function printPositionProfileConfig(me)
            [positionProfileConfig, OK] = me.readPositionProfileConfig();
            if OK
                fprintf('[Epos printPositionProfileConfig] Position Profile Configuration parameters are:\n');
                fprintf('Maximum following error [qc]: %d\n', positionProfileConfig.maxFollowingError);
                fprintf('Minimum software position limit [qc]: %d\n', positionProfileConfig.softwarePositionLimit.minPos);
                fprintf('Maximum software position limit [qc]: %d\n', positionProfileConfig.softwarePositionLimit.maxPos);
                fprintf('Maximum velocity limit [rpm]: %d\n', positionProfileConfig.maxProfileVelocity);
                fprintf('Maximum velocity [rpm]: %d\n', positionProfileConfig.profileVelocity);
                fprintf('Acceleration [rpm/s]: %d\n', positionProfileConfig.profileAcceleration);
                fprintf('Deceleration [rpm/s]: %d\n', positionProfileConfig.profileAcceleration);
                fprintf('Quick stop deceleration [rpm/s]: %d\n', positionProfileConfig.quickstopDeceleration);
                if(positionProfileConfig.motionProfileType)
                    fprintf('Motion profile type: sinusoidal profile\n');
                else
                    fprintf('Motion profile type: linear profile\n');
                end
            else
                fprintf('[Epos printPositionProfileConfig] Failed to request position profile configuration parameters\n');
            end
        end
        %=======================================================================
        %=======================================================================
        function [position, OK] = readTargetPosition(me)

            index = me.objectIndex('TargetPosition');
            subindex = uint8(0);

            [answer, OK] = me.readObject(index, subindex);
            if OK
                OK = ~me.checkError(answer(2:3));
                if OK 
                    position = typecast(answer(4:5), 'int32');
                else
                    position = 'error';
                    OK = false;
                    return;
                end
            else
                position = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [OK] = setTargetPosition(me, position)

            % validate attributes
            if(position< - 2^31 || position > 2^31 - 1)
                fprintf('[Epos setTargetPosition] ERROR position out of range\n');
                OK = false;
                return;
            end

            index = me.objectIndex('TargetPosition');
            subindex = uint8(0);

            % grant is a int32
            position = int32(position);
            [answer, OK] = me.writeObject(index, subindex, typecast(position, 'uint16') );
            if ~OK
                fprintf('[Epos setTarget] Failed to set target position\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
        end
		%=======================================================================
        %=======================================================================
        function [OK] = setPositioningControlOptions(me, isRelativePos, changeNow, newSetpoint)
            
			% validate attributes
			if(~any([0 1] == isRelativePos))
				fprintf('[Epos setPositioningControlOptions] isAbsolutePos not a boolean\n');
				OK = false;
				return;
            end
			if(~any([0 1] == changeNow))
				fprintf('[Epos setPositioningControlOptions] changeNow not a boolean\n');
				OK = false;
				return;
            end
			if(~any([0 1] == newSetpoint))
				fprintf('[Epos setPositioningControlOptions] newSetpoint not a boolean\n');
				OK = false;
				return;
			end
			
			index = me.objectIndex('Controlword');
			subindex = uint8(0);
			[controlWord, OK] = me.readControlWord();
			if ~OK
				fprintf('[Epos setPositioningControlOptions] Failed to read control word\n');
				return;
			end
			if isRelativePos
				% is relative, set bit 6
				% bitmask xxxx xxxx x1xx xxxx
				controlWord = bitor(bin2dec('0000 0000 0100 0000'), controlWord);
			else
				% is Absolute, unset bit 6
				% bitmask xxxx xxxx x0xx xxxx
				controlWord = bitand(bin2dec('1111 1111 1011 1111'), controlWord);
			end
			if changeNow
				% abort current positioning and change now.
				% bitmask xxxx xxxx xx1x xxxx
				controlWord = bitor(bin2dec('0000 0000 0010 0000'), controlWord);
			else
				% wait for current then change, unset bit 5
				% bitmask xxxx xxxx xx0x xxxx
				controlWord = bitand(bin2dec('1111 1111 1101 1111'), controlWord);
            end
			if newSetpoint
				% assume new target position
				% bitmask xxxx xxxx xxx1 xxxx
				controlWord = bitor(bin2dec('0000 0000 0001 0000'), controlWord);
			else
				% do not assume new target position
				% bitmask xxxx xxxx xxx0 xxxx
				controlWord = bitand(bin2dec('1111 1111 1110 1111'), controlWord);
            end
			[answer, OK] = me.writeObject(index, subindex, [controlWord 0]);
			if ~OK
				fprintf('[Epos setPositioningControlOptions] Failed to write control word\n');
				return;
			else
				OK = ~me.checkError(answer(2:3));
				%check for errors
				if(~OK)
					return;
				end
            end
        end
        %=======================================================================
        %=======================================================================
        function [OK] = haltOperation(me)

            index = me.objectIndex('Controlword');
            subindex = uint8(0);
            [controlWord, OK] = me.readControlWord();
			if ~OK
				fprintf('[Epos haltOperation] Failed to read control word\n');
				return;
			end
            % halt is activated with bit 8 set to 1
            % bitmask =  xxxx xxx1 xxxx xxxx
            controlWord = bitor(bin2dec('0000 0001 0000 0000'), controlWord);
            [answer, OK] = me.writeObject(index, subindex, [controlWord 0]);
            if ~OK
                fprintf('[Epos haltOperation] Failed to halt operation\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                if OK
                    fprintf('[Epos haltOperation] Operation halted\n');
                end
            end
        end
        function [OK] = resumeHaltOpereation(me)
            index = me.objectIndex('Controlword');
            subindex = uint8(0);
            [controlWord, OK] = me.readControlWord();
            if ~OK
                fprintf('[Epos resumeHaltOperation] Failed to read control word\n');
                return;
            end
            % halt is activated with bit 8 set to 1
            % bitmask =  xxxx xxx0 xxxx xxxx
            controlWord = bitand(bin2dec('1111 1110 1111 1111'), controlWord);
            [answer, OK] = me.writeObject(index, subindex, [controlWord 0]);
            if ~OK
                fprintf('[Epos resumeHaltOperation] Failed to resume halt operation\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                if OK
                    fprintf('[Epos resumeHaltOperation] Operation resumed from halt\n');
                end
            end
        end

        %=======================================================================
        %> @fn [velocityControlPIgains, OK] = readVelocityControlParam()
        %> @brief  reads the parameters of the velocity control
        %>
        %>  
        %> @retval velocityControlPIgains Values of PI gains in velocity
        %>                                control mode
        %> @retval OK                     A boolean if all went ok or not.
        %=======================================================================
        function [velocityControlPIgains, OK] = readVelocityControlParam(me)
            index = me.objectIndex('VelocityControlParameterSet');
            subindex = uint8(1);

            [pGain, OK] = me.readObject(index, subindex);
            
            if(OK)
                OK = ~me.checkError(pGain(2:3));
                if OK 
                    velocityControlPIgains.pGain = typecast(pGain(4), 'int16');
                else
                    velocityControlPIgains.pGain = 'error';
                    OK = false;
                    return;
                end
            else
                velocityControlPIgains.pGain = [];
                OK = false;
                return;
            end
            
            subindex = uint8(2);
            [iGain, OK] = me.readObject(index, subindex);
            
            if(OK)
                OK = ~me.checkError(iGain(2:3));
                if OK 
                    velocityControlPIgains.iGain = typecast(iGain(4), 'int16');
                else
                    velocityControlPIgains.iGain = 'error';
                    OK = false;
                    return;
                end
            else
                OK = false;
                velocityControlPIgains.iGain = [];
                return;
            end
            
        end
        
         %=======================================================================
        %> @fn [OK] = setVelocityControlParam(me, propGain, intGain)
        %> @brief change gains of the velocity control loop
        %>
        %> Gains range
        %> range : [-2147483648|2147483647]
        %>
        %=======================================================================
        
        function [OK] = setVelocityControlParam(me, pGain, iGain)
            
            % validate attributes first
            if(pGain< 0 || pGain> 2^15 - 1)
                fprintf('[Epos setVelocityControlParam] Error pGain out of range\n');
                OK = false;
                return;
            end
            if(iGain< 0 || iGain> 2^15 - 1)
                fprintf('[Epos setVelocityControlParam] Error iGain out of range\n');
                OK = false;
                return;
            end
            % grant it is a int16
            pGain = int16(pGain);
            iGain = int16(iGain);

            index = me.objectIndex('VelocityControlParameterSet');
            % set pGain           
            subindex = uint8(1);
            [answer, OK] = me.writeObject(index, subindex, [pGain 0]);
            if ~OK
                fprintf('[Epos setVelocityControlParam] Failed to set velocity control proportional gain\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
            
            % set iGain.
            subindex = uint8(2);
            [answer, OK] = me.writeObject(index, subindex, [iGain 0]);
            if ~OK
                fprintf('[Epos setVelocityControlParam] Failed to set velocity control integral gain\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
        end
        function printVelocityControlParam(me)
            [param, OK] = me.readVelocityControlParam();
            if OK
                fprintf('[Epos printVelocityControlParam] Velocity control gains\n');
                fprintf('Proportional gain: %d\n', param.pGain);
                fprintf('Integral gain: %d\n', param.iGain);
            else
                fprintf('[Epos printVelocityControlParam] ERROR Unable to read velocity control parameters\n');
            end
        end
        %=======================================================================
        %=======================================================================
        function [positionControlPIDgains, OK] = readPositionControlParam(me)
            index = me.objectIndex('PositionControlParameterSet');
            
            % get pGain
            subindex = uint8(1);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    positionControlPIDgains.pGain = typecast(answer(4), 'int16');
                else
                    positionControlPIDgains.pGain = 'error';
                    OK = false;
                    return;
                end
            else
                positionControlPIDgains.pGain = [];
                OK = false;
                return;
            end
            % get iGain
            subindex = uint8(2);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    positionControlPIDgains.iGain = typecast(answer(4), 'int16');
                else
                    positionControlPIDgains.iGain = 'error';
                    OK = false;
                    return;
                end
            else
                positionControlPIDgains.iGain = [];
                OK = false;
                return;
            end
            % get dGain
            subindex = uint8(3);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    positionControlPIDgains.dGain = typecast(answer(4), 'int16');
                else
                    positionControlPIDgains.dGain = 'error';
                    OK = false;
                    return;
                end
            else
                positionControlPIDgains.dGain = [];
                OK = false;
                return;
            end
            % get vFeedForward
            subindex = uint8(4);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    positionControlPIDgains.vFeedForward = typecast(answer(4), 'int16');
                else
                    positionControlPIDgains.vFeedForward = 'error';
                    OK = false;
                    return;
                end
            else
                positionControlPIDgains.vFeedForward = [];
                OK = false;
                return;
            end
            % get aFeedForward
            subindex = uint8(5);
            [answer, OK] = me.readObject(index, subindex);
            if(OK)
                OK = ~me.checkError(answer(2:3));
                if OK 
                    positionControlPIDgains.aFeedForward = typecast(answer(4), 'int16');
                else
                    positionControlPIDgains.aFeedForward = 'error';
                    OK = false;
                    return;
                end
            else
                positionControlPIDgains.aFeedForward = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [OK] = setPositionControlParam(me, pGain, iGain, dGain, vFeed, aFeed)
            
            % validate attributes first
            if(pGain < 0 || pGain > 32767)
                fprintf('[Epos setPositionControlParam] ERROR pGain out of range\n');
                OK = false;
                return;
            end
            if(iGain < 0 || iGain > 32767)
                fprintf('[Epos setPositionControlParam] ERROR iGain out of range\n');
                OK = false;
                return;
            end
            if(dGain < 0 || dGain > 32767)
                fprintf('[Epos setPositionControlParam] ERROR dGain out of range\n');
                OK = false;
                return;
            end
            if(vFeed < 0 || vFeed > 65535)
                fprintf('[Epos setPositionControlParam] ERROR vFeed out of range\n');
                OK = false;
                return;
            end
            if(aFeed < 0 || aFeed > 65535)
                fprintf('[Epos setPositionControlParam] ERROR aFeed out of range\n');
                OK = false;
                return;
            end
            % grant it is uint16
            pGain = uint16(pGain);
            iGain = uint16(iGain);
            dGain = uint16(dGain);
            vFeed = uint16(vFeed);
            aFeed = uint16(aFeed);

            % set pGain
            index = me.objectIndex('PositionControlParameterSet');
			subindex = uint8(1);
            [answer, OK] = me.writeObject(index, subindex, [pGain 0]);
            if ~OK
                fprintf('[Epos setPositionControlParam] Failed to set position control proportional gain\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
            % set iGain
            subindex = uint8(2);
            [answer, OK] = me.writeObject(index, subindex, [iGain 0]);
            if ~OK
                fprintf('[Epos setPositionControlParam] Failed to set position control integral gain\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
            % set dGain
            subindex = uint8(3);
            [answer, OK] = me.writeObject(index, subindex, [dGain 0]);
            if ~OK
                fprintf('[Epos setPositionControlParam] Failed to set position control diferential gain\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
            % set vFeed
            subindex = uint8(4);
            [answer, OK] = me.writeObject(index, subindex, [vFeed 0]);
            if ~OK
                fprintf('[Epos setPositionControlParam] Failed to set position control velocity feed forward factor\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
            % set aFeed
            subindex = uint8(5);
            [answer, OK] = me.writeObject(index, subindex, [aFeed 0]);
            if ~OK
                fprintf('[Epos setPositionControlParam] Failed to set position control acceleration feed forward factor\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
        end
        %=======================================================================
        %=======================================================================
        function printPositionControlParam(me)
            [param, OK] = me.readPositionControlParam();
            if OK
                fprintf('[Epos printPositionControlParam] Position control parameters:\n');
                fprintf('Proportional gain: %d\n', param.pGain);
                fprintf('Integral gain: %d\n', param.iGain);
                fprintf('Differential gain: %d\n', param.dGain);
                fprintf('Velocity feedforward factor: %d\n', param.vFeedForward);
                fprintf('Acceleration feedforward factor: %d\n', param.aFeedForward);
            else
                fprintf('[Epos printPositionControlParam] ERROR unable to read position control parameters\n');
            end
        end
        %=======================================================================
        %=======================================================================
        function [followingError, OK] = readFollowingError(me)

            index = me.objectIndex('FollowingErrorActualValue');
            subindex = uint8(0);

            [answer, OK] = me.readObject(index, subindex);
            if OK
                OK = ~me.checkError(answer(2:3));
                if OK 
                    followingError = typecast(answer(4), 'int16');
                else
                    followingError = 'error';
                    OK = false;
                    return;
                end
            else
                followingError = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [maxFollowingError, OK] = readMaxFollowingError(me)

            index = me.objectIndex('MaximalFollowingError');
            subindex = uint8(0);

            [answer, OK] = me.readObject(index, subindex);
            if OK
                OK = ~me.checkError(answer(2:3));
                if OK 
                    maxFollowingError = typecast(answer(4:5), 'uint32');
                else
                    maxFollowingError = 'error';
                    OK = false;
                    return;
                end
            else
                maxFollowingError = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [OK] = setMaxFollowingError(me, maxFollowingError)

            % validate attributes
            if(maxFollowingError<0 || maxFollowingError > 2^32 - 1)
                fprintf('[Epos setMaxFollowingError] ERROR maxFollowingError out of range\n');
                OK = false;
                return;
            end

            index = me.objectIndex('MaximalFollowingError');
            subindex = uint8(0);

            % grant is a uint32
            maxFollowingError = uint32(maxFollowingError);
            [answer, OK] = me.writeObject(index, subindex, typecast(maxFollowingError, 'uint16') );
            if ~OK
                fprintf('[Epos setPositionControlParam] Failed to set maximum following error\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
        end
        %=======================================================================
        %=======================================================================
        function [position, OK] = readPositionValue(me)

            index = me.objectIndex('PositionActualValue');
            subindex = uint8(0);

            [answer, OK] = me.readObject(index, subindex);
            if OK
                OK = ~me.checkError(answer(2:3));
                if OK 
                    position = typecast(answer(4:5), 'int32');
                else
                    position = 'error';
                    OK = false;
                    return;
                end
            else
                position = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [positionWindow, OK] = readPositionWindow(me)

            index = me.objectIndex('PositionWindow');
            subindex = uint8(0);

            [answer, OK] = me.readObject(index, subindex);
            if OK
                OK = ~me.checkError(answer(2:3));
                if OK 
                    positionWindow = typecast(answer(4:5), 'uint32');
                else
                    positionWindow = 'error';
                    OK = false;
                    return;
                end
            else
                positionWindow = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [OK] = setPositionWindow(me, positionWindow)

            % validate attributes
            if(positionWindow<0 || positionWindow > 2^32 - 1)
                fprintf('[Epos setMaxFollowingError] ERROR positionWindow out of range\n');
                OK = false;
                return;
            end

            index = me.objectIndex('PositionWindow');
            subindex = uint8(0);

            % grant is a uint32
            positionWindow = uint32(positionWindow);
            [answer, OK] = me.writeObject(index, subindex, typecast(positionWindow, 'uint16') );
            if ~OK
                fprintf('[Epos setPositionControlParam] Failed to set position positionWindow\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
        end
        %=======================================================================
        %=======================================================================
        function [positionWindowTime, OK] = readPositionWindowTime(me)

            index = me.objectIndex('PositionWindowTime');
            subindex = uint8(0);

            [answer, OK] = me.readObject(index, subindex);
            if OK
                OK = ~me.checkError(answer(2:3));
                if OK 
                    positionWindowTime = answer(4);
                else
                    positionWindowTime = 'error';
                    OK = false;
                    return;
                end
            else
                positionWindowTime = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [OK] = setPositionWindowTime(me, positionWindowTime)

            % validate attributes
            if(positionWindowTime<0 || positionWindowTime > 2^16 - 1)
                fprintf('[Epos setMaxFollowingError] ERROR positionWindowTime out of range\n');
                OK = false;
                return;
            end

            index = me.objectIndex('PositionWindowTime');
            subindex = uint8(0);

            % grant is a uint16
            positionWindowTime = uint16(positionWindowTime);
            [answer, OK] = me.writeObject(index, subindex, [positionWindowTime 0] );
            if ~OK
                fprintf('[Epos setPositionControlParam] Failed to set position positionWindowTime\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
        end
        %=======================================================================
        %=======================================================================
        function [velocity, OK] = readVelocityValue(me)

            index = me.objectIndex('VelocityActualValue');
            subindex = uint8(0);

            [answer, OK] = me.readObject(index, subindex);
            if OK
                OK = ~me.checkError(answer(2:3));
                if OK 
                    velocity = typecast(answer(4:5), 'int32');
                else
                    velocity = 'error';
                    OK = false;
                    return;
                end
            else
                velocity = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [velocity, OK] = readVelocityValueAveraged(me)

            index = me.objectIndex('VelocityActualValueAveraged');
            subindex = uint8(0);

            [answer, OK] = me.readObject(index, subindex);
            if OK
                OK = ~me.checkError(answer(2:3));
                if OK 
                    velocity = typecast(answer(4:5), 'int32');
                else
                    velocity = 'error';
                    OK = false;
                    return;
                end
            else
                velocity = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [current, OK] = readCurrentValue(me)

            index = me.objectIndex('CurrentActualValue');
            subindex = uint8(0);

            [answer, OK] = me.readObject(index, subindex);
            if OK
                OK = ~me.checkError(answer(2:3));
                if OK 
                    current = typecast(answer(4), 'int16');
                else
                    current = 'error';
                    OK = false;
                    return;
                end
            else
                current = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [current, OK] = readCurrentValueAveraged(me)

            index = me.objectIndex('CurrentActualValueAveraged');
            subindex = uint8(0);

            [answer, OK] = me.readObject(index, subindex);
            if OK
                OK = ~me.checkError(answer(2:3));
                if OK 
                    current = typecast(answer(4), 'int16');
                else
                    current = 'error';
                    OK = false;
                    return;
                end
            else
                current = [];
                OK = false;
                return;
            end
		end
                
        %=======================================================================
        %=======================================================================
        function [homeOffset, OK] = readHomeOffset(me)

            index = me.objectIndex('HomeOffset');
            subindex = uint8(0);

            [answer, OK] = me.readObject(index, subindex);
            if OK
                OK = ~me.checkError(answer(2:3));
                if OK 
                    homeOffset = typecast(answer(4:5), 'int32');
                else
                    homeOffset = 'error';
                    OK = false;
                    return;
                end
            else
                homeOffset = [];
                OK = false;
                return;
            end
        end
        %=======================================================================
        %=======================================================================
        function [OK] = setHomeOffset(me, homeOffset)

            % validate attributes
            if(homeOffset< - 2^31 || homeOffset > 2^31 - 1)
                fprintf('[Epos setHomeOffset] ERROR homeOffset out of range\n');
                OK = false;
                return;
            end

            index = me.objectIndex('HomeOffset');
            subindex = uint8(0);

            % grant is a int32
            homeOffset = int32(homeOffset);
            [answer, OK] = me.writeObject(index, subindex, typecast(homeOffset, 'uint16') );
            if ~OK
                fprintf('[Epos setHomeOffset] Failed to set home offset\n');
                return;
            else
                OK = ~me.checkError(answer(2:3));
                %check for errors
                if(~OK)
                    return;
                end
            end
        end
    end
end