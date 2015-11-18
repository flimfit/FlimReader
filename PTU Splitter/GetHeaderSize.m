function [header_size, end_tag_size] = GetHeaderSize(fid,format)

    switch format
        case '.ptu'

            % some constants
            tyEmpty8      = hex2dec('FFFF0008');
            tyBool8       = hex2dec('00000008');
            tyInt8        = hex2dec('10000008');
            tyBitSet64    = hex2dec('11000008');
            tyColor8      = hex2dec('12000008');
            tyFloat8      = hex2dec('20000008');
            tyTDateTime   = hex2dec('21000008');
            tyFloat8Array = hex2dec('2001FFFF');
            tyAnsiString  = hex2dec('4001FFFF');
            tyWideString  = hex2dec('4002FFFF');
            tyBinaryBlob  = hex2dec('FFFFFFFF');
            % RecordTypes
            rtPicoHarpT3     = hex2dec('00010303');% (SubID = $00 ,RecFmt: $01) (V1), T-Mode: $03 (T3), HW: $03 (PicoHarp)
            rtPicoHarpT2     = hex2dec('00010203');% (SubID = $00 ,RecFmt: $01) (V1), T-Mode: $02 (T2), HW: $03 (PicoHarp)
            rtHydraHarpT3    = hex2dec('00010304');% (SubID = $00 ,RecFmt: $01) (V1), T-Mode: $03 (T3), HW: $04 (HydraHarp)
            rtHydraHarpT2    = hex2dec('00010204');% (SubID = $00 ,RecFmt: $01) (V1), T-Mode: $02 (T2), HW: $04 (HydraHarp)
            rtHydraHarp2T3   = hex2dec('01010304');% (SubID = $01 ,RecFmt: $01) (V2), T-Mode: $03 (T3), HW: $04 (HydraHarp)
            rtHydraHarp2T2   = hex2dec('01010204');% (SubID = $01 ,RecFmt: $01) (V2), T-Mode: $02 (T2), HW: $04 (HydraHarp)
            rtTimeHarp260NT3 = hex2dec('00010305');% (SubID = $00 ,RecFmt: $01) (V1), T-Mode: $03 (T3), HW: $05 (TimeHarp260N)
            rtTimeHarp260NT2 = hex2dec('00010205');% (SubID = $00 ,RecFmt: $01) (V1), T-Mode: $02 (T2), HW: $05 (TimeHarp260N)
            rtTimeHarp260PT3 = hex2dec('00010306');% (SubID = $00 ,RecFmt: $01) (V1), T-Mode: $03 (T3), HW: $06 (TimeHarp260P)
            rtTimeHarp260PT2 = hex2dec('00010206');% (SubID = $00 ,RecFmt: $01) (V1), T-Mode: $02 (T2), HW: $06 (TimeHarp260P)

            fprintf(1,'\n');
            Magic = fread(fid, 8, '*char');
            if not(strcmp(Magic(Magic~=0)','PQTTTR'))
                error('Magic invalid, this is not an PTU file.');
            end;
            Version = fread(fid, 8, '*char');
            fprintf(1,'Tag Version: %s\n', Version);

            % there is no repeat.. until (or do..while) construct in matlab so we use
            % while 1 ... if (expr) break; end; end;

            metadata = struct();

            while 1
                % read Tag Head
                TagIdent = fread(fid, 32, '*char'); % TagHead.Ident
                TagIdent = (TagIdent(TagIdent ~= 0))'; % remove #0 and more more readable
                TagIdx = fread(fid, 1, 'int32');    % TagHead.Idx
                TagTyp = fread(fid, 1, 'uint32');   % TagHead.Typ
                                                    % TagHead.Value will be read in the
                                                    % right type function  
                if TagIdx > -1
                  EvalName = [TagIdent '_' int2str(TagIdx + 1)];
                else
                  EvalName = TagIdent;
                end
                fprintf(1,'\n   %-40s', EvalName);  
                % check Typ of Header
                switch TagTyp
                    case tyEmpty8
                        fread(fid, 1, 'int64');   
                        fprintf(1,'<Empty>');
                    case tyBool8
                        TagInt = fread(fid, 1, 'int64');
                        if TagInt==0
                            fprintf(1,'FALSE');
                            metadata.(EvalName) = false;
                        else
                            fprintf(1,'TRUE');
                            metadata.(EvalName) = true;
                        end            
                    case tyInt8
                        TagInt = fread(fid, 1, 'int64');
                        fprintf(1,'%d', TagInt);
                        metadata.(EvalName) = TagInt;
                    case tyBitSet64
                        TagInt = fread(fid, 1, 'int64');
                        fprintf(1,'%X', TagInt);
                        metadata.(EvalName) = TagInt;
                    case tyColor8    
                        TagInt = fread(fid, 1, 'int64');
                        fprintf(1,'%X', TagInt);
                        metadata.(EvalName) = TagInt;
                    case tyFloat8
                        TagFloat = fread(fid, 1, 'double');
                        fprintf(1, '%e', TagFloat);
                        metadata.(EvalName) = TagFloat;
                    case tyFloat8Array
                        TagInt = fread(fid, 1, 'int64');
                        fprintf(1,'<Float array with %d Entries>', TagInt / 8);
                        fseek(fid, TagInt, 'cof');
                    case tyTDateTime
                        TagFloat = fread(fid, 1, 'double');
                        fprintf(1, '%s', datestr(datenum(1899,12,30)+TagFloat)); % display as Matlab Date String
                        metadata.(EvalName) = datenum(1899,12,30)+TagFloat;
                    case tyAnsiString
                        TagInt = fread(fid, 1, 'int64');
                        TagString = fread(fid, TagInt, '*char');
                        TagString = (TagString(TagString ~= 0))';
                        fprintf(1, '%s', TagString);
                        if TagIdx > -1
                           EvalName = [TagIdent '_' int2str(TagIdx + 1)];
                        end;   
                        metadata.(EvalName) = TagString;
                    case tyWideString 
                        % Matlab does not support Widestrings at all, just read and
                        % remove the 0's (up to current (2012))
                        TagInt = fread(fid, 1, 'int64');
                        TagString = fread(fid, TagInt, '*char');
                        TagString = (TagString(TagString ~= 0))';
                        fprintf(1, '%s', TagString);
                        if TagIdx > -1
                           EvalName = [TagIdent '_' int2str(TagIdx + 1)];
                        end;
                        metadata.(EvalName) = TagString;
                    case tyBinaryBlob
                        TagInt = fread(fid, 1, 'int64');
                        fprintf(1,'<Binary Blob with %d Bytes>', TagInt);
                        fseek(fid, TagInt, 'cof');    
                    otherwise
                        error('Illegal Type identifier found! Broken file?');
                end;
                if strcmp(TagIdent, 'Header_End')
                    break
                end
            end

            end_tag_size = 48;

            
        case '.pt3'
            
        
            Ident = char(fread(fid, 16, 'char'));
            fprintf(1,'      Identifier: %s\n', Ident);

            FormatVersion = deblank(char(fread(fid, 6, 'char')'));
            fprintf(1,'  Format Version: %s\n', FormatVersion);

            if not(strcmp(FormatVersion,'2.0'))
               fprintf(1,'\n\n      Warning: This program is for version 2.0 only. Aborted.');
               STOP;
            end;

            CreatorName = char(fread(fid, 18, 'char'));
            fprintf(1,'    Creator Name: %s\n', CreatorName);

            CreatorVersion = char(fread(fid, 12, 'char'));
            fprintf(1,' Creator Version: %s\n', CreatorVersion);

            FileTime = char(fread(fid, 18, 'char'));
            fprintf(1,'       File Time: %s\n', FileTime);

            CRLF = char(fread(fid, 2, 'char'));

            CommentField = char(fread(fid, 256, 'char'));
            fprintf(1,'         Comment: %s\n', CommentField);


            %
            % The following is binary file header information
            %


            Curves = fread(fid, 1, 'int32');
            fprintf(1,'Number of Curves: %d\n', Curves);

            BitsPerRecord = fread(fid, 1, 'int32');
            fprintf(1,'   Bits / Record: %d\n', BitsPerRecord);

            RoutingChannels = fread(fid, 1, 'int32');
            fprintf(1,'Routing Channels: %d\n', RoutingChannels);

            NumberOfBoards = fread(fid, 1, 'int32');
            fprintf(1,'Number of Boards: %d\n', NumberOfBoards);

            ActiveCurve = fread(fid, 1, 'int32');
            fprintf(1,'    Active Curve: %d\n', ActiveCurve);

            MeasurementMode = fread(fid, 1, 'int32');
            fprintf(1,'Measurement Mode: %d\n', MeasurementMode);

            SubMode = fread(fid, 1, 'int32');
            fprintf(1,'        Sub-Mode: %d\n', SubMode);

            RangeNo = fread(fid, 1, 'int32');
            fprintf(1,'       Range No.: %d\n', RangeNo);

            Offset = fread(fid, 1, 'int32');
            fprintf(1,'          Offset: %d ns \n', Offset);

            AcquisitionTime = fread(fid, 1, 'int32');
            fprintf(1,'Acquisition Time: %d ms \n', AcquisitionTime);

            StopAt = fread(fid, 1, 'int32');
            fprintf(1,'         Stop At: %d counts \n', StopAt);

            StopOnOvfl = fread(fid, 1, 'int32');
            fprintf(1,'Stop on Overflow: %d\n', StopOnOvfl);

            Restart = fread(fid, 1, 'int32');
            fprintf(1,'         Restart: %d\n', Restart);

            DispLinLog = fread(fid, 1, 'int32');
            fprintf(1,' Display Lin/Log: %d\n', DispLinLog);

            DispTimeFrom = fread(fid, 1, 'int32');
            fprintf(1,' Display Time Axis From: %d ns \n', DispTimeFrom);

            DispTimeTo = fread(fid, 1, 'int32');
            fprintf(1,'   Display Time Axis To: %d ns \n', DispTimeTo);

            DispCountFrom = fread(fid, 1, 'int32');
            fprintf(1,'Display Count Axis From: %d\n', DispCountFrom); 

            DispCountTo = fread(fid, 1, 'int32');
            fprintf(1,'  Display Count Axis To: %d\n', DispCountTo);

            for i = 1:8
            DispCurveMapTo(i) = fread(fid, 1, 'int32');
            DispCurveShow(i) = fread(fid, 1, 'int32');
            end;

            for i = 1:3
            ParamStart(i) = fread(fid, 1, 'float');
            ParamStep(i) = fread(fid, 1, 'float');
            ParamEnd(i) = fread(fid, 1, 'float');
            end;

            RepeatMode = fread(fid, 1, 'int32');
            fprintf(1,'        Repeat Mode: %d\n', RepeatMode);

            RepeatsPerCurve = fread(fid, 1, 'int32');
            fprintf(1,'     Repeat / Curve: %d\n', RepeatsPerCurve);

            RepeatTime = fread(fid, 1, 'int32');
            fprintf(1,'        Repeat Time: %d\n', RepeatTime);

            RepeatWait = fread(fid, 1, 'int32');
            fprintf(1,'   Repeat Wait Time: %d\n', RepeatWait);

            ScriptName = char(fread(fid, 20, 'char'));
            fprintf(1,'        Script Name: %s\n', ScriptName);


            %
            % The next is a board specific header
            %


            HardwareIdent = char(fread(fid, 16, 'char'));
            fprintf(1,'Hardware Identifier: %s\n', HardwareIdent);

            HardwareVersion = char(fread(fid, 8, 'char'));
            fprintf(1,'   Hardware Version: %s\n', HardwareVersion);

            HardwareSerial = fread(fid, 1, 'int32');
            fprintf(1,'   HW Serial Number: %d\n', HardwareSerial);

            SyncDivider = fread(fid, 1, 'int32');
            fprintf(1,'       Sync Divider: %d\n', SyncDivider);

            CFDZeroCross0 = fread(fid, 1, 'int32');
            fprintf(1,'CFD ZeroCross (Ch0): %4i mV\n', CFDZeroCross0);

            CFDLevel0 = fread(fid, 1, 'int32');
            fprintf(1,'CFD Discr     (Ch0): %4i mV\n', CFDLevel0);

            CFDZeroCross1 = fread(fid, 1, 'int32');
            fprintf(1,'CFD ZeroCross (Ch1): %4i mV\n', CFDZeroCross1);

            CFDLevel1 = fread(fid, 1, 'int32');
            fprintf(1,'CFD Discr     (Ch1): %4i mV\n', CFDLevel1);

            Resolution = fread(fid, 1, 'float');
            fprintf(1,'         Resolution: %5.6f ns\n', Resolution);

            % below is new in format version 2.0

            RouterModelCode      = fread(fid, 1, 'int32');
            RouterEnabled        = fread(fid, 1, 'int32');

            % Router Ch1
            RtChan1_InputType    = fread(fid, 1, 'int32');
            RtChan1_InputLevel   = fread(fid, 1, 'int32');
            RtChan1_InputEdge    = fread(fid, 1, 'int32');
            RtChan1_CFDPresent   = fread(fid, 1, 'int32');
            RtChan1_CFDLevel     = fread(fid, 1, 'int32');
            RtChan1_CFDZeroCross = fread(fid, 1, 'int32');
            % Router Ch2
            RtChan2_InputType    = fread(fid, 1, 'int32');
            RtChan2_InputLevel   = fread(fid, 1, 'int32');
            RtChan2_InputEdge    = fread(fid, 1, 'int32');
            RtChan2_CFDPresent   = fread(fid, 1, 'int32');
            RtChan2_CFDLevel     = fread(fid, 1, 'int32');
            RtChan2_CFDZeroCross = fread(fid, 1, 'int32');
            % Router Ch3
            RtChan3_InputType    = fread(fid, 1, 'int32');
            RtChan3_InputLevel   = fread(fid, 1, 'int32');
            RtChan3_InputEdge    = fread(fid, 1, 'int32');
            RtChan3_CFDPresent   = fread(fid, 1, 'int32');
            RtChan3_CFDLevel     = fread(fid, 1, 'int32');
            RtChan3_CFDZeroCross = fread(fid, 1, 'int32');
            % Router Ch4
            RtChan4_InputType    = fread(fid, 1, 'int32');
            RtChan4_InputLevel   = fread(fid, 1, 'int32');
            RtChan4_InputEdge    = fread(fid, 1, 'int32');
            RtChan4_CFDPresent   = fread(fid, 1, 'int32');
            RtChan4_CFDLevel     = fread(fid, 1, 'int32');
            RtChan4_CFDZeroCross = fread(fid, 1, 'int32');

            % Router settings are meaningful only for an existing router:

            if RouterModelCode>0

                fprintf(1,'-------------------------------------\n'); 
                fprintf(1,'   Router Model Code: %d \n', RouterModelCode);
                fprintf(1,'      Router Enabled: %d \n', RouterEnabled);
                fprintf(1,'-------------------------------------\n'); 

                % Router Ch1 
                fprintf(1,'RtChan1 InputType   : %d \n', RtChan1_InputType);
                fprintf(1,'RtChan1 InputLevel  : %4i mV\n', RtChan1_InputLevel);
                fprintf(1,'RtChan1 InputEdge   : %d \n', RtChan1_InputEdge);
                fprintf(1,'RtChan1 CFDPresent  : %d \n', RtChan1_CFDPresent);
                fprintf(1,'RtChan1 CFDLevel    : %4i mV\n', RtChan1_CFDLevel);
                fprintf(1,'RtChan1 CFDZeroCross: %4i mV\n', RtChan1_CFDZeroCross);
                fprintf(1,'-------------------------------------\n'); 

                % Router Ch2
                fprintf(1,'RtChan2 InputType   : %d \n', RtChan2_InputType);
                fprintf(1,'RtChan2 InputLevel  : %4i mV\n', RtChan2_InputLevel);
                fprintf(1,'RtChan2 InputEdge   : %d \n', RtChan2_InputEdge);
                fprintf(1,'RtChan2 CFDPresent  : %d \n', RtChan2_CFDPresent);
                fprintf(1,'RtChan2 CFDLevel    : %4i mV\n', RtChan2_CFDLevel);
                fprintf(1,'RtChan2 CFDZeroCross: %4i mV\n', RtChan2_CFDZeroCross);
                fprintf(1,'-------------------------------------\n'); 

                % Router Ch3
                fprintf(1,'RtChan3 InputType   : %d \n', RtChan3_InputType);
                fprintf(1,'RtChan3 InputLevel  : %4i mV\n', RtChan3_InputLevel);
                fprintf(1,'RtChan3 InputEdge   : %d \n', RtChan3_InputEdge);
                fprintf(1,'RtChan3 CFDPresent  : %d \n', RtChan3_CFDPresent);
                fprintf(1,'RtChan3 CFDLevel    : %4i mV\n', RtChan3_CFDLevel);
                fprintf(1,'RtChan3 CFDZeroCross: %4i mV\n', RtChan3_CFDZeroCross);
                fprintf(1,'-------------------------------------\n'); 

                % Router Ch4
                fprintf(1,'RtChan4 InputType   : %d \n', RtChan4_InputType);
                fprintf(1,'RtChan4 InputLevel  : %4i mV\n', RtChan4_InputLevel);
                fprintf(1,'RtChan4 InputEdge   : %d \n', RtChan4_InputEdge);
                fprintf(1,'RtChan4 CFDPresent  : %d \n', RtChan4_CFDPresent);
                fprintf(1,'RtChan4 CFDLevel    : %4i mV\n', RtChan4_CFDLevel);
                fprintf(1,'RtChan4 CFDZeroCross: %4i mV\n', RtChan4_CFDZeroCross);
                fprintf(1,'-------------------------------------\n'); 

            end;

            %
            % The next is a T3 mode specific header
            %

            ExtDevices = fread(fid, 1, 'int32');
            fprintf(1,'   External Devices: %d\n', ExtDevices);

            Reserved1 = fread(fid, 1, 'int32');
            fprintf(1,'          Reserved1: %d\n', Reserved1);

            Reserved2 = fread(fid, 1, 'int32');
            fprintf(1,'          Reserved2: %d\n', Reserved2);

            CntRate0 = fread(fid, 1, 'int32');
            fprintf(1,'   Count Rate (Ch0): %d Hz\n', CntRate0);

            CntRate1 = fread(fid, 1, 'int32');
            fprintf(1,'   Count Rate (Ch1): %d Hz\n', CntRate1);

            StopAfter = fread(fid, 1, 'int32');
            fprintf(1,'         Stop After: %d ms \n', StopAfter);

            StopReason = fread(fid, 1, 'int32');
            fprintf(1,'        Stop Reason: %d\n', StopReason);

            Records = fread(fid, 1, 'uint32');
            fprintf(1,'  Number Of Records: %d\n', Records);

            ImgHdrSize = fread(fid, 1, 'int32');
            fprintf(1,'Imaging Header Size: %d bytes\n', ImgHdrSize);

            %Special header for imaging 
            ImgHdr = fread(fid, ImgHdrSize, 'int32');

            end_tag_size = 0;
            
    end
   
    header_size = ftell(fid);
    fprintf(1, '\n');
end
