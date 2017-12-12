function [] = ZigbeeRcvCallback(obj, event)
    global zumoSensors
    try
        rcvString = fscanf(obj,'%c',obj.BytesAvailable);
        startIdx = strfind(rcvString,'S,');
        if startIdx>1
            id = str2double(rcvString(startIdx+2:startIdx+2));
%             zumoSensors(id,:) = [rcvString(startIdx+3:startIdx+3)-48
%                                 rcvString(startIdx+4:startIdx+4)-48
%                                 rcvString(startIdx+5:startIdx+5)-48
%                                 rcvString(startIdx+6:startIdx+6)-48
%                                 rcvString(startIdx+7:startIdx+7)-48
%                                 rcvString(startIdx+8:startIdx+8)-48
%                                 rcvString(startIdx+9:startIdx+9)-48
%                                 rcvString(startIdx+10:startIdx+10)-48
%                                 rcvString(startIdx+11:startIdx+11)-48
%                                 ];
           zumoSensors(id,1) = rcvString(startIdx+3:startIdx+3)-48;
           zumoSensors(id,2) = rcvString(startIdx+4:startIdx+4)-48;
           zumoSensors(id,3) = rcvString(startIdx+5:startIdx+5)-48;
           zumoSensors(id,4) = rcvString(startIdx+6:startIdx+6)-48;
                         
        end
    catch
%         Empty Catch
    end
end

