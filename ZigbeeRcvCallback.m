function [] = ZigbeeRcvCallback(obj, event)
    
global zumoSensors;

try
% rcvString = fgetl(obj);
rcvString = fscanf(obj,'%c',obj.BytesAvailable);
startIdx = strfind(rcvString,'S,');
% if ~isempty(rcvString)
zumoSensors = str2num(rcvString(startIdx+2:startIdx+5))
% end
catch
    
end


end

