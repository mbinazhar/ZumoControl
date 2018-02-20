 function gpsSubscriber(~, message)
    %mubassirSubscriber Subscriber callback function for pose data    
    %   mubassirSubscriber(~,MESSAGE) returns data of detected markers
%     persistent a;
    global LatLon;
    message;
     LatLon(1) = message.Latitude;
     LatLon(2) = message.Longitude;
end