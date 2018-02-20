 function magSubscriber(~, message)
    %mubassirSubscriber Subscriber callback function for pose data    
    %   mubassirSubscriber(~,MESSAGE) returns data of detected markers
%     persistent a;
    global heading;
    heading = message.Data;
end