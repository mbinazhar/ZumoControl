 function lidarSubscriber(~, message)
    %mubassirSubscriber Subscriber callback function for pose data    
    %   mubassirSubscriber(~,MESSAGE) returns data of detected markers
%     persistent a;
    global lidar
   
    message.Ranges;
    a = message.Ranges([360-90 360-35 1 36 91])';
  
    for(i=1:5)
        if a(i) ~=Inf
    lidar(i) = a(i);
        end
end