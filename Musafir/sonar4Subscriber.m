function sonar4Subscriber(~, message)
    %mubassirSubscriber Subscriber callback function for pose data    
    %   mubassirSubscriber(~,MESSAGE) returns data of detected markers
    global sonar
	sonar(1,5) = message.Range_;
end