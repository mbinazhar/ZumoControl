function sonar3Subscriber(~, message)
    %mubassirSubscriber Subscriber callback function for pose data    
    %   mubassirSubscriber(~,MESSAGE) returns data of detected markers
    global sonar
	sonar(1,4) = message.Range_;
end