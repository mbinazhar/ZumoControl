function sonarSubscriber(~, message)
    %mubassirSubscriber Subscriber callback function for pose data    
    %   mubassirSubscriber(~,MESSAGE) returns data of detected markers
    global sonar
    sonar(1)=message.Adc0;
    sonar(2)=message.Adc1;
    sonar(3)=message.Adc2;
    sonar(4)=message.Adc3;
    sonar(5)=message.Adc4;
% 	sonar(1,1) = message.Range_;
end