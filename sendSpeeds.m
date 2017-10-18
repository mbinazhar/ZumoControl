function [] = sendSpeeds( s, zumoID, v_L, v_R  )
%UNTITLED Summary of this function goes here
leftSpeed=v_L+128;
rightSpeed=v_R+128;

if leftSpeed == 0
    leftSpeed=1
end
if rightSpeed == 0
    rightSpeed=1
end


switch zumoID
    case 1
        destinationAddress_hex = ['00';'13';'A2';'00';'40';'D6';'A6';'80'];
    case 2
        destinationAddress_hex = ['00';'13';'A2';'00';'41';'53';'03';'3A'];
    case 3    
        destinationAddress_hex = ['00';'13';'A2';'00';'40';'E7';'3E';'76'];
    otherwise
        destinationAddress_hex = ['00';'13';'A2';'00';'41';'53';'03';'3A'];
end

APIframe(1,1:2) = '7E';         % start condition 
APIframe(2:3,:) = ['00';'11'];  % Length 
APIframe(4,:) = '10';           % Frame type
APIframe(5,:) = '01';           % Frame ID
APIframe(6:13,:) = destinationAddress_hex;
APIframe(14:15,:)= ['FF';'FE']; % Unknown 16-bit address
APIframe(16,:) = '00';
APIframe(17,:) = '00'; 

APIframe(18:20,:) = [dec2hex(leftSpeed,2) ; dec2hex(rightSpeed,2) ; dec2hex(0,2)];

APIframe = hex2dec(APIframe);

bitSum = sum( APIframe(4:length(APIframe) ) );
checksum = 255 - bitand(bitSum,255);
APIframe = [APIframe; checksum];


fwrite(s,APIframe,'uint8');
end

