function [] = sendSpeeds( s, zumoID, v_L, v_R  )
%UNTITLED Summary of this function goes here

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
APIframe(2:3,:) = ['00';'1A'];  % Length 
APIframe(4,:) = '10';           % Frame type
APIframe(5,:) = '01';           % Frame ID
APIframe(6:13,:) = destinationAddress_hex;
APIframe(14:15,:)= ['FF';'FE']; % Unknown 16-bit address
APIframe(16,:) = '00';
APIframe(17,:) = '00'; 

APIframe = hex2dec(APIframe);

APIframe(18:19,:) = double('D,'); % For speed values in mm/sec
APIframe(20:24,:) = double(sprintf('%04d,',v_L));
APIframe(25:29,:) = double(sprintf('%04d\n',v_R));


bitSum = sum( APIframe(4:length(APIframe) ) );
checksum = 255 - bitand(bitSum,255);
APIframe = [APIframe; checksum];


fwrite(s,APIframe,'uint8');
end
    
