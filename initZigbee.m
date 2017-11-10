delete(instrfindall);
s= serial ('COM3','baudrate', 9600);
set (s,'databits',8);
set (s,'stopbits',1);
set (s,'parity','none');


set(s,'BytesAvailableFcnCount', 11);
set(s,'BytesAvailableFcnMode','byte');

fopen(s);


set(s,'BytesAvailableFcn',@ZigbeeRcvCallback);




