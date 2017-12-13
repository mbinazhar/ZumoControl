function mubassirSubscriber(~, message)
    %mubassirSubscriber Subscriber callback function for pose data    
    %   mubassirSubscriber(~,MESSAGE) returns data of detected markers
    global zumoPose
    global robotIds
    n = size(message.Poses);
    for i = 1 : n(1,1)
        id = message.Poses(i).Orientation.X;
        x =  message.Poses(i).Position.X;
        y = -message.Poses(i).Position.Y;
        % - minus added to confirm with ROS Co-ordinate
        % x front, y left, theta = 0 in positive x direction, counter
        % clockwise increase, remains within -pi and pi
        theta = message.Poses(i).Orientation.Y;
        if id==robotIds(1,1)
            zumoPose(1,:) = [x y theta];
        elseif id==robotIds(1,2)
            zumoPose(2,:) = [x y theta];
        elseif id==robotIds(1,3)
            zumoPose(3,:) = [x y theta];
        end
    end
end