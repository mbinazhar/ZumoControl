function mubassirSubscriber(~, message)
    %mubassirSubscriber Subscriber callback function for pose data    
    %   mubassirSubscriber(~,MESSAGE) returns data of detected markers
    %   n = number of marekers
    %   markers, vector of marker pose data

    %pos = [message.Linear.X message.Linear.Y message.Linear.Z];
    %orient = [message.Angular.X message.Angular.Y message.Angular.Z];
    global robots
    n = size(message.Poses);
    %robots = zeros(n,4);
    for z = 1 : n(1,1)
        id = message.Poses(z).Orientation.X;
        x = message.Poses(z).Position.X;
        y = message.Poses(z).Position.Y;
        theta = message.Poses(z).Orientation.Y;
        robots(z,:) = [id x y theta];
    end
    robots;
end