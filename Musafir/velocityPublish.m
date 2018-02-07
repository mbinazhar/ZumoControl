function velocityPublish( v_L, v_R)
global lwheel_pub;
global rwheel_pub;
    
    msg = rosmessage('std_msgs/Int16');
    msg.Data =  v_L;
    
    send(lwheel_pub,msg);
    
    msg.Data =  v_R;
    send(rwheel_pub,msg);
%     rospublisher('/rwheel_desired_vel',msg);
end