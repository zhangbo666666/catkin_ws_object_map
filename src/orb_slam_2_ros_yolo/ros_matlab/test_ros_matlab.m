% Test publish a topic in matlab
rosshutdown
rosinit

global sub;
while(1)
    
    chatpub = rospublisher('/talker', 'std_msgs/String');

    msg = rosmessage(chatpub);
    msg.Data = 'Hello, from matlab';

    send(chatpub, msg);

    latchpub = rospublisher('/talker', 'IsLatching', true);
    pause(3);

    sub = rossubscriber('/talker', @ObjectsCallBack);
    pause(1);

end

function ObjectsCallBack()
    msg = rosmessage(sub);
    msg.Data
end