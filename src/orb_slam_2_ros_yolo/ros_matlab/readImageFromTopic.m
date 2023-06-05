% Manhattan frame estimation
% Subscribe the surface normal of an object point cloud
% Publish the matrix of frame estimation

rosinit

sub = rossubscriber('/camera/color/image_raw', @ObjectsCallBack);

% The subscriber callback function requires at least two input arguments. 
% function subCallback(src,msg)
% The first argument, src, is the associated subscriber object. 
% The second argument, msg, is the received message object.
function ObjectsCallBack(sub, img)
    disp('Read an image');
    img_ = readImage(sub.LatestMessage);
    imshow(img_);
end
