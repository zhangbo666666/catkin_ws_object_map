%function CalculateManhattanFrameFromPointCloud()
%CALCULATEMANHATTANFRAMEFROMPOINTCLOUD Summary of this function goes here
%   Detailed explanation goes here

rosshutdown
rosinit
%rosinit('192.168.0.103') % to connect master 
disp('Start to calculate the Manhattan frame');

global pub;
global msg;

pub = rospublisher('/orb_slam2_rgbd/object_MFE','std_msgs/Float64MultiArray');

msg = rosmessage(pub);

subPointCloud = rossubscriber('/orb_slam2_rgbd/single_object_point_cloud', 'sensor_msgs/PointCloud2', @pointCloudManhattanFrameEstimationCallBack, 'BufferSize', 30);

%end

% The subscriber callback function requires at least two input arguments. 
% function subCallback(src,msg)
% The first argument, src, is the associated subscriber object. 
% The second argument, msg, is the received message object.
function pointCloudManhattanFrameEstimationCallBack(subPointCloud, ptCloud)
    disp('Read a point cloud');
    %cloud = subPointCloud.LatestMessage;
    %scatter3(pointCloud); % Plot the point cloud
    
    tic
    cloud_xyz = readXYZ(ptCloud);
    cloud = pointCloud(cloud_xyz);
    % normals = pcnormals(ptCloud,k) additionally specifies k, the number of points used for local plane fitting
    normals = pcnormals(cloud, 20);
    size(normals,1);

    object_mf = ManhattanFrameEstimation(normals);
    toc
    global msg;
    global pub;
    
    seq = double(ptCloud.Header.Seq);
    data = [seq, object_mf, toc]
    
    % msg.Data = num2str(data)
    msg.Data = data;

    send(pub, msg);
end


function est_mf_ = ManhattanFrameEstimation(PCnormals)
Initialize;
if size(PCnormals,1)==0
    est_mf_=[];
    return;
end
normal_data = PCnormals';
data.normal_data = normal_data;
est_mf = Fn_BranchnBound (data, BnB_param, exp_save_path);
est_mf = est_mf(:,1:3);
est_mf_ = [est_mf(1,1), est_mf(1,2), est_mf(1,3), est_mf(2,1), est_mf(2,2), est_mf(2,3), est_mf(3,1), est_mf(3,2), est_mf(3,3)];

end
