#include "RGBDNode_dataset_new.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detection_3d");
    ros::start();

    if(argc != 5)
    {
        ROS_ERROR ("The input parameters are not right.");
        ros::shutdown();
        return 1;
    }

    string dataset_num = argv[1];
    int all_images_num = std::stoi(argv[2]);
    int image_interval_num = std::stoi(argv[3]); // every interval image number to set a keyframe
    int keyframeDelay = std::stoi(argv[4]);

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    RGBDNode node (node_handle, image_transport, dataset_num, all_images_num, image_interval_num, keyframeDelay);
    ros::spin();

    ros::shutdown();

    return 0;
}

// Original program
/*
RGBDNode::RGBDNode (ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport,
                    string dataset_num, int all_images_num, int image_interval_num, int keyframeDelay):
    NodeDatasetNew (node_handle, image_transport), mDatasetNum(dataset_num), mAllImagesNum(all_images_num),
    mImageIntervalNum(image_interval_num), mKeyframeDelay(keyframeDelay)
{

    mDatasetCompleted = 0;
    dataset_completed_publisher_ = node_handle.advertise<std_msgs::Int32MultiArray> ("/orb_slam2_rgbd/dataset_completed", 100);

    mbIsKeyframe = true;
    mTcw = cv::Mat::ones(4, 4, CV_32F);

    string path = "/home/ubuntu/Datasets/SceneNN/";

    depthSeqencePath = path + mDatasetNum + "/depth/depth";
    colorSeqencePath = path + mDatasetNum + "/image/image";
    trajectoryPath = path + mDatasetNum + "/trajectory.log";

    string filePath = "./" + mDatasetNum;
    if(mkdir(filePath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0) // mkdir(), 成功返回0，错误返�???-1
        cout << "\n Create a folder, named " << mDatasetNum << endl;
    else
        cout << "\n Fail to create a folder, named " << mDatasetNum << endl;

    // obtain trajectory in trajectory.log
    ifstream file(trajectoryPath, std::ios::in);
    char temp[200] = {0};

    while(1)
    {
        static int img_num = 1;
        current_frame_time_ = ros::Time::now();

        cv::Mat depthImage;
        cv::Mat bgrImage;

        if(!grabRGBD(depthSeqencePath, colorSeqencePath, depthImage, bgrImage))
            break;

        if(!file.getline(temp, sizeof(temp)))
        {
            break;
        }

        // obtain the transform matrix of the keyframe
        for(int n = 0; n < 4; n++)
        {
            if(!file.getline(temp, sizeof(temp)))
                break;
            string data_;
            vector<string> data;
            stringstream word(temp);

            for(int i = 0; i < 4; i++)
            {
                word >> data_;
                data.push_back(data_);
            }

            mTcw.at<float>(n, 0) = atof(data[0].c_str());
            mTcw.at<float>(n, 1) = atof(data[1].c_str());
            mTcw.at<float>(n, 2) = atof(data[2].c_str());
            mTcw.at<float>(n, 3) = atof(data[3].c_str());
        }
        //cout << "mTcw = \n" << mTcw << endl;

        cout << "Create keyframe = " << img_num << endl;

        // the number to run the Update function
        int allKeyframesNum = (int)(mAllImagesNum / mImageIntervalNum);
        cout << "Total keyframes = " << allKeyframesNum << endl;

        Update(mDatasetNum, allKeyframesNum, bgrImage, depthImage, mbIsKeyframe, mTcw);
        PublishDatasetCompleted(mDatasetCompleted);
        cout << "Keyframe delay = " << mKeyframeDelay << endl;

        ros::Duration(mKeyframeDelay).sleep(); // wait for mask rcnn

        img_num++;

        // image_interval_num * 5
        int num = 0;
        while(num != (mImageIntervalNum - 1) * 5)
        {
            if(!file.getline(temp, sizeof(temp)))
                break;
            num++;
        }
    }
    file.close();

    mDatasetCompleted = 1;
    PublishDatasetCompleted(mDatasetCompleted);

    cout << "Finish to read the dataset. " << endl;
}
*/

RGBDNode::RGBDNode (ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport,
                    string dataset_num, int all_images_num, int image_interval_num, int keyframeDelay):
    NodeDatasetNew (node_handle, image_transport), mDatasetNum(dataset_num), mAllImagesNum(all_images_num),
    mImageIntervalNum(image_interval_num), mKeyframeDelay(keyframeDelay)
{

    mDatasetCompleted = 0;
    dataset_completed_publisher_ = node_handle.advertise<std_msgs::Int32MultiArray> ("/orb_slam2_rgbd/dataset_completed", 100);

    mbIsKeyframe = true;
    mTcw = cv::Mat::ones(4, 4, CV_32F);

    string path = "/home/zb/dataset/435/";
     
    depthSeqencePath = path + mDatasetNum + "/depth/depth";
    colorSeqencePath = path + mDatasetNum + "/image/image";
    thermalSeqencePath = path + mDatasetNum + "/thermal/thermal";
    tempSeqencePath = path + mDatasetNum + "/temp/temp";
    trajectoryPath = path + mDatasetNum + "/trajectory.log";

    string filePath = "./" + mDatasetNum;
    if(mkdir(filePath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0) // mkdir(), 成功返回0，错误返�???-1
        cout << "\n Create a folder, named " << mDatasetNum << endl;
    else
        cout << "\n Fail to create a folder, named " << mDatasetNum << endl;

    // obtain trajectory in trajectory.log
    ifstream file(trajectoryPath, std::ios::in);
    char temp[200] = {0};


    //edit by dx1
    while(ros::ok())
    {
        static int img_num = 1;
        current_frame_time_ = ros::Time::now();

        cv::Mat depthImage;
        cv::Mat bgrImage;
        cv::Mat thermalImage;
        cv::Mat tempImage;

        //if(!grabRGBD(depthSeqencePath, colorSeqencePath, depthImage, bgrImage))
        if(!grabRGBD(depthSeqencePath, colorSeqencePath, thermalSeqencePath, tempSeqencePath, depthImage, bgrImage, thermalImage, tempImage))
            break;
        

        if(!file.getline(temp, sizeof(temp)))
        {
            break;
        }

        // obtain the transform matrix of the keyframe
        for(int n = 0; n < 4; n++)
        {
            if(!file.getline(temp, sizeof(temp)))
                break;
            string data_;
            vector<string> data;
            stringstream word(temp);

            for(int i = 0; i < 4; i++)
            {
                word >> data_;
                data.push_back(data_);
            }

            mTcw.at<float>(n, 0) = atof(data[0].c_str());
            mTcw.at<float>(n, 1) = atof(data[1].c_str());
            mTcw.at<float>(n, 2) = atof(data[2].c_str());
            mTcw.at<float>(n, 3) = atof(data[3].c_str());

            //  edit by dx
            // mTcw.at<float>(n, 0) = 0;
            // mTcw.at<float>(n, 1) = 0;
            // mTcw.at<float>(n, 2) = 0;
            // mTcw.at<float>(n, 3) = 0;
            // mTcw.at<float>(n, n) = 1;
            
            
        }
        //cout << "mTcw = \n" << mTcw << endl;

        cout << "Create keyframe = " << img_num << endl;

        // the number to run the Update function
        int allKeyframesNum = (int)(mAllImagesNum / mImageIntervalNum);
        cout << "Total keyframes = " << allKeyframesNum << endl;

        // Update(mDatasetNum, allKeyframesNum, bgrImage, depthImage, mbIsKeyframe, mTcw);

         //edit by dx
        Update(mDatasetNum, allKeyframesNum, bgrImage, depthImage, thermalImage, tempImage, mbIsKeyframe, mTcw);
        
        //uesd for record rosbag
        //  Update(bgrImage, depthImage, mbIsKeyframe, mTcw);

        PublishDatasetCompleted(mDatasetCompleted);
        cout << "Keyframe delay = " << mKeyframeDelay << endl;
         
        //edit by dx1
        ros::Duration(mKeyframeDelay/1000.0).sleep(); // wait for mask rcnn
        cout<<"111.gxc"<<endl;

        img_num++;

        // image_interval_num * 5
        int num = 0;
        while(num != (mImageIntervalNum - 1) * 5)
        {
            if(!file.getline(temp, sizeof(temp)))
                break;
            num++;
        }
        //edit by dx
        static int nummm = 0;
        if (nummm > all_images_num)
             break;
        else
             nummm++;
    }
    file.close();

    mDatasetCompleted = 1;
    PublishDatasetCompleted(mDatasetCompleted);

    cout << "Finish to read the dataset. " << endl;
}

RGBDNode::~RGBDNode ()
{
    delete rgb_subscriber_;
}

/// grab RGB and Depth images from the dataset
bool RGBDNode::grabRGBD(std::string& depthSeqencePath, std::string& colorSeqencePath, cv::Mat& depthImage, cv::Mat& bgrImage)
{
    static int index = 0;
    int frameIndex = index * mImageIntervalNum + 1; // every interval image number to set a keyframe

    cout << "\n" << "Frame index = " << frameIndex << endl;

    std::stringstream depthFilename;
    depthFilename << depthSeqencePath << std::setfill('0') << std::setw(5) << frameIndex << ".png";

    std::stringstream colorFilename;
    colorFilename << colorSeqencePath << std::setfill('0') << std::setw(5) << frameIndex << ".png";

    bool isExist = isFileExist(depthFilename.str().c_str()) || isFileExist(colorFilename.str().c_str());

    if (isExist == false){
        frameIndex = 0;
        return false;
    }

    bgrImage = cv::imread(colorFilename.str(), -1);
    depthImage = cv::imread(depthFilename.str(), -1);

    index++;
    return true;
}

bool RGBDNode::grabRGBD(std::string& depthSeqencePath,
                        std::string& colorSeqencePath,
                        std::string& thermalSeqencePath,
                        std::string& tempSeqencePath,
                        cv::Mat& depthImage,
                        cv::Mat& bgrImage,
                        cv::Mat& thermalImage,
                        cv::Mat& tempImage)
{
    static int index = 0;
    int frameIndex = index * mImageIntervalNum + 1; // every interval image number to set a keyframe

    cout << "\n" << "Frame index = " << frameIndex << endl;

    std::stringstream depthFilename;
    depthFilename << depthSeqencePath << std::setfill('0') << std::setw(5) << frameIndex << ".png";

    std::stringstream colorFilename;
    colorFilename << colorSeqencePath << std::setfill('0') << std::setw(5) << frameIndex << ".png";

    std::stringstream thermalFilename;
    thermalFilename << thermalSeqencePath << std::setfill('0') << std::setw(5) << frameIndex << ".png";

    std::stringstream tempFilename;
    tempFilename << tempSeqencePath << std::setfill('0') << std::setw(5) << frameIndex << ".png";
    

    bool isExist = isFileExist(depthFilename.str().c_str())
            || isFileExist(colorFilename.str().c_str())
            || isFileExist(thermalFilename.str().c_str())
            || isFileExist(tempFilename.str().c_str());

    if (isExist == false){
        frameIndex = 0;
        return false;
    }
    
    depthImage = cv::imread(depthFilename.str(), -1);
    bgrImage = cv::imread(colorFilename.str(), -1);

    //edit by dx
    // depthImage = cv::imread("/home/gxc/dataset/SceneNN/3/depth/depth00034.png", -1);
    // bgrImage = cv::imread("/home/gxc/dataset/SceneNN/3/image/img_00365.png", -1);
    
    thermalImage = cv::imread(thermalFilename.str(), -1);
    tempImage = cv::imread(tempFilename.str(), -1);
    
    index++;
    return true;
}

bool RGBDNode::isFileExist(char const* filename)
{
    std::ifstream file_tmp(filename);
    if (!file_tmp.is_open())
    {
        return false;
    }
    file_tmp.close();
    return true;
}

void RGBDNode::PublishDatasetCompleted(int flag)
{
    std_msgs::Int32MultiArray msg;
    msg.data.push_back(std::stoi(mDatasetNum));
    msg.data.push_back(flag);
    dataset_completed_publisher_.publish(msg);
}
