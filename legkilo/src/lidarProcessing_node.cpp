#include "utility.h"

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

enum MODE{SLICE1 = 1, SLICE2,  SLICE4};

class lidarProcessing : public ParamServer
{
    private:

    ros::Subscriber subFullScan;
    ros::Subscriber subLegOdom;
    ros::Publisher  pubCutCloud;

    sensor_msgs::PointCloud2 currentCloudMsg;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;

    pcl::PointCloud<PointXYZIRT>::Ptr   cutcloud01;
    pcl::PointCloud<PointXYZIRT>::Ptr   cutcloud02;
    pcl::PointCloud<PointXYZIRT>::Ptr   cutcloud03;
    pcl::PointCloud<PointXYZIRT>::Ptr   cutcloud04;

    pcl::PointCloud<PointXYZIRT>::Ptr   last_cutcloud01;
    pcl::PointCloud<PointXYZIRT>::Ptr   last_cutcloud02;
    pcl::PointCloud<PointXYZIRT>::Ptr   last_cutcloud03;
    pcl::PointCloud<PointXYZIRT>::Ptr   last_cutcloud04;


    pcl::PointCloud<PointXYZIRT>::Ptr   pub_cutcloud01;
    pcl::PointCloud<PointXYZIRT>::Ptr   pub_cutcloud02;
    pcl::PointCloud<PointXYZIRT>::Ptr   pub_cutcloud03;
    pcl::PointCloud<PointXYZIRT>::Ptr   pub_cutcloud04;

    pcl::PointCloud<PointXYZIRT>::Ptr   tempCloud;

    std_msgs::Header cloudHeader;

    std::mutex mtxOdom;
    std::deque<nav_msgs::Odometry> legQueue;

    int processingMode;

    int firstFrame;
    double curOdomTime;
    std::vector<float> thre;

    public:
    lidarProcessing()
    {
        subFullScan = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1000, &lidarProcessing::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        subLegOdom  = nh.subscribe<nav_msgs::Odometry>("/lio_sam/legOdom", 2000, &lidarProcessing::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        pubCutCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/cutcloud", 1000);

        processingMode = SLICE1;
        firstFrame = 1;
        curOdomTime = -1;
        thre.assign({0.25 / lidarFreq, 0.5 / lidarFreq, 0.75 / lidarFreq, 1/lidarFreq});

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());

        cutcloud01.reset(new pcl::PointCloud<PointXYZIRT>());
        cutcloud02.reset(new pcl::PointCloud<PointXYZIRT>());
        cutcloud03.reset(new pcl::PointCloud<PointXYZIRT>());
        cutcloud04.reset(new pcl::PointCloud<PointXYZIRT>());

        last_cutcloud01.reset(new pcl::PointCloud<PointXYZIRT>());
        last_cutcloud02.reset(new pcl::PointCloud<PointXYZIRT>());
        last_cutcloud03.reset(new pcl::PointCloud<PointXYZIRT>());
        last_cutcloud04.reset(new pcl::PointCloud<PointXYZIRT>());

        pub_cutcloud01.reset(new pcl::PointCloud<PointXYZIRT>());
        pub_cutcloud02.reset(new pcl::PointCloud<PointXYZIRT>());
        pub_cutcloud03.reset(new pcl::PointCloud<PointXYZIRT>());
        pub_cutcloud04.reset(new pcl::PointCloud<PointXYZIRT>());

        tempCloud.reset(new pcl::PointCloud<PointXYZIRT>());

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();

        cutcloud01->clear();
        cutcloud02->clear();
        cutcloud03->clear();
        cutcloud04->clear();

        pub_cutcloud01->clear();
        pub_cutcloud02->clear();
        pub_cutcloud03->clear();
        pub_cutcloud04->clear();

        tempCloud->clear();

    }

    ~lidarProcessing(){}

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& msg){
        std::lock_guard<std::mutex> lock1(mtxOdom);
        legQueue.push_back(*msg);
    }

    int odometryProcessing(){
        if(firstFrame){
            firstFrame = 0;
            return SLICE1;
        }

        std::unique_lock<std::mutex> lock1(mtxOdom);

        if(legQueue.empty()) return SLICE1;

        while(!legQueue.empty()){
                if(legQueue.front().header.stamp.toSec() < curOdomTime - 1.0){
                    legQueue.pop_front();
                }else{
                    break;
                }
        }

        if(legQueue.size() < 20) return SLICE1;

        int odomCount = 0;
        double linearSum = 0;
        double angSum = 0;
        for(int i = 0; i < legQueue.size(); ++i){
            if(legQueue[i].header.stamp.toSec() < curOdomTime){
                odomCount++;
                auto& seq = legQueue[i].twist.twist;
                linearSum += sqrt(seq.linear.x * seq.linear.x +  seq.linear.y * seq.linear.y + seq.linear.z * seq.linear.z);
                angSum += sqrt(seq.angular.x * seq.angular.x +  seq.angular.y * seq.angular.y + seq.angular.z * seq.angular.z);
            }else break;
        }

        if(odomCount < 100) return SLICE1;
        linearSum /= odomCount;
        angSum /= odomCount;

        double ratio =  max(linearSum / maxLinearVel, angSum / maxAngularVel); // linearSum/3  angSum/(2*Pi)

    
        //std::cout << "vel:  " <<linearSum << std::endl;
        //std::cout << "ratio:  " <<ratio << std::endl;

        // Basically, if the robot is moving fast, we need to cut more slices. eq. 20, 21
        return ratio >= 0.75 ? SLICE4 : (ratio >= 0.25 ? SLICE2 : SLICE1);

    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        curOdomTime = laserCloudMsg->header.stamp.toSec();
        processingMode = odometryProcessing();
        //std::cout << "Cut mode:  " << processingMode << std::endl;
        pointCloudProcessing(laserCloudMsg);
        resetParameters();
    }

    void cachePubCloud(pcl::PointCloud<PointXYZIRT>::Ptr& inputCloud, pcl::PointCloud<PointXYZIRT>::Ptr& outputCloud, float timeBias){
        *tempCloud = *inputCloud;
        for(int i = 0; i < tempCloud->points.size(); ++i){
            tempCloud->points[i].time = tempCloud->points[i].time  + timeBias;
        }
        *outputCloud += *tempCloud;
        tempCloud->clear();
    }

    void pointCloudProcessing(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        currentCloudMsg = *laserCloudMsg;
        cloudHeader = currentCloudMsg.header;

        pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        
    
        int cloudSize = laserCloudIn->points.size();

        if(cloudSize == 0) return;

        //static bool is_first_lidar = true;
        //*pub_cutcloud02 = *laserCloudIn;

        double startPointTime = laserCloudIn->points.front().time;
        if(correctStamp){
            ros::Time trueStamp = ros::Time().fromSec(cloudHeader.stamp.toSec() + startPointTime);
            cloudHeader.stamp = trueStamp;
        }


        for(int i=0; i< cloudSize; i++)
        {
                laserCloudIn->points[i].time  = laserCloudIn->points[i].time -startPointTime;
                if(laserCloudIn->points[i].time < thre[0])
                {
                    cutcloud01->push_back(laserCloudIn ->points[i]);
                }
                else if(laserCloudIn->points[i].time < thre[1])
                {
                    cutcloud02->push_back(laserCloudIn ->points[i]);
                }
                else if(laserCloudIn->points[i].time < thre[2])
                {
                    cutcloud03->push_back(laserCloudIn ->points[i]);
                }                
                else
                {
                    cutcloud04->push_back(laserCloudIn ->points[i]);
                }
        }

        switch (processingMode)
        {
        case SLICE1:
            *pub_cutcloud01 = *laserCloudIn;
            pubCloud(pub_cutcloud01, 0);
            break;
        
        case SLICE2:
            // part one
            cachePubCloud(last_cutcloud03,  pub_cutcloud01, - thre[1]);
            cachePubCloud(last_cutcloud04,  pub_cutcloud01, - thre[1]);
            cachePubCloud(cutcloud01, pub_cutcloud01, thre[1]);
            cachePubCloud(cutcloud02, pub_cutcloud01, thre[1]);
            pubCloud(pub_cutcloud01, - thre[1]);

            // part two
            *pub_cutcloud02 = *laserCloudIn;
            pubCloud(pub_cutcloud02, 0);
            break;


        case SLICE4:
            // part one
            cachePubCloud(last_cutcloud02,  pub_cutcloud01, - thre[0]);
            cachePubCloud(last_cutcloud03,  pub_cutcloud01, - thre[0]);
            cachePubCloud(last_cutcloud04, pub_cutcloud01, - thre[0]);
            cachePubCloud(cutcloud01, pub_cutcloud01, thre[2]);
            pubCloud(pub_cutcloud01, -thre[2]);


            //part two
            cachePubCloud(last_cutcloud03,  pub_cutcloud02, - thre[1]);
            cachePubCloud(last_cutcloud04,  pub_cutcloud02, - thre[1]);
            cachePubCloud(cutcloud01, pub_cutcloud02, thre[1]);
            cachePubCloud(cutcloud02, pub_cutcloud02, thre[1]);
            pubCloud(pub_cutcloud02, -thre[1]);

            //part three
            cachePubCloud(last_cutcloud04,  pub_cutcloud03, - thre[2]);
            cachePubCloud(cutcloud01,  pub_cutcloud03,  thre[0]);
            cachePubCloud(cutcloud02, pub_cutcloud03, thre[0]);
            cachePubCloud(cutcloud03, pub_cutcloud03, thre[0]);
            pubCloud(pub_cutcloud03, -thre[0]);

            //part four
            *pub_cutcloud04 = *laserCloudIn;
            pubCloud(pub_cutcloud04, 0);
            break;

        default:
            break;
        }

        last_cutcloud01->clear();
        last_cutcloud02->clear();
        last_cutcloud03->clear();
        last_cutcloud04->clear();

        *last_cutcloud01 = *cutcloud01;
        *last_cutcloud02 = *cutcloud02;
        *last_cutcloud03 = *cutcloud03;
        *last_cutcloud04 = *cutcloud04;

      }

    void pubCloud(const pcl::PointCloud<PointXYZIRT>::Ptr &thisCloud, double correctTime )
    {
        sensor_msgs::PointCloud2 tempCloud;
        pcl::toROSMsg(*thisCloud, tempCloud);
        tempCloud.header = cloudHeader;
        tempCloud.header.stamp = ros::Time().fromSec(cloudHeader.stamp.toSec() + correctTime);
        pubCutCloud.publish(tempCloud);

    }


};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "lio_sam");

    lidarProcessing CPC;

    
    ROS_INFO("\033[1;32m----> lidarProcessing Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}