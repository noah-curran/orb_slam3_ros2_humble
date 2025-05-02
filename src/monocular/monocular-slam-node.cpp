#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

#include <vector>
#include <map>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera/color/image_raw",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    
    m_orb_slam_image_publisher = this->create_publisher<sensor_msgs::msg::Image>(
        "orb_slam_image",
        10
    );
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    // keys
    // get map
    // get visual map
    cv::Mat im;
    cv::Scalar standardColor(0,255,0);
    cv::Scalar odometryColor(255,0,0);
    float imageScale = m_SLAM->GetImageScale();
    ORB_SLAM3::Frame frame = m_SLAM->GetCurrentFrame();
    // else if(mState==Tracking::OK)
    //     {
    //         vCurrentKeys = mvCurrentKeys;
    //         vbVO = mvbVO;
    //         vbMap = mvbMap;
    //     }
    std::vector<cv::KeyPoint> vCurrentKeys = frame.mvKeys;
    int N = vCurrentKeys.size();
    std::vector<bool> vbVO = std::vector<bool>(N,false);
    std::vector<bool> vbMap = std::vector<bool>(N,false);

    std::map<long unsigned int, cv::Point2f> mMatchedInImage;

    std::vector<cv::KeyPoint> vOutlierKeys;
    std::vector<ORB_SLAM3::MapPoint*> vpOutlierMPs;
    vOutlierKeys.clear();
    vOutlierKeys.reserve(N);
    vpOutlierMPs.clear();
    vpOutlierMPs.reserve(N);

    for(int i=0;i<N;i++)
    {
        ORB_SLAM3::MapPoint* pMP = frame.mvpMapPoints[i];
        if(pMP)
        {
            if(!frame.mvbOutlier[i])
            {
                if(pMP->Observations()>0)
                    vbMap[i]=true;
                else
                    vbVO[i]=true;

                mMatchedInImage[pMP->mnId] = vCurrentKeys[i].pt;
            }
            else
            {
                vpOutlierMPs.push_back(pMP);
                vOutlierKeys.push_back(vCurrentKeys[i]);
            }
        }
    }

    im = m_SLAM->CopyTo(im);

    if (im.empty()) {
        RCLCPP_WARN(this->get_logger(), "no image available");
        return;
    }

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    int mnTracked=0;
    int mnTrackedVO=0;
    const float r = 5;
    int n = vCurrentKeys.size();
    for(int i=0;i<n;i++)
    {
        if(vbVO[i] || vbMap[i])
        {
            cv::Point2f pt1,pt2;
            cv::Point2f point;
            if(imageScale != 1.f)
            {
                point = vCurrentKeys[i].pt / imageScale;
                float px = vCurrentKeys[i].pt.x / imageScale;
                float py = vCurrentKeys[i].pt.y / imageScale;
                pt1.x=px-r;
                pt1.y=py-r;
                pt2.x=px+r;
                pt2.y=py+r;
            }
            else
            {
                point = vCurrentKeys[i].pt;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;
            }

            // This is a match to a MapPoint in the map
            if(vbMap[i])
            {
                cv::rectangle(im,pt1,pt2,standardColor);
                cv::circle(im,point,2,standardColor,-1);
                mnTracked++;
            }
            else // This is match to a "visual odometry" MapPoint created in the last frame
            {
                cv::rectangle(im,pt1,pt2,odometryColor);
                cv::circle(im,point,2,odometryColor,-1);
                mnTrackedVO++;
            }
        }
    }

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "orb_frame";
    
    std::string encoding;
    switch (im.type()) {
        case CV_8UC1:
            encoding = "mono8";
            break;
        case CV_8UC3:
            encoding = "bgr8";
            break;
        case CV_16UC1:
            encoding = "mono16";
            break;
        case CV_32FC1:
            encoding = "32FC1";
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "unsupported image format");
    }

    sensor_msgs::msg::Image::SharedPtr ros_image;
    try {
        ros_image = cv_bridge::CvImage(header, encoding, im).toImageMsg();
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: &s", e.what());
        return;
    }

    m_orb_slam_image_publisher->publish(*ros_image);
}
