/*
 *功能：订阅深度相机的rgb图像和深度图像信息，发送目标的相对位置
 *Author:Andy Wei
 *Date:2018年7月9日
 */
#include <ros/ros.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>//包含CVBRIDGE图像转换库
#include <sensor_msgs/image_encodings.h>//ROS图像类型的编码函数
#include <image_transport/image_transport.h>//用来在ROS系统中的话题上发布和订阅图像信息
#include <geometry_msgs/Vector3.h>

#include <opencv2/opencv.hpp>//OpenCV标准头文件
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp> //图像跟踪库
#include <opencv2/core/ocl.hpp>

using namespace std;
using namespace cv;

static const string TRACK = "Tracking";
static const string DEPTH = "Depth Image";

Mat frame;//定义视频流
Mat depthImage;
Rect2d selectRect;
Point origin;
Rect2d result;

bool select_flag = false;//定义图像框选标志
bool bRenewROI = false;
bool bBeginTrack = false;
bool enable_get_depth = false;

float center_depth;//输出图像的中心位置及深度
Point center;
float dist_val[5] ;//取样五个深度值
Ptr<Tracker> tracker = TrackerKCF::create();//定义跟踪器

std::string sub_rgb_img="/camera/rgb/image_raw";//定义发布和订阅的话题名称
std::string sub_depth_img="/camera/depth/image";
std::string pub_pose="/target_pose";

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

/*
 *功能：鼠标回调函数，用于实现基于鼠标点击的目标框选
 *具体实现：通过设置多个标志来实现具体的功能
 */
void onMouse(int event,int x,int y,int,void*)
{
    if(select_flag)
    {
        selectRect.x=MIN(origin.x,x);
        selectRect.y=MIN(origin.y,y);
        selectRect.width=abs(x-origin.x);
        selectRect.height=abs(y-origin.y);
        selectRect &=Rect2d(0,0,frame.cols,frame.rows);
    }
    if(event==CV_EVENT_LBUTTONDOWN)
    {
        bBeginTrack=false;
        select_flag=true;
        origin=Point(x,y);
        selectRect=Rect2d(x,y,0,0);
    }
    else if(event==CV_EVENT_LBUTTONUP)
    {
        select_flag=false;
        bRenewROI=true;
    }
}


/*
 *功能：实现基于普通摄像头的图像跟踪类
 *具体实现：
 *   参数：句柄nh,图像转换句柄it_,图像订阅类image_sub_;
 *   构造函数：订阅图像话题，加载图像显示窗口
 *   析够函数：销毁图像显示窗口
 *
 *   图像话题的回调函数：
 *        定义跟踪器->图像转换->跟踪器初始化（图像+目标）->跟踪器更新->绘制跟踪框->显示结果->读取对应位置的深度值
 */
class ROS_TRACKER
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    ros::Publisher pub_pose_topic= nh_.advertise<geometry_msgs::Vector3>(pub_pose,1);
    geometry_msgs::Vector3 target;
public:
    ROS_TRACKER():it_(nh_)   //构造函数
    {
        image_sub_ = it_.subscribe(sub_rgb_img,1,&ROS_TRACKER::imageCb,this);
        //image_sub_ = it_.subscribe("/camera/rgb/image_raw",1,&ROS_TRACKER::imageCb,this);
        depth_sub_ = it_.subscribe(sub_depth_img,1,&ROS_TRACKER::depthCb,this);
        //depth_sub_ = it_.subscribe("/camera/depth/image",1,&ROS_TRACKER::depthCb,this);
        namedWindow(TRACK);
        namedWindow(DEPTH);
    }
    ~ROS_TRACKER()  //析够函数
    {
        destroyWindow(TRACK);
        destroyWindow(DEPTH);
    }
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s",e.what());
            return;
        }       
        cv_ptr->image.copyTo(frame);//将图像复制到定义好的容器
        cv::setMouseCallback(TRACK,onMouse,0);//设置图像的鼠标事件

        if(bRenewROI) //新的目标被选取后会重新初始化tracker
        {
            tracker->init(frame,selectRect);
            bBeginTrack=true;
            bRenewROI=false;
            enable_get_depth=false;
        }
        if(bBeginTrack)
        {
            tracker->update(frame,result);
            rectangle(frame,result,Scalar(0,255,255),1,8);
            center.x=result.x+(result.width/2); 
            center.y=result.y+(result.height/2);
            target.x=center.x;
            target.y=center.y;
            putText(frame,"Traget Center X:"+SSTR(float(center.x)),Point(100,20),FONT_HERSHEY_SIMPLEX,0.75,Scalar(50,170,50),2);
            putText(frame,"Traget Center Y:"+SSTR(float(center.y)),Point(100,50),FONT_HERSHEY_SIMPLEX,0.75,Scalar(50,170,50),2);
            enable_get_depth=true;
            //rectangle(frame,selectRect,Scalar(255,0,0),2,8,0);
        }
        else
            rectangle(frame,selectRect,Scalar(255,0,0),2,8,0);
        imshow(TRACK,frame);
        waitKey(1);

    }

    void depthCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
            cv_ptr->image.copyTo(depthImage);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",e.what());
        }

        if(enable_get_depth)
        {
            //cout<<"We can get Depth"<<endl;
            dist_val[0] = depthImage.at<float>(result.y+result.height/3 , result.x+result.width/3) ;
            dist_val[1] = depthImage.at<float>(result.y+result.height/3 , result.x+2*result.width/3) ;
            dist_val[2] = depthImage.at<float>(result.y+2*result.height/3 , result.x+result.width/3) ;
            dist_val[3] = depthImage.at<float>(result.y+2*result.height/3 , result.x+2*result.width/3) ;
            dist_val[4] = depthImage.at<float>(result.y+result.height/2 , result.x+result.width/2) ;
            float distance = 0;
            int num_depth_points = 5;
            for(int i = 0; i < 5; i++)
            {
                if(dist_val[i] > 0.4 && dist_val[i] < 10.0)
                    distance += dist_val[i];
                else
                    num_depth_points--;
            }
            distance /= num_depth_points;
            target.z=distance;
            //cout<<"Target Depth="<<SSTR(float(distance))<<endl;
            pub_pose_topic.publish(target);
            putText(depthImage,"Traget Distance:"+SSTR(float(distance)),Point(100,20),FONT_HERSHEY_SIMPLEX,0.75,Scalar(0,255,0),2);
                      
        }
        imshow(DEPTH,depthImage);
        waitKey(1);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_tracker");
    ros::param::get("~sub_rgb_img", sub_rgb_img);
    ros::param::get("~sub_depth_img", sub_depth_img);
    ros::param::get("~pub_pose", pub_pose);
    ROS_TRACKER obj;//视频跟踪类
    ros::spin();
}
