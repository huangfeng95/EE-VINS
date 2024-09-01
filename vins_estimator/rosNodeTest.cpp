/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <dv_ros_msgs/Event.h>
#include <dv_ros_msgs/EventArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"
#include "lock.h"

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
std::vector<dv_ros_msgs::EventArray::ConstPtr> event_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf, i_buf;
std::vector<std::vector<dvs_msgs::Event>> event_buffer_;
std::vector<dvs_msgs::Event> event_buffer;
std::vector<sensor_msgs::Imu> imu_buffer;
// std::mutex mutex_;
bool static processing;
double static imu_t0;
//input params for viode
int height_ = 480;
int weight_ = 752;
double Focus_ = 376;
double pixel_size_ = 1;
double Focus = 4644;
double pixel_size = 18.5;


void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void event_callback(const dv_ros_msgs::EventArray::ConstPtr &event_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    event_buf.push_back(event_msg);
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    i_buf.lock();
    imu_buffer.emplace_back(*imu_msg);
    i_buf.unlock();
    if (imu_t0==0)
    {
        imu_t0 = (*imu_msg).header.stamp.toSec();
    }
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

void event_cb_txt(const string& event_txt_path)
{
    ifstream event_txt(event_txt_path);
    if (event_txt.is_open())
    {
        string line;
        double t, t0;
        int x, y, p;
        getline(event_txt, line);
        while (line[0]=='#')
        {
            getline(event_txt, line);
            std::cout<<line<<endl;
        }
        event_txt >> t >> x >> y >> p;
        t0 = t;
        
        while (!event_txt.eof())
        {
            std::vector<dvs_msgs::Event> tmp_events;
            while ((t-t0)<=0.100&&(!event_txt.eof()))             
            {
                dvs_msgs::Event tmp_event;
                tmp_event.x = x;
                tmp_event.y = y;
                tmp_event.polarity = p;
                tmp_event.ts = ros::Time(t);
                tmp_events.emplace_back(tmp_event);
                event_txt >> t >> x >> y >> p;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                event_buffer_.emplace_back(tmp_events);
            }
            t0 = t;
        }
        cout<<fixed<<setprecision(9)<<"end!"<<endl;
        event_txt.close();
    }
    else
    {
        ROS_ERROR("Txt file not found in: %s", EVENT_TXT_PATH.c_str());
    }
}

void static txt_format_data_process()
{
    while (processing)
    {
        if (event_buffer_.size() != 0 && event_buffer.size() ==0)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (int i = 0; i < event_buffer_[0].size(); ++i)
            {
                event_buffer.emplace_back(event_buffer_[0][i]);
            }
            event_buffer_.erase(event_buffer_.begin());
        }
        if (imu_buffer.size()>0 && event_buffer.size()>0)
        {
            if (imu_buffer[imu_buffer.size() - 1].header.stamp.toSec()<(event_buffer[0].ts.toSec()+imu_t0))  //imu最晚时间戳早于事件组
            {
                imu_buffer.clear();
            }
            else if (imu_buffer[0].header.stamp.toSec()>(event_buffer[event_buffer.size() - 1].ts.toSec()+imu_t0)) //imu最早时间戳晚于事件组
            {
                event_buffer.clear();
            }
            else if (event_buffer[0].ts.toSec()>=event_buffer[event_buffer.size() - 1].ts.toSec())  //事件时间戳有误
            {
                event_buffer.clear();
            }
            else
            {
                double angular_velocity_x=0.0, angular_velocity_y=0.0,angular_velocity_z=0.0;
                double average_angular_rate_x, average_angular_rate_y,average_angular_rate_z;
               
                int cnt=0;//imu counter
                for(int i=0;i<imu_buffer.size();i++)
                {
                    if((imu_buffer[i].header.stamp.toSec() >= (event_buffer[0].ts.toSec()+imu_t0-0.003)) && (imu_buffer[i].header.stamp.toSec() <= (event_buffer[event_buffer.size() - 1].ts.toSec()+imu_t0+0.003)))
                    {
                        angular_velocity_x+=imu_buffer[i].angular_velocity.x;
                        angular_velocity_y+=imu_buffer[i].angular_velocity.y;
                        angular_velocity_z+=imu_buffer[i].angular_velocity.z;
                        imu_buffer.erase(imu_buffer.begin() + i); 
                        i--;
                        cnt++;
                    }
                }
                if (cnt > 0)
                {
                    double imu_average_angular_rate_x = angular_velocity_x/double(cnt);
                    double imu_average_angular_rate_y = angular_velocity_y/double(cnt);
                    double imu_average_angular_rate_z = angular_velocity_z/double(cnt);
                    
                    average_angular_rate_x = imu_average_angular_rate_x;
                    average_angular_rate_y = imu_average_angular_rate_z;
                    average_angular_rate_z = imu_average_angular_rate_y;
                    double average_angular_rate = std::sqrt((average_angular_rate_x*average_angular_rate_x) + (average_angular_rate_y*average_angular_rate_y) + (average_angular_rate_z*average_angular_rate_z));

                    //Motion  compensation
                    double t0=event_buffer[0].ts.toSec();//the first event
                    for(int i=0;i<event_buffer.size();++i){
                        t0 = std::min(t0,event_buffer[i].ts.toSec());
                    }

                    double time_diff = 0.0;//time diff
                    double max_time_diff = 0.0;//max time diff
                    std::vector<std::vector<int>>count_image(height_,std::vector<int>(weight_));//count image
                    
                    cv::Mat normalized_mean_timestamp_image_ = cv::Mat::zeros(height_,weight_,CV_32FC1);
                    for(int i=0;i<event_buffer.size();++i)
                    {
                        time_diff = double(event_buffer[i].ts.toSec()-t0);
                        if (time_diff >= 0.1) 
                        {
                            continue;
                        }
                        if (event_buffer[i].ts.toSec()>event_buffer[event_buffer.size()-1].ts.toSec())
                        {
                            continue;
                        }
                        
                        max_time_diff = std::max(max_time_diff,time_diff);

                        //Calculate the rotation offset of the event point
                        double x_angular=time_diff*average_angular_rate_x;
                        double y_angular=time_diff*average_angular_rate_y;
                        double z_angular=time_diff*average_angular_rate_z;
                                
                        int x=event_buffer[i].x - weight_/2; 
                        int y=event_buffer[i].y - height_/2;

                        //Angle of initial position of event point
                        double pre_x_angel = atan(y*pixel_size_/Focus_);
                        double pre_y_angel = atan(x*pixel_size_/Focus_);

                        //compensate
                        int compen_x = (int)((x*cos(z_angular) - sin(z_angular)*y) - (x - (Focus_*tan(pre_y_angel + y_angular)/pixel_size_)) + weight_/2);
                        int compen_y = (int)((x*sin(z_angular) + cos(z_angular)*y) - (y - (Focus_*tan(pre_x_angel - x_angular)/pixel_size_)) + height_/2);
                        event_buffer[i].x = compen_x;
                        event_buffer[i].y = compen_y;
                                
                        //count image and time image
                        if(compen_y < height_ && compen_y >= 0 && compen_x < weight_ && compen_x >= 0)
                        {
                            count_image[compen_y][compen_x]++;
                            normalized_mean_timestamp_image_.at<float>(compen_y, compen_x) +=  time_diff;
                        }
                    }

                    //算平均时间
                    int max_count = 0;
                    double mean_rel_ts = 0.0;
                    int count_rel_ts = 0;
                    for (int y = 0; y < normalized_mean_timestamp_image_.rows; ++y)
                    {
                        for (int x = 0; x < normalized_mean_timestamp_image_.cols; ++x)
                        {
                            if (count_image[y][x] != 0)
                            {
                                max_count = std::max(max_count,count_image[y][x]);
                                normalized_mean_timestamp_image_.at<float>(y, x) = normalized_mean_timestamp_image_.at<float>(y, x) / count_image[y][x];
                                mean_rel_ts += normalized_mean_timestamp_image_.at<float>(y, x);
                                count_rel_ts += 1;
                            }
                            else
                            {
                                normalized_mean_timestamp_image_.at<float>(y, x) = 0.;
                            }
                        }
                    }
                
                    if (count_rel_ts > 0)
                        mean_rel_ts /= (double)count_rel_ts;

                    normalized_mean_timestamp_image_ = (normalized_mean_timestamp_image_ - mean_rel_ts) / max_time_diff;              
                    
                    cv::Mat thresholded_mean_timestamp_image_ = cv::Mat::zeros(height_,weight_, CV_8UC1);
                    cv::threshold(normalized_mean_timestamp_image_,
                        thresholded_mean_timestamp_image_, 0, 1,
                        cv::THRESH_TOZERO);
                    
                    cv::blur(thresholded_mean_timestamp_image_, thresholded_mean_timestamp_image_, cv::Size(5,5));

                    double ev_stamp = (event_buffer[0].ts.toSec()+imu_t0+event_buffer[event_buffer.size() - 1].ts.toSec()+imu_t0)/2;
                    mutex_.lock();
                    estimator.ev_stamps.push_back(ev_stamp);
                    estimator.ev_maps.push_back(thresholded_mean_timestamp_image_);
                    mutex_.unlock();

                    event_buffer.clear();
                }
                else
                {
                    //release buffer
                    event_buffer.clear();
                }
            }
        }
    }
    std::cout<<"end data_process!"<<std::endl;
}

void static bag_format_data_process()
{
    while (processing)
    {
        if (event_buf.size() != 0)
        {
            mutex_.lock();
            if (event_buf[0]->events.size()>0)
            {
                for (int i = 0; i < event_buf[0]->events.size(); ++i)
                {
                    dvs_msgs::Event tmp_event;
                    tmp_event.x = event_buf[0]->events[i].x;
                    tmp_event.y = event_buf[0]->events[i].y;
                    tmp_event.polarity = event_buf[0]->events[i].polarity;
                    tmp_event.ts = event_buf[0]->events[i].ts;
                    event_buffer.emplace_back(tmp_event);
                }
            }
            event_buf.erase(event_buf.begin());
            mutex_.unlock();
        }
        if (imu_buffer.size()>0 && event_buffer.size()>0)
        {
            if (event_buffer[event_buffer.size() - 1].ts.toSec()-event_buffer[0].ts.toSec()<=0.05) //50ms
            {
                continue;
            }
            else
            {
                double angular_velocity_x=0.0, angular_velocity_y=0.0,angular_velocity_z=0.0;
                double average_angular_rate_x, average_angular_rate_y,average_angular_rate_z;
               
                int cnt=0;//imu counter
                i_buf.lock();
                for(int i=0;i<imu_buffer.size();i++)
                {
                    if((imu_buffer[i].header.stamp.toSec() >= (event_buffer[0].ts.toSec()-0.003)) && (imu_buffer[i].header.stamp.toSec() <= (event_buffer[event_buffer.size() - 1].ts.toSec()+0.003)))
                    {
                        angular_velocity_x+=imu_buffer[i].angular_velocity.x;
                        angular_velocity_y+=imu_buffer[i].angular_velocity.y;
                        angular_velocity_z+=imu_buffer[i].angular_velocity.z;
                        imu_buffer.erase(imu_buffer.begin() + i); 
                        i--;
                        cnt++;
                    }
                }
                imu_buffer.clear();
                i_buf.unlock();
                if (cnt > 0)
                {
                    //Calculate the average imu angular rates
                    // Follow calibration results
                    double imu_average_angular_rate_x = angular_velocity_x/double(cnt);
                    double imu_average_angular_rate_y = angular_velocity_y/double(cnt);
                    double imu_average_angular_rate_z = angular_velocity_z/double(cnt);

                    Eigen::Vector3d angular_imu,angular_camera;
                    angular_imu << imu_average_angular_rate_x, imu_average_angular_rate_y, imu_average_angular_rate_z; 
                    
                    angular_camera = RIC[0]*angular_imu;
                    
                    average_angular_rate_x = angular_camera[0];
                    average_angular_rate_y = angular_camera[1];
                    average_angular_rate_z = angular_camera[2];
                    double average_angular_rate = std::sqrt((average_angular_rate_x*average_angular_rate_x) + (average_angular_rate_y*average_angular_rate_y) + (average_angular_rate_z*average_angular_rate_z));

                    //Motion  compensation
                    double t0=event_buffer[0].ts.toSec();//the first event
                    for(int i=0;i<event_buffer.size();++i){
                        t0 = std::min(t0,event_buffer[i].ts.toSec());
                    }

                    double time_diff = 0.0;//time diff
                    double max_time_diff = 0.0;//max time diff
                    std::vector<std::vector<int>>count_image(ROW,std::vector<int>(COL));//count image
                    
                    cv::Mat normalized_mean_timestamp_image_ = cv::Mat::zeros(ROW,COL,CV_32FC1);
                    for(int i=0;i<event_buffer.size();++i)
                    {
                        time_diff = double(event_buffer[i].ts.toSec()-t0);
                        if (time_diff >= 0.1)   
                        {
                            continue;
                        }
                        if (event_buffer[i].ts.toSec()>event_buffer[event_buffer.size()-1].ts.toSec())
                        {
                            continue;
                        }
                        
                        max_time_diff = std::max(max_time_diff,time_diff);

                        //Calculate the rotation offset of the event point
                        double x_angular=time_diff*average_angular_rate_x;
                        double y_angular=time_diff*average_angular_rate_y;
                        double z_angular=time_diff*average_angular_rate_z;
                                
                        int x=event_buffer[i].x - COL/2; 
                        int y=event_buffer[i].y - ROW/2;

                        //Angle of initial position of event point
                        double pre_x_angel = atan(y*pixel_size/Focus);
                        double pre_y_angel = atan(x*pixel_size/Focus);

                        //compensate
                        int compen_x = (int)((x*cos(z_angular) - sin(z_angular)*y) - (x - (Focus*tan(pre_y_angel + y_angular)/pixel_size)) + COL/2);
                        int compen_y = (int)((x*sin(z_angular) + cos(z_angular)*y) - (y - (Focus*tan(pre_x_angel - x_angular)/pixel_size)) + ROW/2);
                        event_buffer[i].x = compen_x;
                        event_buffer[i].y = compen_y;
                                
                        //count image and time image
                        if(compen_y < ROW && compen_y >= 0 && compen_x < COL && compen_x >= 0)
                        {
                            count_image[compen_y][compen_x]++;
                            normalized_mean_timestamp_image_.at<float>(compen_y, compen_x) +=  time_diff;
                        }
                    }
                    
                    int max_count = 0;
                    double mean_rel_ts = 0.0;
                    int count_rel_ts = 0;
                    for (int y = 0; y < normalized_mean_timestamp_image_.rows; ++y)
                    {
                        for (int x = 0; x < normalized_mean_timestamp_image_.cols; ++x)
                        {
                            if (count_image[y][x] != 0)
                            {
                                max_count = std::max(max_count,count_image[y][x]);
                                normalized_mean_timestamp_image_.at<float>(y, x) = normalized_mean_timestamp_image_.at<float>(y, x) / count_image[y][x];
                                mean_rel_ts += normalized_mean_timestamp_image_.at<float>(y, x);
                                count_rel_ts += 1;
                            }
                            else
                            {
                                normalized_mean_timestamp_image_.at<float>(y, x) = 0.;
                            }
                        }
                    }
                
                    if (count_rel_ts > 0)
                        mean_rel_ts /= (double)count_rel_ts;

                    normalized_mean_timestamp_image_ = (normalized_mean_timestamp_image_ - mean_rel_ts) / max_time_diff;              
                    cv::Mat thresholded_mean_timestamp_image_ = cv::Mat::zeros(ROW,COL, CV_64FC1);
                    cv::threshold(normalized_mean_timestamp_image_,
                        thresholded_mean_timestamp_image_, 0, 1,
                        cv::THRESH_TOZERO);
                    cv::blur(thresholded_mean_timestamp_image_, thresholded_mean_timestamp_image_, cv::Size(5,5));

                    double ev_stamp = (event_buffer[0].ts.toSec()+event_buffer[event_buffer.size() - 1].ts.toSec())/2;
                    
                    ev_map_mutex_.lock();
                    estimator.ev_stamps.push_back(ev_stamp);
                    estimator.ev_maps.push_back(thresholded_mean_timestamp_image_);
                    ev_map_mutex_.unlock();

                    event_buffer.clear();
                }
                else
                {
                    //release buffer
                    event_buffer.clear();
                }
            }
        }
    }
    std::cout<<"end data_process!"<<std::endl;
}

void exit_handler(int sig_num)
{
  printf("SIGNAL received: num =%d\n", sig_num);
  if (sig_num == 1 || sig_num == 2 || sig_num == 3 || sig_num == 9 ||
      sig_num == 15) {
    processing = false;
    exit(0);
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun EE-VINS vins_node [config file] \n"
               "for example: rosrun EE-VINS vins_node "
               "/home/.../config/viode/calibration_mono.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu, sub_event;
    if(USE_IMU)
    {
        sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    }
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    if (!USE_EVENT_TXT)
    {
        sub_event = n.subscribe(EVENT_TOPIC, 100, &event_callback);
    }
    ros::Subscriber sub_img1;
    if(STEREO)
    {
        sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    }
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);
    
    // install signal use sigaction
    struct sigaction sig_action;
    sigemptyset(&sig_action.sa_mask);
    sig_action.sa_flags = 0;
    sig_action.sa_handler = exit_handler;
    sigaction(SIGHUP, &sig_action, NULL);  // 1
    sigaction(SIGINT, &sig_action, NULL);  // 2
    sigaction(SIGQUIT, &sig_action, NULL); // 3
    sigaction(SIGKILL, &sig_action, NULL); // 9
    sigaction(SIGTERM, &sig_action, NULL); // 15

    if (USE_EVENT_TXT)
    {
        ros::AsyncSpinner spinner(3); // Use 3 threads
        processing = true;
        std::thread event_load(event_cb_txt, EVENT_TXT_PATH);
        event_load.detach();
        std::thread process(txt_format_data_process);
        process.detach();
        std::thread sync_thread{sync_process};
        ros::spin();
    }
    else
    {
        ros::AsyncSpinner spinner(3); // Use 3 threads
        processing = true;
        std::thread process(bag_format_data_process);
        process.detach();
        std::thread sync_thread{sync_process};
        ros::spin();
    }
    
    // ros::spin();

    return 0;
}
