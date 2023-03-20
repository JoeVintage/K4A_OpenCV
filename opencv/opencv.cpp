#include <stdio.h>
#include <stdlib.h>

#include <k4a/k4a.h>
#include <k4abt.h>
#include <iostream>
#include <fstream>
using namespace std;
#include <map>
#include <opencv2/opencv.hpp>

#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    } 


int getRealTimeImageStream() {
   
    /* getRealTimeImageStream();*/

  // 初始化 Azure Kinect DK
    k4a_device_t device = NULL;
    k4a_device_configuration_t camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    camera_config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    camera_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED; // No need for depth during calibration
    camera_config.camera_fps = K4A_FRAMES_PER_SECOND_15;     // Don't use all USB bandwidth
    camera_config.subordinate_delay_off_master_usec = 0;     // Must be zero for master
    camera_config.synchronized_images_only = true;

    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    VERIFY(k4a_device_start_cameras(device, &camera_config), "Start K4A cameras failed!");

    k4a_calibration_t sensor_calibration;
    VERIFY(k4a_device_get_calibration(device, camera_config.depth_mode, camera_config.color_resolution, &sensor_calibration),
        "Get depth camera calibration failed!");

    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");


    // 准备捕获器
  

    // 循环读取视频帧
    while (true)
    {
        k4a_capture_t capture = NULL;
        // 等待 Azure Kinect DK 准备就绪
        k4a_wait_result_t wait_result = k4a_device_get_capture(device, &capture, K4A_WAIT_INFINITE);
        if (wait_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // 获取颜色帧
                  // 获取颜色帧
            k4a_image_t color_image = k4a_capture_get_color_image(capture);
            
            // 将颜色帧转换为 OpenCV 格式
            cv::Mat color_mat(k4a_image_get_height_pixels(color_image), k4a_image_get_width_pixels(color_image),
                CV_8UC4, k4a_image_get_buffer(color_image));
        
            

            // 处理颜色帧
            // ...
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);
            if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                // It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Add capture to tracker process queue timeout!\n");
                break;
            }
            else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
            {
                printf("Error! Add capture to tracker process queue failed!\n");
                break;
            }

            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                // Successfully popped the body tracking result. Start your processing


                const int num_bodies = k4abt_frame_get_num_bodies(body_frame);
                k4abt_joint_t* joints = new k4abt_joint_t[num_bodies];
                for (size_t i = 0; i < num_bodies; i++)
                {
                    k4abt_skeleton_t skeleton;
                    k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
                    uint32_t id = k4abt_frame_get_body_id(body_frame, i);
                    //3代表head
                    k4abt_joint_t nose = skeleton.joints[K4ABT_JOINT_NOSE];
                    k4abt_joint_t head = skeleton.joints[K4ABT_JOINT_HEAD];
                    k4abt_joint_t eye_left = skeleton.joints[K4ABT_JOINT_EYE_LEFT];
                    k4abt_joint_t eye_right = skeleton.joints[K4ABT_JOINT_EYE_RIGHT];
                    
                  /*  k4a_calibration_2d_to_2d();
                    k4a_transformation_depth_image_to_color_camera();*/
           
                    k4a_float2_t nosePoint2d;
                    int noseVaild;
                    k4a_calibration_3d_to_2d(&sensor_calibration, &(nose.position), K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &nosePoint2d, &noseVaild);

                    k4a_float2_t eyeLeftPoint2d;
                    int eyeLeftVaild;
                    k4a_calibration_3d_to_2d(&sensor_calibration, &(eye_left.position), K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &eyeLeftPoint2d, &eyeLeftVaild);

                    k4a_float2_t eyeRightPoint2d;
                    int eyeRightVaild;
                    k4a_calibration_3d_to_2d(&sensor_calibration, &(eye_right.position), K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &eyeRightPoint2d, &eyeRightVaild);

                    float scale = nose.position.xyz.z / 650.0;
              
                    printf("Head Position is w:%f x：%f y:%f z:%f\n", head.orientation.wxyz.w, head.orientation.wxyz.x, head.orientation.wxyz.y, head.orientation.wxyz.z);
                    float faceSizeWidth = 200.0 / scale;
                    float faceSizeHeight = 300.0 / scale;
                    float eyeWidth = 100.0 / scale;
                    float eyeHeight = 50.0 / scale;
                    float headPositionX = abs(head.orientation.wxyz.x);
                    float headPositionY = abs(head.orientation.wxyz.y);
                
                    if (headPositionX > 0.3 && headPositionX < 0.7 && headPositionY > 0.3 && headPositionY < 0.7) {
                        float headOffsetX = 0;
                         headOffsetX = (headPositionX - 0.5) * faceSizeWidth *2;
                         faceSizeWidth = faceSizeWidth + abs(headOffsetX/4);
                
                        cv::Rect noseRect(nosePoint2d.xy.x  - faceSizeWidth / 2, nosePoint2d.xy.y - faceSizeHeight / 2, faceSizeWidth, faceSizeHeight);
                        cv::Rect leftEyeRect(eyeLeftPoint2d.xy.x - eyeWidth / 2, eyeLeftPoint2d.xy.y - eyeHeight, eyeWidth, eyeHeight);
                        cv::Rect rightEyeRect(eyeRightPoint2d.xy.x - eyeWidth / 2, eyeRightPoint2d.xy.y - eyeHeight, eyeWidth, eyeHeight);

                        cv::ellipse(color_mat, cv::Point(nosePoint2d.xy.x + headOffsetX, nosePoint2d.xy.y - faceSizeHeight * 0.2), cv::Size(faceSizeWidth, faceSizeHeight), 0, 0, 360, cv::Scalar(0, 0, 255), -1);
                        cv::ellipse(color_mat, cv::Point(eyeLeftPoint2d.xy.x, eyeLeftPoint2d.xy.y - eyeHeight / 2), cv::Size(eyeWidth, eyeHeight), 0, 0, 360, cv::Scalar(0, 0, 0), -1);
                        cv::ellipse(color_mat, cv::Point(eyeRightPoint2d.xy.x, eyeRightPoint2d.xy.y - eyeHeight / 2), cv::Size(eyeWidth, eyeHeight), 0, 0, 360, cv::Scalar(0, 0, 0), -1);
                  
                    }
               
                }

                printf("%zu bodies are detected!\n", num_bodies);
            }
            else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Pop body frame result timeout!\n");
                break;
            }
            else
            {
                printf("Pop body frame result failed!\n");
                break;
            }
            cv::imshow("Color Image", color_mat);
            cv::waitKey(1);
            // 释放帧和图像
            k4abt_frame_release(body_frame);
            k4a_image_release(color_image);
            k4a_capture_release(capture);
        }
        else
        {
            // 处理等待失败的情况
        }
    }

    // 关闭 Azure Kinect DK
    k4a_device_close(device);

}




int main()
{
    getRealTimeImageStream();


}