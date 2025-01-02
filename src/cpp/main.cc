/*-------------------------------------------
                头文件引入
-------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "yolov5.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"

#if defined(RV1106_1103) 
    #include "dma_alloc.hpp"
#endif

#define MAX_ITEM 1

/*-------------------------------------------
    全局变量 & 同步
-------------------------------------------*/
pthread_mutex_t src_image_mutex = PTHREAD_MUTEX_INITIALIZER;
image_buffer_t src_image;
volatile bool g_camera_running = true;
cv::KalmanFilter KF;
bool kf_initialized = false;
int no_detection_count = 0;
float last_norm_x = 0.0f;
float last_norm_y = 0.0f;
const int MAX_NO_DETECTION = 100;

/*-------------------------------------------
    初始化卡尔曼滤波器
-------------------------------------------*/
void init_kalman_filter() {
    // 状态向量 [x, y, vx, vy]
    const int stateSize = 4;
    // 测量向量 [x, y]
    const int measSize = 2;
    // 控制向量大小
    const int contrSize = 0;

    KF.init(stateSize, measSize, contrSize);

    // 转移矩阵 A
    cv::setIdentity(KF.transitionMatrix);
    KF.transitionMatrix.at<float>(0,2) = 1.0f;  // dx = vx
    KF.transitionMatrix.at<float>(1,3) = 1.0f;  // dy = vy

    // 测量矩阵 H
    cv::Mat_<float> measurement(2,1);
    measurement.setTo(cv::Scalar(0));
    KF.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
    KF.measurementMatrix.at<float>(0,0) = 1.0f;
    KF.measurementMatrix.at<float>(1,1) = 1.0f;

    // 过程噪声协方差矩阵 Q
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-4));
    
    // 测量噪声协方差矩阵 R
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));

    // 后验错误估计协方差矩阵 P
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));
}

/*-------------------------------------------
    摄像头线程函数
-------------------------------------------*/
void* camera_thread_func(void* arg)
{
    // 初始化摄像头
    cv::VideoCapture cap;
    cv::Mat bgr(640, 480, CV_8UC3);
    
    // 设置摄像头参数
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.open(11);
    
    if (!cap.isOpened()) {
        printf("无法打开摄像头\n");
        g_camera_running = false;
        return NULL;
    }

    while (g_camera_running)
    {
        // 读取摄像头帧
        cap >> bgr;
        if (bgr.empty()) {
            printf("读取摄像头帧失败\n");
            continue;
        }

        // 转换为RGB格式
        cv::Mat rgb_frame;
        cv::cvtColor(bgr, rgb_frame, cv::COLOR_BGR2RGB);

        // 准备image buffer
        image_buffer_t temp_image;
        memset(&temp_image, 0, sizeof(temp_image));

        temp_image.width = rgb_frame.cols;
        temp_image.height = rgb_frame.rows;
        temp_image.format = IMAGE_FORMAT_RGB888;
        temp_image.size = rgb_frame.total() * rgb_frame.elemSize();

#if defined(RV1106_1103)
        int ret_dma = dma_buf_alloc(
            RV1106_CMA_HEAP_PATH,
            temp_image.size,
            &temp_image.fd,
            (void**)&temp_image.virt_addr
        );
        if (ret_dma != 0) {
            printf("DMA缓冲区分配失败！ret_dma=%d\n", ret_dma);
            continue;
        }
        memcpy(temp_image.virt_addr, rgb_frame.data, temp_image.size);
        dma_sync_cpu_to_device(temp_image.fd);
#else
        temp_image.virt_addr = (unsigned char*)malloc(temp_image.size);
        if (!temp_image.virt_addr) {
            printf("内存分配失败\n");
            continue;
        }
        memcpy(temp_image.virt_addr, rgb_frame.data, temp_image.size);
#endif

        // 更新全局图像
        pthread_mutex_lock(&src_image_mutex);
        if (src_image.virt_addr != NULL) {
#if defined(RV1106_1103)
            dma_buf_free(src_image.size, &src_image.fd, src_image.virt_addr);
#else
            free(src_image.virt_addr);
#endif
        }
        memcpy(&src_image, &temp_image, sizeof(image_buffer_t));
        pthread_mutex_unlock(&src_image_mutex);
    }

    cap.release();
    return NULL;
}

/*-------------------------------------------
    比较函数定义
-------------------------------------------*/
int compare_detect_results(const void *a, const void *b)
{
    const object_detect_result *res_a = (const object_detect_result *)a;
    const object_detect_result *res_b = (const object_detect_result *)b;
    
    int area_a = (res_a->box.right - res_a->box.left) * (res_a->box.bottom - res_a->box.top);
    int area_b = (res_b->box.right - res_b->box.left) * (res_b->box.bottom - res_b->box.top);
    
    if (area_a > area_b) return -1;
    if (area_a < area_b) return 1;
    return 0;
}

/*-------------------------------------------
                主函数
-------------------------------------------*/
int main(int argc, char **argv)
{
    printf("启动人体检测程序...\n");
    
    const char *model_path = "./model/tracker.rknn";
    int ret;

    system("RkLunch-stop.sh");  // 停止RockChip的启动服务，以便使用摄像头

    // 初始化卡尔曼滤波器
    init_kalman_filter();

    // 创建摄像头线程
    pthread_t camera_thread;
    int thread_ret = pthread_create(&camera_thread, NULL, camera_thread_func, NULL);
    if (thread_ret != 0) {
        printf("创建摄像头线程失败！返回值=%d\n", thread_ret);
        return -1;
    }

    // 等待摄像头初始化
    sleep(1);
    if (!g_camera_running) {
        printf("摄像头初始化失败！\n");
        return -1;
    }

    // 初始化所有变量
    rknn_app_context_t rknn_app_ctx;
    cv::Mat_<float> measurement(2,1);
    
    // 清空内存和初始化
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    memset(&src_image, 0, sizeof(image_buffer_t));
    measurement.setTo(cv::Scalar(0));
    
    init_post_process();

    printf("加载YOLOv5模型: %s\n", model_path);
    ret = init_yolov5_model(model_path, &rknn_app_ctx);
    if (ret != 0) {
        printf("加载YOLOv5模型失败！返回值=%d 模型路径=%s\n", ret, model_path);
        goto out;
    }
    printf("模型加载成功\n");

    printf("开始主循环处理...\n");
    // 主循环
    while (g_camera_running)
    {
        pthread_mutex_lock(&src_image_mutex);
        if (src_image.virt_addr == NULL) {
            pthread_mutex_unlock(&src_image_mutex);
            continue;
        }

        image_buffer_t current_image;
        memcpy(&current_image, &src_image, sizeof(image_buffer_t));
        pthread_mutex_unlock(&src_image_mutex);

        object_detect_result_list od_results;
        ret = inference_yolov5_model(&rknn_app_ctx, &current_image, &od_results);
        if (ret != 0) {
            continue;
        }

        // 过滤和排序person检测结果
        if (od_results.count > 0)
        {
            object_detect_result_list filtered_results;
            filtered_results.count = 0;

            // 只保留person类别的检测结果
            for (int i = 0; i < od_results.count; i++)
            {
                const char *cls_name = coco_cls_to_name(od_results.results[i].cls_id);
                if (strcmp(cls_name, "person") == 0)
                {
                    filtered_results.results[filtered_results.count++] = od_results.results[i];
                }
            }

            // 按检测框面积排序
            if (filtered_results.count > 0)
            {
                qsort(filtered_results.results, filtered_results.count, 
                      sizeof(object_detect_result), compare_detect_results);
                
                if (filtered_results.count > MAX_ITEM) {
                    filtered_results.count = MAX_ITEM;
                }
            }

            od_results = filtered_results;
        }

        // 处理检测结果
        if (od_results.count == 0) {
            no_detection_count++;
            
            // 更新卡尔曼滤波器
            if (no_detection_count > MAX_NO_DETECTION) {
                // 超过100次无检测，使用0,0
                measurement.at<float>(0) = 0.0f;
                measurement.at<float>(1) = 0.0f;
            } else {
                // 使用最后一次的有效坐标
                measurement.at<float>(0) = last_norm_x;
                measurement.at<float>(1) = last_norm_y;
            }

            if (!kf_initialized) {
                // 第一次运行，初始化卡尔曼滤波器状态
                KF.statePost.at<float>(0) = measurement.at<float>(0);
                KF.statePost.at<float>(1) = measurement.at<float>(1);
                KF.statePost.at<float>(2) = 0.0f;
                KF.statePost.at<float>(3) = 0.0f;
                kf_initialized = true;
            }

            // 预测和更新
            cv::Mat prediction = KF.predict();
            cv::Mat estimated = KF.correct(measurement);

            // 获取滤波后的位置
            float filtered_x = estimated.at<float>(0);
            float filtered_y = estimated.at<float>(1);
            float velocity_x = estimated.at<float>(2);
            float velocity_y = estimated.at<float>(3);

            printf("no detection[%d]: raw=(%.3f,%.3f) filtered=(%.3f,%.3f) vel=(%.3f,%.3f)\n",
                no_detection_count,
                measurement.at<float>(0), measurement.at<float>(1),
                filtered_x, filtered_y,
                velocity_x, velocity_y);
        }

        for (int i = 0; i < od_results.count; i++)
        {
            object_detect_result *det_result = &(od_results.results[i]);
            no_detection_count = 0;  // 重置无检测计数器
            
            // 计算检测框中心点
            float center_x = (det_result->box.left + det_result->box.right) / 2.0f;
            float center_y = (det_result->box.top + det_result->box.bottom) / 2.0f;
            
            // 转换到[-1, 1]范围
            float norm_x = (center_x - current_image.width/2.0f) / (current_image.width/2.0f);
            float norm_y = -((center_y - current_image.height/2.0f) / (current_image.height/2.0f));

            // 保存最后的有效坐标
            last_norm_x = norm_x;
            last_norm_y = norm_y;

            // 更新卡尔曼滤波器
            measurement.at<float>(0) = norm_x;
            measurement.at<float>(1) = norm_y;

            if (!kf_initialized) {
                // 第一次运行，初始化卡尔曼滤波器状态
                KF.statePost.at<float>(0) = norm_x;
                KF.statePost.at<float>(1) = norm_y;
                KF.statePost.at<float>(2) = 0;  // 初始速度为0
                KF.statePost.at<float>(3) = 0;
                kf_initialized = true;
            }

            // 预测
            cv::Mat prediction = KF.predict();
            // 更新
            cv::Mat estimated = KF.correct(measurement);

            // 获取滤波后的位置
            float filtered_x = estimated.at<float>(0);
            float filtered_y = estimated.at<float>(1);
            float velocity_x = estimated.at<float>(2);
            float velocity_y = estimated.at<float>(3);

            printf("person %d: box=(%d,%d,%d,%d) raw=(%.3f,%.3f) filtered=(%.3f,%.3f) vel=(%.3f,%.3f) conf=%.3f\n",
                i,
                det_result->box.left, det_result->box.top,
                det_result->box.right, det_result->box.bottom,
                norm_x, norm_y,
                filtered_x, filtered_y,
                velocity_x, velocity_y,
                det_result->prop);
        }


    }

out:
    printf("清理资源...\n");
    g_camera_running = false;
    deinit_post_process();

    ret = release_yolov5_model(&rknn_app_ctx);
    if (ret != 0) {
        printf("释放YOLOv5模型资源失败！返回值=%d\n", ret);
    }

    if (src_image.virt_addr != NULL) {
#if defined(RV1106_1103)
        dma_buf_free(src_image.size, &src_image.fd, src_image.virt_addr);
#else
        free(src_image.virt_addr);
#endif
    }

    pthread_join(camera_thread, NULL);
    printf("程序结束\n");
    return 0;
}