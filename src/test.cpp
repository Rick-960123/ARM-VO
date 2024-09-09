#include <iostream>
#include <string>
#include <time.h>
#include <vector>
#include <fstream>
#include <chrono>
#include "ARM_VO.hpp"

using namespace std;

// #define SHOW_BY_OPENCV
#ifdef SHOW_BY_OPENCV
#include <opencv2/opencv.hpp>
using namespace cv;
#endif

#undef ALOGI
#undef ALOGE
#define ALOGI(format, ...) printf(format, ##__VA_ARGS__); printf("\n");
#define ALOGE(format, ...) printf(format, ##__VA_ARGS__); printf("\n");

#pragma pack(push)
#pragma pack(1)
typedef struct {
    short droneState; // value is enum VDroneState
	float RX;
	float RZ;
	float RY;
	short PSI;
	short TETA;
	short GAMMA;
	short VRX;
	short VRY;
	short VRZ;
	short GimbolPitch;
	short GimbolRoll;
	short GimbolYaw;
	uint8_t StateFlag;
    unsigned int timestamp;
} FcData_t;
#pragma pack(pop)

int main()
{
    // open bin file
    // string basePath = "/userdata/gpw/";
    string basePath = "/home/rick/SN_00198/";
    string videoPath = basePath + "vcam_down_198_20000102-080013.bin";
    string fcDataPath = basePath + "fc_data_198_20170804-170010.bin";
    std::ifstream videoStream(videoPath, ios::binary | ios::in);
    std::ifstream fcDataStream(fcDataPath, ios::binary | ios::in);
    if (!videoStream.is_open() || !fcDataStream.is_open()) {
        ALOGE("open video or fc data bin file fail!");
        return -1;
    }

    // create vision alg ptr
    // VisionAlgImp::Ptr visionAlgPtr(new VisionAlgImp(basePath));
    // visionAlgPtr->setMissionSn(160);
    // visionAlgPtr->setDebugRecordOn(false);
    string paramsFileName = basePath + "camera_info.yaml";
    ARM_VO VO(paramsFileName);
    Viewer Results;

    uint8_t *data = (uint8_t *)malloc(1920*1080);

    auto start = std::chrono::high_resolution_clock::now();
    int numFramesProcessed = 0;
    double first_stamp = -1;
    double end_stamp = -1;
    int width = 640, height = 480;
    unsigned int FPS, sum_fps = 0;
    unsigned int imageCounter = 0;

    while (true) {
        // read video data
        auto read_data = std::chrono::high_resolution_clock::now();
        uint32_t vStamp = 0, size = 0;
        if (!videoStream.read((char *)&vStamp, sizeof(uint32_t)).good()) break;
        if (!videoStream.read((char *)&size, sizeof(uint32_t)).good()) break;
        ALOGI("stamp %d size %d", vStamp, size);
        if (!videoStream.read((char *)data, size).good()) break;
        // update frame
        if (vStamp < 25322) continue;

        // visionAlgPtr->updateDownFrame(data, size, vStamp);
        cv::Mat curr_frame = cv::Mat(height, width, CV_8UC1, data, width).clone();

        if (!VO.initialized)
        {
            VO.init(curr_frame);
        }
        else
        {
            clock_t start = clock();

            VO.update(curr_frame);

            clock_t finish = clock();
            FPS = 1000 / (1000*(finish-start)/CLOCKS_PER_SEC);
            sum_fps+=FPS;

            Results.show(curr_frame, VO.prev_inliers, VO.curr_inliers, FPS, VO.t_f);
        }

        auto updateDownFrame = std::chrono::high_resolution_clock::now();
        double MilliSecondsTakenSingle = 1.0f * std::chrono::duration_cast<std::chrono::milliseconds>(updateDownFrame - read_data).count();
        printf("updateDownFrame: %f ms\n ",  MilliSecondsTakenSingle);
        
        numFramesProcessed++;
        if (first_stamp < 0){
            first_stamp = static_cast<double>(vStamp);
        }
        end_stamp = static_cast<double>(vStamp);
        // read fc data
        // uint32_t fcTime = 0;
        // bool fcEOF = false;
        // do {
        //     FcData_t fcData;
        //     if (!fcDataStream.read((char *)&fcData, sizeof(FcData_t)).good()) {
        //         fcEOF = true;
        //         break;
        //     }
        //     visionAlgPtr->updateFcData((uint8_t *)&fcData, sizeof(FcData_t), fcTime);
        // } while(fcTime < vStamp);
        // if (fcEOF) break;

#ifdef SHOW_BY_OPENCV
        // show frame
        int width = 640, height = 480;
        Mat frame = Mat(height, width, CV_8UC1, data, width);
        cv::imshow("VisionLanding", frame);
        char key = (char)waitKey(50);
#endif

    }

        auto end = std::chrono::high_resolution_clock::now();
        double numSecondsProcessed = (end_stamp - first_stamp) * 1e-3;
        double MilliSecondsTakenSingle = 1.0f * std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        double MilliSecondsTakenMT = 0.0;

        printf("\n======================"
               "\n%d Frames (%.1f fps)"
               "\n%.2fms per frame (single core); "
               "\n%.2fms per frame (multi core); "
               "\n%.3fx (single core); "
               "\n%.3fx (multi core); "
               "\n======================\n\n",
               numFramesProcessed, numFramesProcessed / numSecondsProcessed,
               MilliSecondsTakenSingle / numFramesProcessed,
               MilliSecondsTakenMT / (float) numFramesProcessed,
               1000 / (MilliSecondsTakenSingle / numSecondsProcessed),
               1000 / (MilliSecondsTakenMT / numSecondsProcessed));
        
        printf("\n======================"
               "\ndata duration: %.2f s ; "
               "\nprocess duration: %.2f s; "
               "\n======================\n\n",
               numSecondsProcessed,
               MilliSecondsTakenSingle / 1000.0f);

    free(data);
}
