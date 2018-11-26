// Copyright (c) 2018 JachinShen(jachinshen@foxmail.com)
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

// Detect Armor in RoboMaster 2018

// Precompile paramaters
#include "armor.h"
#include "bayer_hack.h"
#include "platform.h"
// wrapper for Global Shutter Camera
#include "camera/camera_wrapper.h"

#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using std::cout;
using std::endl;
using std::fstream;
using std::ios;
using std::string;

#define FROM_FILE 0
#define FROM_CAMERA 1

#define OPENMP_STOP 0
#define OPENMP_RUN 1

#define RECORD_OFF 0
#define RECORD_ON 1

#ifdef _DEBUG
#define VIDEO_SOURCE FROM_FILE
#define DRAW_LEVEL SHOW_ALL
#define OPENMP_SWITCH OPENMP_STOP
#else
#define VIDEO_SOURCE FROM_CAMERA
#define DRAW_LEVEL SHOW_NO
#define OPENMP_SWITCH OPENMP_RUN
#endif

#define RECORD RECORD_OFF

string getRecordFileName();
double tic();
void printFps();

int main(void) {
    // The main while loop should only run once if everything is okay.
    // If something wrong happens(like the global shutter camera issues), it will
    // retry. Add to confirm it working.
    while (1) {
#if VIDEO_SOURCE == FROM_CAMERA
        cout << "Fetch Image from Camera!" << endl;
        GlobalShutterCamera video;
#elif VIDEO_SOURCE == FROM_FILE
        cout << "Fetch Image from File!" << endl;
        VideoCapture video;
#endif

#if RECORD == RECORD_ON
        string filename = getRecordFileName();
        cout << "Record File Name: " << file_name << endl;
        VideoWriter g_writer;
        // PIM1 format occupys little space.
        // 120fps, 640x480 resolution, based on the camera.
        // is_color=false because of the gray camera.
        g_writer.open(file_name, CV_FOURCC('P', 'I', 'M', '1'), 120,
                      cv::Size(640, 480), false);
#endif
        // Read video
#if VIDEO_SOURCE == FROM_CAMERA
        if (video.init() == 0) {
            cout << "Init Global Shutter Camera successfully!" << endl;
        } else {
            cout << "Init Global Shutter Camera Failed!" << endl;
            // If failed, retry.
            continue;
        }
#elif VIDEO_SOURCE == FROM_FILE
        video.open("/home/jachinshen/Videos/Robo/station-infanity/lefttoright.avi");
        if (video.isOpened())
            cout << "Open Video Successfully!" << endl;
        else {
            cout << "Open Video failed!" << endl;
            continue;
        }
#endif

        Armor armor;
        armor.init();

        // use 2 frames for parallel process
        // when loading picture from camera to frame1, process frame2
#if OPENMP_SWITCH == OPENMP_RUN
        Mat frame1, frame2;
        bool ok = true;

        // Sometimes the begining several images are full of noise.
        // It may take long time to process them, so skip them.
        for (int i = 0; i < 10; ++i) video.read(frame2);

        while (ok) {
            printFps();
            #pragma omp parallel sections
            {
                #pragma omp section
                { video.read(frame1); }
                #pragma omp section
                {
                    if (armor.run(frame2) < 0) {
                        cout << "Processing Error!" << endl;
                        ok = false;
                    }
                }
            }
            // wait for both section completed
            #pragma omp barrier
            printFps();
            #pragma omp parallel sections
            {
                #pragma omp section
                { video.read(frame2); }
                #pragma omp section
                {
                    if (armor.run(frame1) < 0) {
                        cout << "Processing Error!" << endl;
                        ok = false;
                    }
                }
            }
            #pragma omp barrier
        }
#elif OPENMP_SWITCH == OPENMP_STOP
        Mat frame;
        for (int i = 0; i < 10; ++i) video.read(frame);
        while (video.read(frame)) {
            printFps();
            imshow("color", frame);
#if RECORD == RECORD_ON
            g_writer.write(frame);
#endif
            // Record before process to prevent modification.
            armor.run(frame);
            cv::waitKey(1);
        }
        cout << "End!" << endl;
#endif
    }
}

string getRecordFileName() {
    fstream video_file;
    string video_id_str;
    int video_id;
    string file_name;

    // Read record file name.
    // Add one and write back to promise no same file name.
    // Otherwise the newer one will cover older one.
    video_file.open("../record_file_name.txt", ios::in);
    video_file >> video_id_str;
    video_file.close();

    video_file.open("../record_file_name.txt", ios::out);
    video_id = atoi(video_id_str.c_str()) + 1;//atoi is to change string into int 
    video_file << video_id;
    video_file.close();

    file_name = string(getpwuid(getuid())->pw_dir) + "/Videos/Record" +
                video_id_str + ".avi";//如果知道一个用户的用户ID或者登录名,可以通过getpwuid或getpwnam函数获得用户的登录信息
    return file_name;
}

double tic() {
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((float)t.tv_sec + ((float)t.tv_usec) / 1000000.);
}

void printFps() {
    static double running_time = tic();
    double fps = 1 / (tic() - running_time);
    cout << "fps: " << fps << endl;
    running_time = tic();
}
