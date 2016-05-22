/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <ctime>
#include <cmath>
#include <vector>
#include <cassert>
#include <unistd.h>
#include <string>
#include <opencv2/opencv.hpp>

#include "mavlink_control.h"
#include "input_params.h"
#include <iostream>
// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
using namespace cv;

Mat preErode = getStructuringElement(MORPH_ELLIPSE,
        Size(2 * 1 + 1, 2 * 1 + 1),
        Point(1, 1));
// <TODO: remove this from being global later>
input_params inputs = input_params();

int top(int argc, char **argv) {

    //    int ndx(1), nfound(0);
    //    std::string result, jpg, solution1;
    //    double tim;
    //    clock_t ti, tf;
    //    Mat frame, hsv, upL, downL, redImg, dilateF, erodeF, endF;
    //    int dilation_size = 15, erode_size = 2;
    //
    //    std::ofstream timer;
    //    timer.open("/home/pi/NGCP/ballFrames/solution1/Solution1TimeProfile", std::ofstream::out | std::ofstream::trunc);
    //    timer << "Frame, Time [s]\n";
    //
    //    //store path names
    //    solution1 = "/home/pi/NGCP/ballFrames/solution1/frame";
    //    jpg = ".jpg";

    //    VideoCapture cap("/home/pi/NGCP/ballFrames/ball_vid.avi");
    //    while (true) {
    //        cap >> frame;
    //
    //        if (!frame.data) {
    //            std::cout << "Error reading ball frame" << ndx << std::endl;
    //            std::cout << "Done and found " << nfound << "/" << ndx << " circles" << std::endl;
    //            return -1;
    //        }
    //        ndx++;
    //        //begin timer;
    //        ti = clock();
    //
    //        //hsv
    //        cvtColor(frame, hsv, CV_BGR2HSV);
    //
    //        //filter 
    //        inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), downL);
    //        inRange(hsv, Scalar(160, 100, 100), Scalar(179, 255, 255), upL);
    //        addWeighted(downL, 1.0, upL, 1.0, 0.0, redImg);
    //
    //        //erode
    //        erode(redImg, redImg, preErode);
    //
    //        //calculate mean or magnitude and only continue if greater than threshold
    //        m = mean(redImg);
    //
    //        if (norm(m) > 0) {
    //            //dilate
    //            dilate(redImg, dilateF, dilateE);
    //            //erode
    //            erode(dilateF, erodeF, erodeE);
    //            //hough
    //            HoughCircles(erodeF, circles, CV_HOUGH_GRADIENT, 2, erodeF.rows / 2, 100, 50);
    //        }
    //        tf = clock();
    //        tim = ((double) tf - (double) ti) / (double) CLOCKS_PER_SEC;
    //        timer << "Frame" << ndx << ", " << tim << std::endl;
    //
    //        if (circles.size() > 0) {
    //            nfound++;
    //            endF = frame.clone();
    //            // draw circle
    //            for (size_t i = 0; i < circles.size(); i++) {
    //                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    //                int radius = cvRound(circles[i][2]);
    //                // circle center
    //                circle(endF, center, 3, Scalar(0, 255, 0), -1, 8, 0);
    //                // circle outline
    //                circle(endF, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    //            }
    //            // save images
    //            result = solution1 + std::to_string(ndx) + jpg;
    //            imwrite(result, endF);
    //            std::cout << "Ball found in frame" << ndx << std::endl;
    //            circles.clear();
    //        }
    //    }

    // --------------------------------------------------------------------------
    //   PARSE THE COMMANDS
    // --------------------------------------------------------------------------

    // Default input arguments
#ifdef __APPLE__
    char *uart_name = (char*) "/dev/tty.usbmodem1";
#else
    char *uart_name = (char*) "/dev/ttyAMA0";
#endif
    int baudrate = inputs.getBaud();
    float D = inputs.getDiagonal(); //[m] Search box defaults to 50 m diagonal length
    int videoN(inputs.getVideo_ID()); //set video device number
    // <TODO: Add command line argument to set video device number (int)>
    // <TODO: Add command line argument to set video resolution (test to determine run times with typical resolutions)>
    // do the parse, will throw an int if it fails
    // should throw error if size of search box is not specified
    parse_commandline(argc, argv, uart_name, baudrate, D);

    // --------------------------------------------------------------------------
    //   PORT and THREAD STARTUP
    // --------------------------------------------------------------------------

    /*
     * Instantiate a serial port object
     *
     * This object handles the opening and closing of the offboard computer's
     * serial port over which it will communicate to an autopilot.  It has
     * methods to read and write a mavlink_message_t object.  To help with read
     * and write in the context of pthreading, it gaurds port operations with a
     * pthread mutex lock.
     *
     */



    // <Will store member data for uart_name and baudrate>
    Serial_Port serial_port(uart_name, baudrate);

    /*
     * Instantiate an autopilot interface object
     *
     * This starts two threads for read and write over MAVlink. The read thread
     * listens for any MAVlink message and pushes it to the current_messages
     * attribute.  The write thread at the moment only streams a position target
     * in the local NED frame (mavlink_set_position_target_local_ned_t), which
     * is changed by using the method update_setpoint().  Sending these messages
     * are only half the requirement to get response from the autopilot, a signal
     * to enter "offboard_control" mode is sent by using the enable_offboard_control()
     * method.  Signal the exit of this mode with disable_offboard_control().  It's
     * important that one way or another this program signals offboard mode exit,
     * otherwise the vehicle will go into failsafe.
     *
     */
    // <Instantiates object and initializes values to 0>
    sleep(5);
    Autopilot_Interface autopilot_interface(&serial_port);

    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    serial_port_quit = &serial_port;
    autopilot_interface_quit = &autopilot_interface;
    signal(SIGINT, quit_handler);

    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */
    // <Opens serial port on pc and sets up port with baud rate and 8N1>
    // Will print OPEN PORT
    // Will print Connected to %s with %d baud ...
    serial_port.start();

    // <Starts a read and write thread from the pixhawk if connected>
    //1. Will first check pc serial port is open
    //2. Start a read thread and print: CHECK FOR MESSAGES while waiting for reponse from Pixhawk
    //3. Will print: FOUND as well as the system and autopilot ID
    //4. Stores initial position and attiutude information for the pixhawk (must be done before writing)
    //5. Starts a write thread and waits until the thread is started. Will return when thread is created
    autopilot_interface.start();

    // Generate array of setpoints
    std::vector<float> xSetPoints;
    std::vector<float> ySetPoints;

    genSetPoints(D, autopilot_interface, xSetPoints, ySetPoints);

    // <TODO: Throw this into a function>
    // Instantiate VideoCapture object
    VideoCapture cam(videoN); //open video1
    VideoWriter video("/home/pi/NGCP/RPI_cpslo/Datalogs/ball_vid.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(inputs.getVideo_Width(), inputs.getVideo_Height()), true);
    sleep(1); //sleep for a second

    //set camera resolution
    cam.set(CV_CAP_PROP_FRAME_WIDTH, inputs.getVideo_Width());
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, inputs.getVideo_Height());

    //error checking for camera
    if (!cam.isOpened()) {
        std::cout << "Camera device " << videoN << "could not be opened. Exiting"
                << std::endl;
        return -1;
    }

    // --------------------------------------------------------------------------
    //   RUN COMMANDS
    // --------------------------------------------------------------------------

    /*
     * Now we can implement the algorithm we want on top of the autopilot interface
     */
    // <Modify the commands function below>
    commands(autopilot_interface, xSetPoints, ySetPoints, cam, video);

    while (true) {
        std::cout << "Exited commands" << std::endl;
        sleep(1);
        //continue looping so setpoint is still over the ball
    };
    // --------------------------------------------------------------------------
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------

    /*
     * Now that we are done we can stop the threads and close the port
     */
    autopilot_interface.stop();
    serial_port.stop();

    // --------------------------------------------------------------------------
    //   DONE
    // --------------------------------------------------------------------------

    // woot!
    return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void commands(Autopilot_Interface &api,
        const std::vector<float> &xSetPoints, const std::vector<float> &ySetPoints,
        VideoCapture &cam, VideoWriter &video) {

    // <NOTE: LOCAL AXIS SYSTEM IS NED (NORTH EAST DOWN) SO POSITIVE Z SETPOINT IS LOSS IN ALTITUDE>
    float setAlt = inputs.getAltitude(); //[m] set set point altitude above initial altitude
    float setTolerance = inputs.getTolerance(); //tolerance for set point (how close before its cleared)
    uint8_t ndx(0);
    std::vector<Vec3f> circles; //vector for circles found by OpenCV
    usleep(100); // give some time to let it sink in

    bool setPointReached = false, ballFound = false;

    //uncomment to allow logging to SD card
    std::ofstream Local_Pos; //#32, 85 (target)
    std::ofstream Global_Pos; //#33, 87 (target)
    std::ofstream Attitude; //#30
    std::ofstream HR_IMU; //#105 HIGHRES_IMU

    // initialize command data structures
    mavlink_set_position_target_local_ned_t sp;
    mavlink_set_position_target_local_ned_t ip = api.initial_position;

    //open log files and append header line
    genDatalogs(Local_Pos, Global_Pos, Attitude, HR_IMU, api, 30);
    genDatalogs(Local_Pos, Global_Pos, Attitude, HR_IMU, api, 20);
    genDatalogs(Local_Pos, Global_Pos, Attitude, HR_IMU, api, 2);

    assert(xSetPoints.size() == ySetPoints.size());

    for (ndx = 0; ndx < xSetPoints.size(); ndx++) {
        // <TODO: Fix the mix up with x,y and N,E>
        set_position(ySetPoints[ndx], xSetPoints[ndx], ip.z - setAlt, sp);
        api.update_setpoint(sp);
        setPointReached = false;

        genDatalogs(Local_Pos, Global_Pos, Attitude, HR_IMU, api, 1);
        //loop until set point is reached
        while (!setPointReached) {
            mavlink_local_position_ned_t lpos = api.current_messages.local_position_ned;

            ballFound = checkFrame(cam, video, circles);

            // if ball is found, update setpoint to current position and return
            if (ballFound) {
                cam.release();
                set_position(lpos.x, lpos.y, lpos.z, sp);
                api.update_setpoint(sp);
                genDatalogs(Local_Pos, Global_Pos, Attitude, HR_IMU, api, 11);
                genDatalogs(Local_Pos, Global_Pos, Attitude, HR_IMU, api, 1);
//                return;
            }

            // <TODO: Implement position check to occur less often (lower priority) >
            if (abs(lpos.x - sp.x) < setTolerance && abs(lpos.y - sp.y) < setTolerance && abs(lpos.z - sp.z) < setTolerance) {
                setPointReached = true;
                break;
            }

            genDatalogs(Local_Pos, Global_Pos, Attitude, HR_IMU, api, 1);
        }
        genDatalogs(Local_Pos, Global_Pos, Attitude, HR_IMU, api, 10);
        genDatalogs(Local_Pos, Global_Pos, Attitude, HR_IMU, api, 1);
        sleep(1); //wait a second for vehicle to catch up
    }

    // <Ends video capture if end of set points is reached>
    cam.release();
    video.release();

    //set final yaw to 0 radians (expected to be pointing north)
    set_yaw(0.0, sp);
    api.update_setpoint(sp);

    genDatalogs(Local_Pos, Global_Pos, Attitude, HR_IMU, api, 20);
    genDatalogs(Local_Pos, Global_Pos, Attitude, HR_IMU, api, 31);
    return;
}

//Function to generate array of setpoints give a search box diagonal distance

bool checkFrame(VideoCapture &cam, VideoWriter &video, std::vector<Vec3f> &circles) {
    Mat frame; //Mat to store current frame from camera
    Mat hsv; //Mat to store transformed HSV space image
    Mat upLim; //Mat to store HSV image with upper limit applied
    Mat downLim; //Mat to store HSV image with lower limit applied
    Mat redImg; //Mat to store HSV image with combined upper and lower limits
    Scalar val; //Scalar to store value for mean of an Mat img
    static int nfound(1);
    std::string ogPath, finalPath, originalImg, finalImg;
    ogPath = "/home/pi/NGCP/RPI_cpslo/Datalogs/OriginalImg";
    finalPath = "/home/pi/NGCP/RPI_cpslo/Datalogs/FinalImg";

    //capture frame
    cam >> frame;
    //    resize(frame, frame, Size(640, 360), 0, 0, INTER_CUBIC);
    video.write(frame);

    //convert to HSV space
    cvtColor(frame, hsv, CV_BGR2HSV);

    // <TODO: remove hard coded limits>
    inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), downLim);
    inRange(hsv, Scalar(160, 100, 100), Scalar(179, 255, 255), upLim);

    //combine two ranges into single image
    addWeighted(downLim, 1.0, upLim, 1.0, 0.0, redImg);

    //erode to remove noise
    erode(redImg, redImg, preErode);

    if (norm(mean(redImg)) > 0) {
        originalImg = ogPath + std::to_string(nfound) + ".jpg";
        finalImg = finalPath + std::to_string(nfound) + ".jpg";
        imwrite(originalImg, frame);
        imwrite(finalImg, redImg);
        nfound++;
        return true;
    }

    //    
    //    //apply Gaussian blur to improve detection
    //    GaussianBlur(redImg, redImg, Size(9, 9), 2, 2);
    //
    //    //apply Hough transform (configured to only really work at 7m)
    //    //inputArray, outputArray, method, dp, minDistance, param1, param2, minR, maxR
    //    //redImg is 320x240
    //    HoughCircles(redImg, circles, CV_HOUGH_GRADIENT, 1, redImg.rows / 2, 50, 24, 5, 9);
    //    //if circle is found, save image and return true
    //    if (circles.size() > 0) {
    //
    //        // clone original frame to draw circle on
    //        Mat endFrame = frame.clone();
    //
    //        // draw circle
    //        for (size_t i = 0; i < circles.size(); i++) {
    //            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    //            int radius = cvRound(circles[i][2]);
    //            // circle center
    //            circle(endFrame, center, 3, Scalar(0, 255, 0), -1, 8, 0);
    //            // circle outline
    //            circle(endFrame, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    //        }
    //
    //        // save images
    //        imwrite("/home/pi/NGCP/RPI_cpslo/Datalogs/OriginalImg.jpg", frame);
    //        imwrite("/home/pi/NGCP/RPI_cpslo/Datalogs/HSVImg.jpg", redImg);
    //        imwrite("/home/pi/NGCP/RPI_cpslo/Datalogs/FinalImg.jpg", endFrame);

    return false;
}

void genSetPoints(const float &D, Autopilot_Interface &api,
        vector<float> &xSetPoints, vector<float> &ySetPoints) {
    // <TODO: Update to allow passage of camera frame parameters>
    float Fw(inputs.getFrame_Width()); //[m]

    // use these to track x and y displacement
    float dx(0.0), dy(0.0), sgn(0.0);

    //NOTE: ONLY USE THESE VALUES IF search box is assumed true
    float s45(0.707107), c45(0.707107); //store look up values for sin/cos 45

    //store target positions
    float ix, iy, hp;

    ix = api.initial_position.x;
    iy = api.initial_position.y;

    // add a set point half way north
    hp = D * s45 / 2;

    while (dx < (D * s45)) {
        //increment dy sgn * D * s45
        sgn = 1 - sgn;
        dy = sgn * D * s45;

        //add a half way north set point
        ySetPoints.push_back(hp + iy);
        xSetPoints.push_back(dx + ix);

        //pushback dy + initial_position
        ySetPoints.push_back(dy + iy);
        xSetPoints.push_back(dx + ix);

        //increment dx by 0.5 Fw
        dx += 0.5 * Fw;

        //pushback dx + initial_position
        xSetPoints.push_back(dx + ix);
        ySetPoints.push_back(dy + iy);

        //assert dy is bounded
        assert(dy <= iy + D);

        //assert dx is bounded
        assert(dx <= ix + D);

    }

    //set final setPoint to be diagonal from start
    xSetPoints.push_back(D * s45);
    ySetPoints.push_back(D * c45);
}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port

void genDatalogs(std::ofstream &Local_Pos, std::ofstream &Global_Pos,
        std::ofstream &Attitude, std::ofstream &HR_IMU, Autopilot_Interface &api, int flag) {

    // Specify type of message to write to data logs with flag
    // <TODO: Make the flags an enum>
    /*
     * int flag
     *  1   - append single line of data to all output files
     *  2   - append initial position line to Local Position NED file
     *  10  - append set point reached message to all output files
     *  11  - append ball found at following... messag to all output files
     *  20  - append header line to all output files
     *  30  - open all threads for writing
     *  31  - close all threads
     */
    // <TODO: Update this to open a new file name based on time stamp>

    bool flush = true;
    if (flag == 1) {

        mavlink_local_position_ned_t lpos = api.current_messages.local_position_ned;
        mavlink_position_target_local_ned_t ltar = api.current_messages.position_target_local_ned;
        mavlink_global_position_int_t gpos = api.current_messages.global_position_int;
        mavlink_position_target_global_int_t gtar = api.current_messages.position_target_global_int;
        mavlink_attitude_t att = api.current_messages.attitude;
        mavlink_highres_imu_t imu = api.current_messages.highres_imu;

        //print lpos and ltar
        Local_Pos << imu.time_usec << ", " <<
                lpos.x << ", " << lpos.y << ", " << lpos.z << ", " <<
                ltar.x << ", " << ltar.y << ", " << ltar.z << "\n";
        std::cout << "Local Pos and target: " << lpos.x << ", " << lpos.y << ", " << lpos.z << ", " <<
                ltar.x << ", " << ltar.y << ", " << ltar.z << "\n";

        //print gpos and gtar
        Global_Pos << imu.time_usec << ", " <<
                gpos.lat << ", " << gpos.lon << ", " << gpos.alt << ", " <<
                gtar.lat_int << ", " << gtar.lon_int << ", " << gtar.alt << "\n";

        //print Attitude
        Attitude << imu.time_usec << ", " <<
                att.roll << ", " << att.pitch << ", " << att.yaw << ", " <<
                att.rollspeed << ", " << att.pitchspeed << ", " << att.yawspeed << "\n";

        //print IMU Data
        HR_IMU << imu.time_usec << ", " <<
                imu.xacc << ", " << imu.yacc << ", " << imu.zacc << ", " <<
                imu.xgyro << ", " << imu.ygyro << ", " << imu.zgyro << ", " <<
                imu.xmag << ", " << imu.ymag << ", " << imu.zmag << "\n";

    }

    if (flag == 2) {
        //initial position is where the rbpi is powered up
        Local_Pos << "Initial Position NED XYZ = [" << api.initial_position.x << ", "
                << api.initial_position.y << ", " << api.initial_position.z << "] [m]\n";
    }

    if (flag == 10) {
        //
        Local_Pos << "Set point reached within tolerance\n";
        Global_Pos << "Set point reached within tolerance\n";
        Attitude << "Set point reached within tolerance\n";
        HR_IMU << "Set point reached within tolerance\n";
    }
    if (flag == 11) {
        //
        Local_Pos << "Ball found at following location\n";
        Global_Pos << "Ball found at following location\n";
        Attitude << "Ball found at following attitude\n";
        HR_IMU << "Ball found at following flight condition\n";
    }

    if (flag == 20) {
        Local_Pos << "Timestamp [usec], X Position, Y Position, Z Position, "
                "X Target, Y Target, Z Target\n";
        Local_Pos << "LocalPos = [...\n";

        Global_Pos << "Timestamp [usec], "
                "Latitude [deg * 1e7], Longitude [deg * 1e7], Altitude [m?], "
                "Target Lat [deg * 1e7], Target Long [deg * 1e7], Target Alt [m?]\n";
        Global_Pos << "GlobalPos = [...\n";

        Attitude << "Timestamp [usec], phi (roll) [rad], theta (pitch) [rad], psi "
                "(yaw) [rad], p (roll rate) [rad/s], q (pitch rate) [rad/s], r (yaw rate) [rad/s]\n";
        Attitude << "Attitude = [...\n";

        HR_IMU << "Timestamp [usec], xacc [m/s2], yacc [m/s2], zacc [m/s2], "
                "xgyro [rad/s], ygyro [rad/s], zgyro [rad/s], "
                "xmag [Gauss], ymag [Gauss], zmag [Gauss]\n";
        HR_IMU << "IMU = [...\n";
    }

    if (flag == 30) {
        Local_Pos.open("Out_Local Position and Target", std::ofstream::out | std::ofstream::trunc);
        Global_Pos.open("Out_Global Position and Target", std::ofstream::out | std::ofstream::trunc);
        Attitude.open("Out_Attitude", std::ofstream::out | std::ofstream::trunc);
        HR_IMU.open("Out_IMU Data", std::ofstream::out | std::ofstream::trunc);
    }

    if (flag == 31) {
        Local_Pos.close();
        Global_Pos.close();
        Attitude.close();
        HR_IMU.close();
        flush = false; //dont need to flush if streams already closed

        append_to_data_log();
    }
    if (flush) {
        //flush buffer
        Local_Pos.flush();
        Global_Pos.flush();
        Attitude.flush();
        HR_IMU.flush();
    }
}

void append_to_data_log() {
    time_t cur_time_stamp;
    string line;

    /*Creating file based on timestamp*/
    time(&cur_time_stamp);
    std::ofstream data_log;
    data_log.open(ctime(&cur_time_stamp));

    /*Read from Out_Attitude File*/
    std::ifstream attitude_file("Out_Attitude");
    if (attitude_file.is_open()) {
        while (getline(attitude_file, line)) {
            data_log << line << '\n';
        }

        attitude_file.close();
    }

    data_log << '\n' << '\n';

    std::ifstream global_file("Out_Global Position and Target");
    if (global_file.is_open()) {
        while (getline(global_file, line)) {
            data_log << line << '\n';
        }

        global_file.close();
        remove("Out_Global Position and Target");
    }

    data_log << '\n' << '\n';

    std::ifstream local_file("Out_Local Position and Target");
    if (local_file.is_open()) {
        while (getline(local_file, line)) {
            data_log << line << '\n';
        }

        local_file.close();
        remove("Out_Local Position and Target");
    }

    data_log << '\n' << '\n';

    std::ifstream imu_file("Out_IMU Data");
    if (imu_file.is_open()) {
        while (getline(imu_file, line)) {
            data_log << line << '\n';
        }

        imu_file.close();
        remove("Out_IMU Data");
    }

    data_log.close();
}

void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate, float &D) {

    // string for command line usage
    const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate> -D <search box in meters>";

    // Read input arguments
    for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n", commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                uart_name = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (argc > i + 1) {
                baudrate = atoi(argv[i + 1]);

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        if (strcmp(argv[i], "-D") == 0) {
            if (argc > i + 1) {
                D = atoi(argv[i + 1]);
            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }
    }
    // end: for each input argument

    // Done!
    return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C

void quit_handler(int sig) {
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    // autopilot interface
    try {
        autopilot_interface_quit->handle_quit(sig);
    } catch (int error) {
    }

    // serial port
    try {
        serial_port_quit->handle_quit(sig);
    } catch (int error) {
    }

    // end program here
    exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------

int main(int argc, char **argv) {
    // This program uses throw, wrap one big try/catch here
    try {
        if (argc < 2) {
            fprintf(stderr, "Input file is not specified!\n");
            //			return 1;
        }
        if (argc == 2) {
            inputs.setInputFile(argv[1]);
        }
        int result = top(argc, argv);
        return result;
    } catch (int error) {
        fprintf(stderr, "mavlink_control threw exception %i \n", error);
        return error;
    }
}
