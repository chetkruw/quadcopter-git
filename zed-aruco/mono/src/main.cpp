///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/***********************************************************************************************
 ** This sample demonstrates how to reloc a ZED camera using an ArUco marker.                  **
 ** Images are captured with the ZED SDK and cameras poses is then computed from ArUco pattern **
 ** to reset ZED tracking with this known position.                                            **
 ***********************************************************************************************/

// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "aruco.hpp"

// OCV includes
#include <opencv2/opencv.hpp>

// Standard includes
#include <time.h>
#include <sched.h>

using namespace sl;
using namespace std;

int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::VGA;
    init_params.camera_fps = 60;
    init_params.coordinate_units = UNIT::METER;
    init_params.sensors_required = false;
    init_params.depth_mode = DEPTH_MODE::PERFORMANCE;
    // init_params.coordinate_system = RIGHT_HANDED_Z_UP;
    // init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;

    RuntimeParameters run_params;
    run_params.enable_depth = false;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        cout << "Error, unable to open ZED camera: " << err << "\n";
        zed.close();
        return 1; // Quit if an error occurred
    }

    auto cameraInfo = zed.getCameraInformation();
    Resolution image_size = cameraInfo.camera_resolution;
    Mat image_zed(image_size, MAT_TYPE::U8_C4);
    cv::Mat image_ocv = cv::Mat(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(MEM::CPU));
    cv::Mat image_ocv_rgb;

    auto calibInfo = cameraInfo.calibration_parameters.left_cam;
    cv::Matx33d camera_matrix = cv::Matx33d::eye();
    camera_matrix(0, 0) = calibInfo.fx;
    camera_matrix(1, 1) = calibInfo.fy;
    camera_matrix(0, 2) = calibInfo.cx;
    camera_matrix(1, 2) = calibInfo.cy;

    cv::Matx<float, 4, 1> dist_coeffs = cv::Vec4f::zeros();

    float actual_marker_size_meters = 0.16f; // real marker size in meters
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);

    cout << "Make sure the ArUco marker is a 6x6 (100), measuring " << actual_marker_size_meters * 1000 << " mm" << endl;

    Transform pose;
    Pose zed_pose;
    vector<cv::Vec3d> rvecs, tvecs;
    vector<int> ids;
    vector<vector<cv::Point2f> > corners;
    string position_txt;

    bool can_reset = false;

    PositionalTrackingParameters tracking_params;
    tracking_params.enable_imu_fusion = false; // for this sample, IMU (of ZED-M) is disable, we use the gravity given by the marker.
    // zed.enablePositionalTracking(tracking_params);

    // Creating Buffers For Value Storing
    static int IMAX = 500;
    double xbuffer[IMAX];
    double ybuffer[IMAX];
    double zbuffer[IMAX];
    double timebuffer[IMAX];
    double *xp = xbuffer;
    double *yp = ybuffer;
    double *zp = zbuffer;
    double *timep = timebuffer;
    double x_old, y_old, z_old;
    double x_grab, y_grab, z_grab;
    double x_new, y_new, z_new;
    int firstrun = 1;
    double alpha = 0.1; // for low pass, lower is smoother


    // create time structure
    struct timespec first = {}, grab = {}, now = {};
    double timenow, nownano; // for reporting timenow

    cout << "Starting Timing Check" << endl;
    int count = 0;
    float total_ocv_time = 0;
    clock_t t = clock();

    // Loop until 'q' is pressed
    char key = '.';
    zed.resetPositionalTracking(pose);
    while ((key != 'q' && count < IMAX)) {
	if (zed.grab(run_params) == ERROR_CODE::SUCCESS) {
            // Start Timer
	    clock_t ocv_t = clock();
		
	    // Retrieve the left image
            zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, image_size);

            // convert to RGB
            cv::cvtColor(image_ocv, image_ocv_rgb, cv::COLOR_RGBA2RGB);
            
	    ocv_t = clock();
	    // detect marker
            cv::aruco::detectMarkers(image_ocv_rgb, dictionary, corners, ids);
	    ocv_t = clock() - ocv_t;
            total_ocv_time += ocv_t;

            // get actual ZED position
            zed.getPosition(zed_pose);

            // display ZED position
            // cv::rectangle(image_ocv_rgb, cv::Point(0, 0), cv::Point(490, 75), cv::Scalar(0, 0, 0), -1);
            // cv::putText(image_ocv_rgb, "Loaded dictionary : 6x6.     Press 'SPACE' to reset the camera position", cv::Point(10, 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(220, 220, 220));
            position_txt = "ZED  x: " + to_string(zed_pose.pose_data.tx) + "; y: " + to_string(zed_pose.pose_data.ty) + "; z: " + to_string(zed_pose.pose_data.tz);
	    // cv::putText(image_ocv_rgb, position_txt, cv::Point(10, 35), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(236, 188, 26));
	    // cout << position_txt << endl;
	    // cout << to_string(count) << endl;

            // if at least one marker detected
            if (ids.size() > 0) {
                cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_size_meters, camera_matrix, dist_coeffs, rvecs, tvecs);
                pose.setTranslation(sl::float3(tvecs[0](0), tvecs[0](1), tvecs[0](2)));
                pose.setRotationVector(sl::float3(rvecs[0](0), rvecs[0](1), rvecs[0](2)));
                pose.inverse();
                can_reset = true;

                // cv::aruco::drawDetectedMarkers(image_ocv_rgb, corners, ids);
                // cv::aruco::drawAxis(image_ocv_rgb, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], actual_marker_size_meters * 0.5f);
                position_txt = "Aruco x: " + to_string(pose.tx) + "; y: " + to_string(pose.ty) + "; z: " + to_string(pose.tz);
                // cv::putText(image_ocv_rgb, position_txt, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(124, 252, 124));


		// grab time
		clock_gettime(CLOCK_MONOTONIC_RAW,&grab);

		//position grabbed as pose.tx,-tz,ty;
		
		x_grab = pose.tx;
		y_grab = -pose.tz;
		z_grab = pose.ty;

		if (firstrun > 0) {
			first = grab;
			x_old = x_grab;
			y_old = y_grab;
			z_old = z_grab;
			firstrun = 0; // set first time "old" values
		}

		// current time
		now.tv_sec = grab.tv_sec - first.tv_sec;
		now.tv_nsec = grab.tv_nsec - first.tv_nsec;
		nownano = (double) now.tv_nsec;
		timenow = now.tv_sec + (nownano/1e9);

		// calc new values
		x_new = (x_grab-x_old)*alpha + (1-alpha)*x_old;
		y_new = (y_grab-y_old)*alpha + (1-alpha)*y_old;
		z_new = (z_grab-z_old)*alpha + (1-alpha)*z_old;

		if (xp < xbuffer+IMAX) {
		    *xp++ = x_new;
		    *yp++ = y_new;
		    *zp++ = z_new;
		    *timep++ = timenow;
		}	

		// update old values
		x_old = x_new;
		y_old = y_new;
		z_old = z_new;
	
		//if (count % 10 == 0) {	
		if (true) {
		// print to console for now
		printf("Time: %f, x: %f, y: %f, z: %f\n",timenow,x_new,y_new,z_new);
		}

            } else
                can_reset = false;

            // Display image
            // cv::imshow("Image", image_ocv_rgb);

            // if KEY_R is pressed and aruco marker is visible, then reset ZED position
            if ((key == ' ') && can_reset)
                zed.resetPositionalTracking(pose);
        }
	count++;
    }
    t = clock() - t;
    float total_time = ((float)t)/CLOCKS_PER_SEC;
    cout << "Total Time        : " << to_string(total_time) << endl;
    cout << "Time w/ Detection : " << to_string(total_ocv_time/CLOCKS_PER_SEC) << endl;
    cout << "Number of Cycles  : " << to_string(count) << endl;
    cout << "Average Rate      : " << to_string(count/total_time) << endl;

    FILE *fp;
    fp = fopen("basic_output.csv","w+");
    fprintf(fp, "ID, TIME, X, Y, Z\n");
    for (int i = 0; i < IMAX; i++) {
	fprintf(fp, "1, %lf, %lf, %lf, %lf\n", timebuffer[i], xbuffer[i], ybuffer[i], zbuffer[i]);
    }

    fclose(fp);

    zed.close();
    return 0;

}
