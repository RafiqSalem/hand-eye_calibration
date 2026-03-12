# Hand-Eye Calibration

<img src="resources/eih-process.gif" height=300px align="left"/>
<img src="resources/eth-process.gif" height=300px align="middle"/>

 While performing the calibration process: 
 - Left image: __eye-in-hand__: To find the `T_robot_camera`. 
 - Right image: __eye-to-hand__. To find the `T_base_camera`.

## __Index__:
- [Problem](https://github.com/cvg25/hand-eye_calibration#problem)
- [Motivation](https://github.com/cvg25/hand-eye_calibration#motivation)
- [Proposed solution explained](https://github.com/cvg25/hand-eye_calibration#proposed-solution-explained)
- [How to use](https://github.com/cvg25/hand-eye_calibration#how-to-use)
- [Advantages](https://github.com/cvg25/hand-eye_calibration#advantages)
- [Limitations](https://github.com/cvg25/hand-eye_calibration#limitations)
- [Hardware](https://github.com/cvg25/hand-eye_calibration#hardware)
- [Acknowledgments](https://github.com/cvg25/hand-eye_calibration#acknowledgments)

## __Problem__:


<img src="resources/eye-in-hand.svg" height=280px align="left"/>
<img src="resources/eye-to-hand.svg" height=280px align="middle"/>

 Left image: __eye-in-hand__. To find the `T_robot_camera`. Right image: __eye-to-hand__. To find the `T_ext_camera`.
[Source](https://doc.rc-visard.com/latest/en/handeye_calibration.html)

"In robotics and mathematics, the hand eye calibration problem (also called the robot-sensor or robot-world calibration problem) is the problem of determining the transformation between a robot end-effector and a camera or between a robot base and the world coordinate system." [Learn more.](https://en.wikipedia.org/wiki/Hand_eye_calibration_problem)

## __Motivation__:
To develop a calibration procedure that:
- It is __end to end__. No human intervention once the system is ready to start the calibration process.
- It is suitable for both cases, __eye-in-hand__ (camera mounted on the robot) and __eye-to-hand__ (camera fixed to the workspace).
- __It optimizes for depth and rgb channels at the same time.__
- It is easily parameterized by a .json file to fit any posible configuration without need of code editing.
- It is fast and easy compared to other more manual or different steps solutions.
- It uses common python libraries.



## __Proposed solution explained__:
The calibration process estimates the camera extrinsics with respect to:
- Robot base coordinates for the __eye-to_hand__.
- Tool Center Position coordinates for the __eye-in-hand__.

During the calibration, the robot moves to a predefined set of 3D locations. At each 3D location the camera detects the center point of the checkerboard pattern, and it stores both: the position of the robot with respect to the base and the position of the center point detected with respect to the center of the camera.

The 3D position of a pixel seen in an rgbd image can be computed as:
```
point_z = camera_depth_img[checkerboard_center_pix[1]][checkerboard_center_pix[0]]
point_x = np.multiply(checkerboard_center_pix[0]-camera.intrinsics[0][2], point_z/camera.intrinsics[0][0])
point_y = np.multiply(checkerboard_center_pix[1]-camera.intrinsics[1][2], point_z/camera.intrinsics[1][1])
```  
Where `camera.instrisics` is the intrisics matrix of the camera:
```
camera.instrics = [[fx, 0, ox],
                   [0, fy, oy],
                   [0,  0,  1]] # f = focal distance, o = offset from origin.
```
The result of visiting all 3D locations is two datasets of equivalent points but in different coordinate systems. To find the optimal rigid transformation matrix between those two datasets, the following steps are executed:
1. Find the centroids of both dataset.
2. Bring both dataset to the origin then find the optimal rotation R.
3. Find the traslation t.

```
def get_rigid_transform(A, B):
    assert len(A) == len(B)
    N = A.shape[0]; # Total points
    centroid_A = np.mean(A, axis=0) #  Find centroids
    centroid_B = np.mean(B, axis=0)
    AA = A - np.tile(centroid_A, (N, 1)) # Centre the points
    BB = B - np.tile(centroid_B, (N, 1))
    H = np.dot(np.transpose(AA), BB) # Dot is matrix multiplication for array
    U, S, Vt = np.linalg.svd(H) # Find the rotation matrix R
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0: # Special reflection case
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)
    t = np.dot(-R, centroid_A.T) + centroid_B.T # Find the traslation t
    return R, t
```
Visit this post by [nghiaho.com](http://nghiaho.com/?page_id=671) to learn more as it is the original source of inspiration. The implementation of only the __hand-to-eye__ case was first seen on [github.com/andyzeng/visual-pushing-grasping](https://github.com/andyzeng/visual-pushing-grasping).

As a result of the calibration process, an image will be shown where the datasets should be aligned. It can be observed that the robot dataset (blue) has perfect distribution of points, whereas the dataset captured by the RGBD camera (red) will reflect the precision given by its depth sensor. 
Red points will not overlap the blue ones unless RGBD precision is 100% accurate.

<img src="resources/out1.png" height=280px align="left"/>
<img src="resources/out2.png" height=280px align="middle"/>

 Images: examples of different calibrations results.



## __How to use__:
1. Put the checkerboard pattern fixed to either the robot hand tool center position (eye-to-hand) or the workspace (eye-in-hand). Make sure it will not move during the calibration process.

<img src="resources/eih-checkerboard.jpg" height=280px align="left"/> 
<img src="resources/eth-checkerboard.jpg" height=280px align="middle"/>

 Left image: __eye-in-hand__, checkerboard fixed to workspace. Right image: __eye-to-hand__, checkerboard fixed to robot end-effector.

2. Go to `hand-eye_calibration/rs_server/` and compile `rs_server.cpp`. Make sure you have the [librealsense SDK](https://github.com/IntelRealSense/librealsense/) installed.
```
$ cd hand-eye_calibration/rs_server/
$ make
```
3. Connect realsense to the computer using a USB3.0 interface connection.
4. Start RealSense TCP streaming server and indicate the `PORT`:
```
$ ./rs_streamer
```
Keep it running while calibrating, each time the robot connects to retrieve an RGBD frame it will output `Connected to client.` on the terminal. To further test a python TCP client that fetches RGBD frames from the server edit the `configurations/camera_config.json` (see next step for details) and then run: `$ python camera_streamer.py`

5. Create a brand new conda environment and run: `$ pip install -r requirements.txt` in order to install all the python libraries needed at the versions tested.

6. Open the `configurations/calibrate_config.json` file and fill in the parameters:
  - __calibration_type__:
    - "EYE_IN_HAND": the camera mounted on the robot.
    - "EYE_TO_HAND": the camera is fixed to the workspace and independent of robot moves. 
  - __calib_grid_step__: it defines the step distance in meters between "photo positions". (e.g. 0.05 meters)
  - __workspace_limits__: it is a cube defined by 3 points (X,Y,Z) from the robots' base. Inside this cube the robot will move to the different "photo positions" parameterized by __calib_grid_step__ to capture a data point. It is important to take into account that the closest distance from the camera to the checkerboard is higher than the minZ value of the depth channel. Note: on RealSense D415, minZ is 0.5 meters. shape = (3,2). rows = X, Y, Z. colums = MIN, MAX values.
  - __reference_point_offset__: it is the point (X,Y,Z) position of the center of the checkerboard pattern with respect to the robots' base on eye-in-hand or the robot's TCP on eye-to-hand.
  - __tool_orientation__: it is the orientation of the robot TCP in the robots' base system for every "photo position".
  - __checkerboard_size__: the size of the checkerboard. E.g. a checkerboard of 4x4 black-white squares is of size 3 as that is the number of inner crosses.
  - __camera_config_file__: the configuration file for the camera. Example at: `configurations/camera_config.json`
    - Should edit:
      - __tcp_host_ip__: IP Address of the rs_streamer TCP server.  
      - __tcp_port__: PORT of the rs_streamer TCP server.
    - Might edit: If you change the parameters (im_height, im_width, buffer_size), you should also edit the rs_server to match.
  - __robot_config_file__: the configuration file for the UR Robot. Example at: `configurations/robot_config.json`
    - Should edit:
      - __robot_ip__: IP Address of the robot.
      - __program__: path to program that should be loaded on the robot, which includes its installation: tcp configuration, center of gravity... etc.
      - __home_joints_rad__: robot home pose. In radians for each of its joints.
    - Might need to edit:
      - __tcp_port__: TCP Port connection of the robot.
      - __rtc_port__: RTC Port connection of the robot.
      - __dashboard_port__: Dashboard
    - Edit at your own risk: the rest of parameters regarding velocities and accelerations have been tested for UR robots to perform the calibration under safety conditions. Changing those might speed up the process but also suffer from bad image captures due to sharp motions, affecting the precision of the calibration result.

7. Open a different terminal and execute `$ python calibrate.py` to move the robot and calibrate. Note: the robot will move inside the 3D cube grid indicated by workspace_limits with a step size of calib_grid_step at the config.json file. Be cautious.

8. Once the calibration process ends, two files will be stored in `hand-eye_calibration/calibrations/`:
 - `DATETIME_camera_pose.txt`: it contains the transformation matrix between the robot and the camera.
 - `DATETIME_camera_depth_offset.txt`: it contains a value which is a scale factor that should be multiplied with each pixel captured from the camera. Note: as tested RealSense D415 series are not likely to suffer a scaling problem, but other devices might.

9. To test the result of the calibrated camera extrinsics edit the `configurations/touch_tester_config.json` file to meet your setup and execute `$ python touch_tester.py`. It provides a UI where the user can click at any point in the RGBD image and the robot moves its end-effector to the 3D location of that point.

## __Advantages__:
- It directly optimizes for both depth and rgb channels at the same time. This has many advantages. For example, it introduces the noise of the depth sensor in the calibration process and calculates the transformation matrix with it. As opposed to other solutions where the calibration process is only estimated using the rgb channels and a reference point (e.g. ArUco marker) which its depth precision differs from the depth channel of the image, therefore not capturing this discrepancies inside the calibration process.
- It is fast and easy compared to other more manual or different steps solutions.
- It uses common python libraries.
- It is suitable for both cases, __eye-in-hand__ (camera mounted on the robot) and __eye-to-hand__ (camera fixed to the workspace).
- It is __end to end__. No human intervention once the system is ready to start the calibration process.

## __Limitations__:
- Precision increases with samples. As the robot has to move to different photo positions, it is sometimes difficult to use in very small spaces or with obstacles.
- It is only suitable for RGBD Cameras.

## __Hardware__:

- RGBD Camera, Robot and PC.

 #### Tested on:
  - RGBD Camera: Intel RealSense D415
  - Robot: UR10e
  - PC: Ubuntu 18 and 20 running Python 3.8

## __Acknowledgments__:

  This calibration software was developed at INESCOP Footwear Technological Institute (https://www.inescop.es) to calibrate the cameras for the [SoftManBot H2020](http://softmanbot.eu/) project. Polígono Industrial Campo Alto, Elda, Alicante (Spain).

  Method and source code inspired by:

  https://github.com/IntelRealSense/librealsense

  https://github.com/andyzeng/visual-pushing-grasping

  https://github.com/hengguan/Hand-Eye-Calibration-Matlab

  https://github.com/nghiaho12/rigid_transform_3D






  // Code inspired by:
// https://github.com/IntelRealSense/librealsense.git
// https://github.com/andyzeng/visual-pushing-grasping.git

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>
#include <signal.h>
#include <iomanip>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

//------------------- TCP Server Code -------------------
//-------------------------------------------------------

typedef void * (*THREADFUNCPTR)(void *);

class Server {

    public:
        Server(int port);
        void * listener_thread();
        void init_listener_thread();
        void update_buffer(const unsigned char * data, int offset, unsigned long numbytes);

    private:
        int init_sock, conn_sock;
        char * send_buffer;
        int buffer_size = 1024;
        char receive_buffer[1024];
        struct sockaddr_in serv_addr;
        struct sockaddr_storage serv_storage;
        socklen_t addr_size;
        pthread_mutex_t buffer_access_mutex;
        pthread_t listener_thread_id;
        unsigned long frame_size;
};

Server::Server(int port) {
    init_sock = socket(PF_INET, SOCK_STREAM, 0);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons (port);
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    memset(serv_addr.sin_zero, '\0', sizeof(serv_addr.sin_zero));
    bind(init_sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    send_buffer = new char[buffer_size];
}

void Server::init_listener_thread() {
    pthread_create(&listener_thread_id, NULL, (THREADFUNCPTR) &Server::listener_thread, this);
    pthread_mutex_init(&buffer_access_mutex, NULL);
}

void * Server::listener_thread() {
    while(true) {
        if (listen(init_sock, 5) == 0)
            printf ("Listening...\n");
        else
            printf ("Error.\n");

        // Creates new socket for incoming connection
        addr_size = sizeof(serv_storage);
        conn_sock = accept (init_sock, (struct sockaddr *) &serv_storage, &addr_size);
        printf ("Connected to client.\n");

        while(true) {

            // Parse ping from client
            memset(receive_buffer, 0, sizeof(receive_buffer));
            int resp_msg_size = recv(conn_sock, receive_buffer, 64, 0);
            if (resp_msg_size <= 0) break;
            // Send buffer data
            pthread_mutex_lock(&buffer_access_mutex);
            int msg_size = send(conn_sock, send_buffer, buffer_size, MSG_MORE);
            if (msg_size == 0 ) printf("Warning: No data was sent to client.\n");
            int tmp = errno;
            if (msg_size < 0) printf ("Errno %d\n", tmp);
            pthread_mutex_unlock(&buffer_access_mutex);
        }
    }
}

void Server::update_buffer(const unsigned char * data, int offset, unsigned long numbytes) {
    pthread_mutex_lock(&buffer_access_mutex);

    // Update buffer size
    unsigned long new_buffer_size = numbytes + offset;
    if (new_buffer_size > buffer_size) {
        delete [] send_buffer;
        buffer_size = new_buffer_size;
        send_buffer = new char[buffer_size];
    }

    // Copy data
    memcpy(send_buffer + offset, data, numbytes);
    pthread_mutex_unlock(&buffer_access_mutex);
}

//------------------------------------------------------------------------------------------------

// Configure all streams to run at 1280x720 resolution at 30 frames per second
const int STREAM_WIDTH = 1280;
const int STREAM_HEIGHT = 720;
const int STREAM_FPS = 30;
const int WAIT_FRAMES_FOR_AUTOEXPOSURE = 30;

using namespace std;
int main(int argc, char * argv[]) try {

  std::cout << "Starting Realsense Server, enter PORT: ";
  int port;
  cin >> port;

  Server realsense_server(port);
  realsense_server.init_listener_thread();

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
	rs2::config configuration;
	rs2::context ctx;
	rs2::device dev;

	auto devices = ctx.query_devices();
  // Comprobamos si hay mas de un dispositivo conectado:
  // - si lo hay, preguntamos al usuario que seleccione por número de serie.
  // - si no lo hay escogemos el unico.
  if (devices.size() > 1)
  {
    std::cout << "More than one Realsense detected: " << std::endl;
    for (unsigned int i = 0; i < devices.size(); i++ ){
      std::string serial = devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      std::cout << "["<< i << "] Serial: " << serial << std::endl;
    }
    std::cout << "Enter option: ";
    int option;
    cin >> option;
    dev = devices[option];
  } else {
    dev = devices[0];
  }

  const char* camera_name = dev.get_info(RS2_CAMERA_INFO_NAME);

std::cout << "Detected camera : " << camera_name << std::endl;

if (strcmp(camera_name, "Intel RealSense D435") == 0) // Check for compatibility
{
    std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    std::string json_file_name = "./camera_configuration.json";
    std::cout << "Configuring camera : " << serial << std::endl;

    auto advanced_mode_dev = dev.as<rs400::advanced_mode>();

    // Check if advanced-mode is enabled to pass the custom config
    if (!advanced_mode_dev.is_enabled())
    {
        advanced_mode_dev.toggle_advanced_mode(true);
        std::cout << "Advanced mode enabled." << std::endl;
    }

    // Select the custom configuration file
    std::ifstream t(json_file_name);
    std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    advanced_mode_dev.load_json(preset_json);

    configuration.enable_device(serial);
    configuration.enable_stream(rs2_stream::RS2_STREAM_DEPTH, STREAM_WIDTH, STREAM_HEIGHT, RS2_FORMAT_Z16, STREAM_FPS);
    configuration.enable_stream(rs2_stream::RS2_STREAM_COLOR, STREAM_WIDTH, STREAM_HEIGHT, RS2_FORMAT_RGB8, STREAM_FPS);

    // Start streaming
    rs2::pipeline_profile profile = pipe.start(configuration);

    rs2::colorizer color_map;

    // Print active device information
    rs2::pipeline_profile active_pipe_profile = pipe.get_active_profile();
    rs2::device dev = active_pipe_profile.get_device();

    std::cout << "Device information:" << std::endl;
    for (int i = 0; i < static_cast<int>(RS2_CAMERA_INFO_COUNT); i++)
    {
        rs2_camera_info info_type = static_cast<rs2_camera_info>(i);
        std::cout << "  " << std::left << std::setw(20) << info_type << " : ";

        if (dev.supports(info_type))
            std::cout << dev.get_info(info_type) << std::endl;
        else
            std::cout << "N/A" << std::endl;
    }

    // Get sensors
    std::vector<rs2::sensor> sensors = dev.query_sensors();
    rs2::sensor depth_sensor = sensors[0];
    rs2::sensor color_sensor = sensors[1];

    // Enable auto white balance
    rs2_option wb_option_type = static_cast<rs2_option>(11);
    if (color_sensor.supports(wb_option_type))
        color_sensor.set_option(wb_option_type, 1);

    // Wait auto exposure
    for (int i = 0; i < WAIT_FRAMES_FOR_AUTOEXPOSURE; ++i)
        pipe.wait_for_frames();

    // Get intrinsics
    rs2::video_stream_profile color_stream_profile =
        active_pipe_profile.get_stream(rs2_stream::RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    rs2_intrinsics color_intrinsics = color_stream_profile.get_intrinsics();

    float color_intrinsics_arr[9] =
    {
        color_intrinsics.fx, 0.0f, color_intrinsics.ppx,
        0.0f, color_intrinsics.fy, color_intrinsics.ppy,
        0.0f, 0.0f, 1.0f
    };

    float depth_scale = depth_sensor.as<rs2::depth_sensor>().get_depth_scale();

    rs2::align align(rs2_stream::RS2_STREAM_COLOR);

    // Filters
    rs2::decimation_filter dec_filter;
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1);

    rs2::threshold_filter thr_filter;
    thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.100);
    thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 4.000);

    rs2::spatial_filter spat_filter;
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.6);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 8);

    rs2::temporal_filter temp_filter;
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.400);

    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);

    while (true)
    {
        rs2::frameset data = pipe.wait_for_frames();
        rs2::frame color = data.get_color_frame();

        auto processed = align.process(data);
        rs2::depth_frame aligned_depth = processed.get_depth_frame();

        rs2::frame filtered = aligned_depth;

        filtered = dec_filter.process(filtered);
        filtered = thr_filter.process(filtered);
        filtered = depth_to_disparity.process(filtered);
        filtered = spat_filter.process(filtered);
        filtered = temp_filter.process(filtered);

        aligned_depth = disparity_to_depth.process(filtered);

        int depth_size = aligned_depth.get_width() *
                         aligned_depth.get_height() *
                         aligned_depth.get_bytes_per_pixel();

        realsense_server.update_buffer(
            (unsigned char*)aligned_depth.get_data(),
            10 * 4,
            depth_size
        );

        int color_size =
            data.get_color_frame().get_width() *
            data.get_color_frame().get_height() *
            data.get_color_frame().get_bytes_per_pixel();

        realsense_server.update_buffer(
            (unsigned char*)color.get_data(),
            10 * 4 + depth_size,
            color_size
        );

        realsense_server.update_buffer(
            (unsigned char*)color_intrinsics_arr,
            0,
            9 * 4
        );

        realsense_server.update_buffer(
            (unsigned char*)&depth_scale,
            9 * 4,
            4
        );
    }
}
else
{
    std::cout << "Detected camera : " << camera_name << std::endl;
    std::cout << "Selected device is not an Intel RealSense D435, check the devices list." << std::endl;
    return EXIT_FAILURE;
}

  return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n  " << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch (const std::exception& e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}

