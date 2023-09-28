#include "vtracker.h"

void frameCb(const sensor_msgs::ImageConstPtr &msg)
{
    // std::cout << "framecb" << std::endl;

    frame_rcv = cv_bridge::toCvShare(msg, "bgr8")->image;
    output = frame_rcv.clone();

    cv::Point2f obs, track;
    obs.x = ball_center_rcv.x;
    obs.y = ball_center_rcv.y;

    // KF Update
    // Measure Matrix Z
    meas.at<float>(0) = ball_center_rcv.x;
    meas.at<float>(1) = ball_center_rcv.y;

    kf.correct(meas); // Kalman Correction
    kf.statePost = state;
    // Publish centroid e por nome

    cv::circle(output, obs, 6, cv::Scalar(0, 255, 0), -1);
    cv::putText(output, "Obs",
                cv::Point(obs.x + 3, obs.y - 3),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    track.x = state.at<float>(0);
    track.y = state.at<float>(1);
    cv::circle(output, track, 6, cv::Scalar(255, 0, 0), -1);
    cv::putText(output, "Tra",
                cv::Point(track.x - 3, track.y - 3),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

    geometry_msgs::Point ball_center_track;
    ball_center_track.x = track.x;
    ball_center_track.y = track.y;
    pub_ball_center_tracker.publish(ball_center_track);

    //std::cout << ball_center_track.x << " " << ball_center_track.y << std::endl;

    // save in the array for future plot
    est_recv.push_back(ball_center_rcv);
    est_kf.push_back(ball_center_track);


    // exercício 3 -------------->
    
    int len = est_kf.size() - 1;
    float m1, m2, diff, angle_rad, angle_deg;
    if (len > 2)
    {
        angle_rad = - atan2(est_kf[len].y - est_kf[len - 1].y, est_kf[len].x - est_kf[len - 1].x )
                    + atan2(est_kf[len - 1].y - est_kf[len - 2].y, est_kf[len - 1].x - est_kf[len - 2].x );
        angle_deg = angle_rad * 180.0 / M_PI;
        
        // For debug of angle value for detecting bowling ball collision with the ground
        //std::cout << angle_deg << std::endl;

        if (angle_deg > 35)
        {
            //std::cout << "Collision" << std::endl;
            cv::putText(output, "Collision",
                        cv::Point((int)output.size().width - 200, 100),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
        }
    }
    // <----------------------------
    
    cv::imshow("output", output);
    cv::waitKey(1);

    new_frame = true;
}

void ballcenterCb(const geometry_msgs::Point &msg)
{
    // std::cout << "ballCentercb" << std::endl;

    ball_center_rcv = msg;
    new_center = true;
}

int main(int argc, char **argv)
{

    // Init the ros system
    ros::init(argc, argv, "vtracker_node");

    // Create the node handles to establish the program as a ROS node
    ros::NodeHandle n_public;
    image_transport::ImageTransport it(n_public);

    image_transport::Subscriber sub_frame = it.subscribe("/vloader/frame", 1, frameCb);
    ros::Subscriber sub_ball_center = n_public.subscribe("/vloader/ball_center", 1, ballcenterCb);

    pub_ball_center_tracker = n_public.advertise<geometry_msgs::Point>("/vtracker/ball_center_tracker", 1);

    // Transition State Matrix A
    // [ 1 0 dT 0  ]
    // [ 0 1 0  dT ]
    // [ 0 0 1  0  ]
    // [ 0 0 0  1  ]
    cv::setIdentity(kf.transitionMatrix);
    kf.transitionMatrix.at<float>(2) = 1 / fps;
    kf.transitionMatrix.at<float>(7) = 1 / fps;

    // Measure Matrix H
    // [ 1 0 0 0 ]
    // [ 0 1 0 0 ]
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(5) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0    ]
    // [ 0    Ey  0     0    ]
    // [ 0    0   Ev_x  0    ]
    // [ 0    0   0     Ev_y ]
    cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(10) = 5.0f;
    kf.processNoiseCov.at<float>(15) = 5.0f;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

    // Covariance Matrix P
    // cv::setIdentity(kf.errorCovPre, cv::Scalar(1));
    kf.errorCovPre.at<float>(0) = 5.0f;
    kf.errorCovPre.at<float>(5) = 5.0f;
    kf.errorCovPre.at<float>(10) = 5.0f;
    kf.errorCovPre.at<float>(15) = 5.0f;

    // ros::spin();

    // State Matrix X
    state.at<float>(0) = 0;
    state.at<float>(1) = 0;
    state.at<float>(2) = 0;
    state.at<float>(3) = 0;

    ros::Rate rate(30.0);
    while (ros::ok())
    {
        state = kf.predict();
        kf.statePre = state;
        ros::spinOnce();
        rate.sleep();
    }
}