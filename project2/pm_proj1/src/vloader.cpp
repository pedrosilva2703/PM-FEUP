
#include "vloader.h"

int main(int argc, char **argv)
{

  // Init the ros system
  ros::init(argc, argv, "vloader_node");

  // Create the node handles to establish the program as a ROS node
  ros::NodeHandle n_public;

  image_transport::ImageTransport it(n_public);
  image_transport::Publisher pub_frame = it.advertise("/vloader/frame", 1);

  ros::Publisher pub_ball_center;
  pub_ball_center = n_public.advertise<geometry_msgs::Point>("/vloader/ball_center", 1);

  readParams();

  // Open video
  cv::VideoCapture cap(video_path);

  // Check if video opened successfully
  if (!cap.isOpened())
  {
    std::cout << "Error opening video stream or file" << std::endl;
    return -1;
  }

  cv::Mat frame;
  int fps = 30;

  // Acquire input size
  cv::Size frame_size = cv::Size((int)cap.get(cv::CAP_PROP_FRAME_WIDTH), (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT));

  // Define the codec and create VideoWriter object
  std::string final_video_path = "/home/percmap/catkin_ws/src/pm_proj1/export/p1_plastic.mp4";
  size_t found = video_path.find("Plastic");
  if (found != std::string::npos)
  {
    final_video_path = "/home/percmap/catkin_ws/src/pm_proj1/export/p1_plastic.mp4";
  }
  else
  {
    final_video_path = "/home/percmap/catkin_ws/src/pm_proj1/export/p1_tennis.mp4";
  }
  cv::VideoWriter writer(final_video_path, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), fps, frame_size);

  // Iterate through frames
  while (cap.isOpened())
  {

    // Capture next frame
    cap >> frame;

    // If the frame is empty, break immediately
    if (frame.empty())
      break;

    // exercise 1 ------>
    cv::Mat out;
    out = frame.clone();

    // b) - Segment by color
    cv::Mat blur, frame_HSV, frame_threshold, frame_result;

    cv::medianBlur(frame, blur, 3);
    cv::cvtColor(blur, frame_HSV, cv::COLOR_BGR2HSV);

    cv::inRange(frame_HSV, cv::Scalar(lower[0], lower[1], lower[2]),
                cv::Scalar(upper[0], upper[1], upper[2]), frame_threshold);

    // Create a structuring element (SE)
    int morph_size = 1; // for kernel of size 3
    int iterations = 2;
    cv::Mat element = getStructuringElement(
        cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
        cv::Point(morph_size, morph_size));

    cv::Mat erod, dill;
    // For Dilation
    cv::dilate(frame_threshold, dill, element, cv::Point(-1, -1), iterations);
    // For Erosion
    cv::erode(dill, frame_result, element, cv::Point(-1, -1), iterations);
    //cv::imshow("Mask", frame_result);
    // Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat contourOutput = frame_result.clone();
    cv::findContours(contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    // Moments
    std::vector<cv::Moments> mu(contours.size());
    for (size_t i = 0; i < contours.size(); i++)
    {
      mu[i] = moments(contours[i]);
    }
    std::vector<cv::Point2f> mc(contours.size());
    for (size_t i = 0; i < contours.size(); i++)
    {
      // add 1e-5 to avoid division by zero
      mc[i] = cv::Point2f(static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5)),
                          static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5)));
    }

    // Draw the contours
    cv::Mat contourImage(frame_threshold.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Scalar color_blue = cv::Scalar(255, 0, 0);

    // Ball center
    geometry_msgs::Point ball_center;

    std::vector<cv::Point> max_contour;
    max_contour = contours[0];
    int max_contour_idx = 0;

    for (size_t idx = 0; idx < contours.size(); idx++)
    {
      float cArea = cv::contourArea(contours[idx]);
      if (cArea > cv::contourArea(contours[max_contour_idx]))
      {
        max_contour = contours[idx];
        max_contour_idx = idx;
      }
    }
    if (cv::contourArea(max_contour) > 5000)
    {
      cv::drawContours(out, contours, max_contour_idx, color_blue, 2);
      cv::circle(out, mc[max_contour_idx], 5, color_blue, -1);
      ball_area.push_back(cv::contourArea(max_contour));
      ball_center.x = mc[max_contour_idx].x;
      ball_center.y = mc[max_contour_idx].y;

      
      //Código para o exercício 3
      cv::Rect box = cv::boundingRect(max_contour);
      cv::rectangle(out, box, cv::Scalar(0, 0, 255), 2);
      float ratio_height = (float)(box.height) / bnd_height;
      float ratio_width = (float)(box.width) / bnd_width;
/*
      //verificar e atribuir dimensoes maximas da bola
      if ((box.width > bnd_width) && (box.height > bnd_height))
      {
        bnd_width = box.width;
        bnd_height = box.height;

      }//transiçoes
      else if( state==0 && ratio_height < 0.90 && ratio_width > 1.00 ){
        state=1;
      }
      else if( state==1 && ratio_height > 0.90 && ratio_width < 1.00 ){
        state=0;
      }
      //outputs da maquina de estados
      if (state != 0)
      {
        cv::putText(out, "Collision",
                    cv::Point(out.size().width - 150, 50),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
      }
      //Fim do código para o exercício 3
*/      
    }

    // Publish the ball center position
    pub_ball_center.publish(ball_center);

    sensor_msgs::ImagePtr msg;

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = ros::Time::now();
    pub_frame.publish(msg);

    // Write the contours and centroid into the output file
    writer.write(out);

    // --------->

    // Display the frame
    //cv::imshow("Result", out);

    // Press Space Bar to continue, ESC to exit
    cv::waitKey(500);
/*
    char c = (char)cv::waitKey(0);
    if (c == 27)
      break;
*/
    ros::spinOnce();
  }

  //  When everything is done, release the video capture and writer objects
  cap.release();
  writer.release();

  // Close all the frames
  cv::destroyAllWindows();

  // For Exercise 1 and balls area
  double sumArea = 0;

  for (int i = 0; i < ball_area.size(); i++)
  {
    sumArea = sumArea + ball_area[i];
    // std::cout << ball_area[i] << std::endl;
  }
  std::cout << "Average Ball area: " << (double)sumArea / ball_area.size() << std::endl;

  return 0;
}

void readParams()
{

  std::string color_mean, color_sd;
  if (ros::param::has("/video_path"))
  {
    ros::param::get("/video_path", video_path);
  }

  int mean[3];
  int sd[3];
  if (ros::param::has("/h_mean"))
  {
    ros::param::get("/h_mean", mean[0]);
  }
  if (ros::param::has("/s_mean"))
  {
    ros::param::get("/s_mean", mean[1]);
  }
  if (ros::param::has("/v_mean"))
  {
    ros::param::get("/v_mean", mean[2]);
  }

  if (ros::param::has("/h_sd"))
  {
    ros::param::get("/h_sd", sd[0]);
  }
  if (ros::param::has("/s_sd"))
  {
    ros::param::get("/s_sd", sd[1]);
  }
  if (ros::param::has("/v_sd"))
  {
    ros::param::get("/v_sd", sd[2]);
  }
  int i = 0;
  while (i < 3)
  {
    lower.push_back(mean[i] - sd[i]);
    upper.push_back(mean[i] + sd[i]);
    // std::cout << "lower(" << i << "): " << lower.at(i) << std::endl;
    // std::cout << "upper(" << i << "): " << upper.at(i) << std::endl;
    i++;
  }

  /*
  std::stringstream ss(color_mean);
  while (ss.good())
  {
    std::string substr;
    std::getline(ss, substr, ',');
    if (substr != ",")
      c_mean.push_back(std::stoi(substr));
  }


  std::stringstream ss2(color_sd);
  while (ss2.good())
  {
    std::string substr;
    std::getline(ss2, substr, ',');
    if (substr != ",")
      c_sd.push_back(std::stoi(substr));
  }

  int i = 0;
  while (i < c_mean.size())
  {
    std::cout << "color_mean(" << i << "): " << color_mean.at(i) << std::endl;
    std::cout << "color_sd(" << i << "): " << color_sd.at(i) << std::endl;

    lower.push_back(color_mean.at(i) - color_sd.at(i));
    upper.push_back(color_mean.at(i) + color_sd.at(i));

    std::cout << "lower(" << i << "): " << lower.at(i) << std::endl;
    std::cout << "upper(" << i << "): " << upper.at(i) << std::endl;
    i++;
  }
  */
}