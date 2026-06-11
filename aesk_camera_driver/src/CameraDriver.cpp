#include "aesk_camera_driver/CameraDriver.hpp"

#include <cv_bridge/cv_bridge.h>

#include <tf2/LinearMath/Matrix3x3.h>

namespace aesk_camera_driver
{
CameraDriver::CameraDriver(ros::NodeHandle& t_node_handle)
  : m_node_handle(t_node_handle), m_image_transport(m_node_handle)
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("Camera topic: : %s", m_camera_topic.c_str());
    ROS_INFO("Publisher topic: : %s", m_pub_topic.c_str());

    double intrinsic_params[9];
    for (int i = 0; i < 9; i++)
    {
        intrinsic_params[i] = m_intrinsic_parameters[i];
    }
    cv::Mat(3, 3, CV_64F, &intrinsic_params).copyTo(m_intrinsic_matrix);

    ROS_INFO("Intrinsic matrix:");
    std::cout << m_intrinsic_matrix << '\n';

    ROS_INFO("Distortion coefficients:");
    for (auto coeff : m_distortion_coefficients)
    {
        std::cout << coeff << " ";
    }
    std::cout << '\n';

    m_image_subscriber =
        m_image_transport.subscribe(m_camera_topic, 1, &CameraDriver::topicCallback, this);

    m_image_publisher = m_image_transport.advertise(m_pub_topic, 1);

    ROS_INFO("Successfully launched node.");
}

bool CameraDriver::readParameters()
{
    if (!m_node_handle.getParam("camera_topic", m_camera_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("frame_id", m_frame_id))
    {
        return false;
    }
    if (!m_node_handle.getParam("publisher_topic", m_pub_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("camera_matrix/data", m_intrinsic_parameters))
    {
        return false;
    }
    if (!m_node_handle.getParam("distortion_coefficients/data",
                                m_distortion_coefficients))
    {
        return false;
    }
    return true;
}

void CameraDriver::topicCallback(const sensor_msgs::ImageConstPtr& t_image)
{
    // Convert ROS image message to OpenCV image via cv_bridge.
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(t_image, sensor_msgs::image_encodings::BGR8);

    cv::Mat undistorted_image;
    cv::undistort(cv_ptr->image, undistorted_image, m_intrinsic_matrix, m_distortion_coefficients);

    sensor_msgs::ImagePtr image_message =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistorted_image).toImageMsg();
    image_message->header.frame_id = m_frame_id;
    image_message->header.stamp = t_image->header.stamp;

    m_image_publisher.publish(image_message);

    // cv::Mat outImg;
    // cv::resize(undistorted_image, outImg, cv::Size(), 0.5, 0.5);
    // cv::imshow("Image", outImg);
    // cv::waitKey(10);
}
}  // namespace aesk_camera_driver