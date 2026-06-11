#pragma once

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

namespace aesk_camera_driver
{
class CameraDriver
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    CameraDriver(ros::NodeHandle& t_node_handle);

  private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * ROS message filter callback method.
     * @param t_image the received Image message.
     */
    void topicCallback(const sensor_msgs::ImageConstPtr& t_image);

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! Camera topic name.
    std::string m_camera_topic;

    //! Topic name for publisher.
    std::string m_pub_topic;

    //! Frame id for image which will publish.
    std::string m_frame_id;

    //! Intrinsic parameters.
    std::vector<double> m_intrinsic_parameters;

    //! Distortion coefficients.
    std::vector<double> m_distortion_coefficients;

    //! Extrinsic parameters.
    std::vector<double> m_extrinsic_parameters;

    //! Intrinsic matrix.
    cv::Mat m_intrinsic_matrix;

    image_transport::ImageTransport m_image_transport;

    image_transport::Subscriber m_image_subscriber;

    image_transport::Publisher m_image_publisher;

    //! Projected point cloud publisher.
    ros::Publisher m_point_cloud_pub;

    //! Message filter subscriber for Image.
    // ImageSubscriber m_image_subscriber;
};

}  // namespace aesk_camera_driver
