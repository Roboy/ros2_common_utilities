#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <memory>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/convert.h>

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


#include "visualization_msgs/msg/interactive_marker.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <string>
#include <sys/stat.h>



// #include <chrono>
// #include <functional>

// #include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

using namespace Eigen;

struct COLOR {
    COLOR(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {};
    void randColor(){
        r = 1.0f-(rand() / ((float)(RAND_MAX)));
        g = 1.0f-(rand() / ((float)(RAND_MAX)));
        b = 1.0f-(rand() / ((float)(RAND_MAX)));
    };
    float r, g, b, a;
};

using namespace visualization_msgs;
using std::string;

class rviz_visualization {
public:
    rviz_visualization();

    ~rviz_visualization();

    static void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback);

    void initializeInteractiveMarkerServer();

    visualization_msgs::msg::Marker makeBox(visualization_msgs::msg::InteractiveMarker::SharedPtr msg);

    visualization_msgs::msg::InteractiveMarkerControl::SharedPtr makeBoxControl(visualization_msgs::msg::InteractiveMarker::SharedPtr msg);

    void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf2::Vector3 &position,
                        bool show_6dof, double scale = 1, const char *frame = "world",
                        const char *name = "interactive_marker",
                        const char *description = "for interaction and shit");

    Vector3d convertGeometryToEigen(const geometry_msgs::msg::Vector3::SharedPtr vector_in);

    geometry_msgs::msg::Vector3 convertEigenToGeometry(const Vector3d &vector_in);

    void PoseMsgToTF(const geometry_msgs::msg::Pose::SharedPtr msg, tf2::Transform &bt);

    /**
     * Publishes a mesh visualization marker
     * @param package ros package in which this mesh is located
     * @param relative_path the relative path inside that ros packae
     * @param the mesh file name
     * @param pos at this position
     * @param orientation with this orientation
     * @param modelname name of the mesh.dae
     * @param frame in this frame
     * @param ns namespace
     * @param message_id unique id
     * @param duration in seconds
     * @param color of the mesh
     */
    void publishMesh(const char *package, const char *relative_path, const char *modelname, Vector3d &pos,
                     Quaterniond &orientation,
                     double scale, const char *frame, const char *ns, int message_id, int duration = 0,
                     COLOR color = COLOR(1, 1, 1, 1), bool update = false);

    /**
     * Publishes a mesh visualization marker
     * @param package ros package in which this mesh is located
     * @param relative_path the relative path inside that ros packae
     * @param the mesh file name
     * @param pose of mesh
     * @param modelname name of the mesh.dae
     * @param frame in this frame
     * @param ns namespace
     * @param message_id unique id
     * @param duration in seconds
     * @param color of the mesh
     */
    void publishMesh(const char *package, const char *relative_path, const char *modelname, geometry_msgs::msg::Pose::SharedPtr pose,
                     double scale, const char *frame, const char *ns, int message_id, int duration = 0,
                     COLOR color = COLOR(1, 1, 1, 1), bool update = false);

    /**
     * Publishes a sphere visualization marker
     * @param pos at this positon
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void
    publishSphere(Vector3d &pos, const char *frame, const char *ns, int message_id, COLOR color, float radius = 0.01,
                  int duration = 0);

    /**
     * Publishes a sphere visualization marker
     * @param pose at this positon
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void publishSphere(geometry_msgs::msg::Pose::SharedPtr pose, const char *frame, const char *ns, int message_id, COLOR color,
                       float radius = 0.01, int duration = 0);

    /**
     * Publishes a cube visualization marker
     * @param pos at this positon
     * @param quat with this orientation
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void publishCube(Vector3d &pos, Vector4d &quat, const char *frame, const char *ns, int message_id, COLOR color,
                     float radius = 0.01, int duration = 0);

    /**
     * Publishes a cube visualization marker
     * @param pose with this pose
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void publishCube(geometry_msgs::msg::Pose::SharedPtr pose, const char *frame, const char *ns, int message_id, COLOR color,
                     float radius = 0.01, int duration = 0);

    /**
     * Publishes a cube visualization marker
     * @param pos position
     * @param quat quaternion
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param dx cube x dim
     * @param dy cube y dim
     * @param dz cube z dim
     * @param duration for this duration in seconds (0=forever)
     */
    void publishCube(Vector3d &pos, Quaternionf &quat, const char *frame, const char *ns, int message_id, COLOR color,
                     float dx = 0.01, float dy = 0.01, float dz = 0.01, int duration = 0);

    /**
     * Publishes a cylinder visualization marker
     * @param pos at this positon
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void
    publishCylinder(Vector3d &pos, const char *frame, const char *ns, int message_id, COLOR color, float radius = 0.01,
                    int duration = 0);

    /**
     * Publishes a ray visualization marker
     * @param pos at this positon
     * @param dir direction
     * @param frame in this frame
     * @param message_id a unique id
     * @param ns namespace
     * @param color rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void publishRay(Vector3d &pos, Vector3d &dir, const char *frame, const char *ns, int message_id, COLOR color,
                    int duration = 0);

    /**
     * Publishes a text message marker
     * @param pos at this positon
     * @param text with this text
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param color rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     * @param height height of the text
     */
    void publishText(Vector3d &pos, const char *text, const char *frame, const char *ns, int message_id, COLOR color,
                     int duration, float height);

    /**
     * Clears a marker with this id
     */
    void clearMarker(int id);

    /**
     * Clears all markers in rviz
     */
    void clearAll();

    /**
     * Gets a tf transform
     * @param from source frame
     * @param to target frame
     * @param transform will be filled with the transform
     * @return success
     */
    bool getTransform(string from, string to, geometry_msgs::msg::Pose::SharedPtr transform);

    /**
     * Gets a tf transform
     * @param from source frame
     * @param to target frame
     * @param transform will be filled with the transform
     * @return success
     */
    bool getTransform(string from, string to, Matrix4d &transform);

    /**
     * Queries the tf listener for the specified transform
     * @param lighthouse
     * @param to another frame
     * @param transform the transform if available
     * @return true if available
     */
    bool getLighthouseTransform(bool lighthouse, const char *to, Matrix4d &transform);

    /**
     * Queries the tf listener for the specified transform
     * @param lighthouse
     * @param from another frame
     * @param transform the transform if available
     * @return true if available
     */
    bool getLighthouseTransform(const char *from, bool lighthouse, Matrix4d &transform);

    /**
     * Queries the tf listener for the specified transform
     * @param to this frame
     * @param from another frame
     * @param transform the transform if available
     * @return true if available
     */
    bool getTransform(const char *from, const char *to, tf2::Transform &transform);

    /**
     * Publishes a tf transform
     * @param from source frame
     * @param to target frame
     * @param transform
     * @return success
     */
    bool publishTransform(string from, string to, geometry_msgs::msg::Pose::SharedPtr transform);


    /**
     * transforms a TransformStamped msg to a Eigen::Affine3d object
     * @param transformStamped input TransformStamped msg 
     * @param eigen_transform output Eigen::Affine3d object
     */
    void transformStampedToEigen(const geometry_msgs::msg::TransformStamped transformStamped, Eigen::Affine3d& eigen_transform);


private:
    rclcpp::Node::SharedPtr node_; // ros::NodeHandlePtr nh;
    static std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server;
    static interactive_markers::MenuHandler menu_handler;
    static bool first;
    visualization_msgs::msg::MarkerArray marker_array;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener; // tf2::TransformListener listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster; // tf2::TransformBroadcaster broadcaster; = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
public:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualization_pub;            // ros::Publisher visualization_pub
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_array_pub; // ros::Publisher visualization_array_pub
    
    bool publish_as_marker_array = false;
    size_t number_of_markers_to_publish_at_once = 100;
};