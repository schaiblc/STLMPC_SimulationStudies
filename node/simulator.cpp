#include <ros/ros.h>
#include <cstdlib> //Revision (R4): rand() for per-beam dropout

// interactive marker
#include <interactive_markers/interactive_marker_server.h>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

#include "f1tenth_simulator/pose_2d.hpp"
#include "f1tenth_simulator/ackermann_kinematics.hpp"
#include "f1tenth_simulator/scan_simulator_2d.hpp"

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/car_params.hpp"
#include "f1tenth_simulator/ks_kinematics.hpp"
#include "f1tenth_simulator/st_kinematics.hpp"
#include "f1tenth_simulator/precompute.hpp"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/tf.h>
#include <iostream>
#include <cstdio>
#include <math.h>
#include <utility>
#include <fstream>
#include <vector>


using namespace racecar_simulator;

class RacecarSimulator {
private:
    // A ROS node
    ros::NodeHandle n;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* move_base_client;

    // The transformation frames used
    std::string map_frame, base_frame, scan_frame;

    // obstacle states (1D index) and parameters
    std::vector<int> added_obs;
    // listen for clicked point for adding obstacles
    ros::Subscriber obs_sub;
    int obstacle_size;

    // interactive markers' server
    interactive_markers::InteractiveMarkerServer im_server;

    // The car state and parameters
    CarState state;
    Pose2D state_det; //State of the other vehicle detected
    double detx=0;
    double dety=0;
    double dettheta=0;

    // Revision (B5): parameterized scripted adversary + sensor degradation.
    int adv_enable=0;               // move state_det along a scripted trajectory
    double adv_speed=1.3;           // current adversary speed (mutated by "brake")
    double adv_speed0=1.3;          // nominal adversary speed
    std::string adv_maneuver="straight";
    double adv_brake_decel=2.5, adv_brake_start=4.0;
    double adv_swerve_rate=0.3, adv_swerve_start=4.0, adv_swerve_dur=1.0;
    int adv2_enable=0;              // R3: second (occluding) adversary, scan-baked
    Pose2D state_det2;
    double adv2_speed=1.3;
    double veh_det_length=0.5, veh_det_width=0.4; // adversary footprint for scan-baking
    double beam_dropout_prob=0.0;   // R4: per-beam dropout probability

    // Per-map start poses (single source of truth, from config/map_starts.yaml).
    // The ego and adversaries are placed here at construction and snapped back to
    // them the instant nav is enabled, so every run starts identically.
    double adv_init_x=2.0, adv_init_y=0.4, adv_init_theta=0.1;    // adversary spawn (map frame)
    double adv2_init_x=3.0, adv2_init_y=0.4, adv2_init_theta=0.1; // second adversary spawn (map frame)
    double ego_init_x=0.0, ego_init_y=0.0, ego_init_theta=0.0;   // ego spawn
    // Adversary placement relative to the ego (heading-invariant): +forward along the
    // ego heading, +left to port, rel_theta = heading offset. Defaults chosen per
    // maneuver in the constructor. adv_relative=0 falls back to absolute adv_init_*.
    int adv_relative=1;
    double adv_forward=4.5, adv_left=0.0, adv_rel_theta=0.0;
    double adv2_forward=5.0, adv2_left=2.9, adv2_rel_theta=-1.5707963;
    std::string map_name="map1"; // selects the block in map_starts.yaml
    int nav_mux_idx=4;           // mux slot that signals autonomous nav is active
    int nav_started=0;           // latched true once nav is first enabled
    ros::Subscriber mux_sub;     // watch the mux to anchor the adversary clock to nav-enable

    // Course-completion goal (from map_starts.yaml) + run termination. The run ends
    // (success) when the ego reaches the goal after first leaving it by goal_arm_dist,
    // or (timeout) when run_timeout elapses after nav-enable.
    double goal_x=0.0, goal_y=0.0, goal_radius=1.0;
    double goal_arm_dist=2.0;    // ego must get this far from goal before a reach counts
    double run_timeout=60.0;     // s after nav-enable before a run is declared timed-out
    int shutdown_on_goal=0;      // 1 => ros::shutdown() when the run ends (batch mode)
    int goal_armed=0, goal_reached=0, run_over=0;
    ros::Publisher run_complete_pub; // latched std_msgs/Bool: true=completed, false=timeout
    ros::Timer shutdown_timer;   // brief grace before shutdown so logs flush
    ros::Publisher goal_marker_pub; // rviz: goal sphere + radius ring + label
    int goal_marker_tick=0;      // throttle goal-marker republishing

    double previous_seconds;
    double scan_distance_to_base_link;
    double max_speed, max_steering_angle;
    double max_accel, max_steering_vel, max_decel;
    double desired_speed, desired_steer_ang;
    double accel, steer_angle_vel;
    CarParams params;
    double width;
    double update_pose_rate=1;

    double start_time=0;

    // A simulator of the laser
    ScanSimulator2D scan_simulator;
    double map_free_threshold;

    // For publishing transformations
    tf2_ros::TransformBroadcaster br;

    // A timer to update the pose
    ros::Timer update_pose_timer;

    ros::Timer waypoint_timer;

    // Listen for drive commands
    ros::Subscriber drive_sub;

    // Listen for drive commands (from ROS explore package)
    ros::Subscriber explore_sub;

    // Listen for a map
    ros::Subscriber map_sub;
    bool map_exists = false;

    // Listen for updates to the pose
    ros::Subscriber pose_sub;
    ros::Subscriber pose_rviz_sub;
    ros::Subscriber tf_sub;

    // Publish a scan, odometry, and imu data
    bool broadcast_transform;
    bool pub_gt_pose;
    ros::Publisher scan_pub;
    ros::Publisher pose_pub;
    ros::Publisher odom_pub;
    ros::Publisher imu_pub;

    // publisher for map with obstacles
    ros::Publisher map_pub;

    ros::Publisher waypoint_pub;
    double waypoint_distance;
    int waypoint_count;
    int use_manual_fwd=0;

    // keep an original map for obstacles
    nav_msgs::OccupancyGrid original_map;
    nav_msgs::OccupancyGrid current_map;

    // for obstacle collision
    int map_width, map_height;
    double map_resolution, origin_x, origin_y;


    std::vector<std::pair<double, double>> MPC_track;

    int indx=0;

    // safety margin for collisions
    double thresh;
    double speed_clip_diff;

    // precompute cosines of scan angles
    std::vector<double> cosines;

    // scan parameters
    double scan_fov;
    double scan_ang_incr;

    // pi
    const double PI = 3.1415;

    // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    double lidar_call=0;

    // for collision check
    bool TTC = false;
    double ttc_threshold;

    // steering delay
    int buffer_length;
    std::vector<double> steering_buffer;


public:

    RacecarSimulator(): im_server("racecar_sim") {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        move_base_client = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
        move_base_client->waitForServer(ros::Duration(5.0));

        // Initialize car state and driving commands
        // state = {.x=0, .y=0, .theta=0, .velocity=0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
        // state_det.x=15; state_det.y=6.2; state_det.theta=0; //columbia
        // state_det.x=2; state_det.y=-0.2; state_det.theta=0; //berlin
        // Revision (B5): parameterized adversary + sensor-degradation params.
        n.param("adv_enable", adv_enable, 0);

        // Per-map start poses (single source of truth). map_starts.yaml is loaded
        // into this node's namespace by campaign.launch; the block for the current
        // map_name supplies both the ego spawn and the adversary spawn(s). These are
        // the values used throughout the run (and the ego + adversaries are reset to
        // them the instant nav is enabled, see mux_callback). Any pose can still be
        // overridden per-run with an explicit *_init_* private param.
        n.param<std::string>("map_name", map_name, std::string("map1"));
        std::string mb = "map_starts/" + map_name + "/";
        // fall back to the legacy levinelobby (map1) values if the map is not tabled
        n.param(mb+"ego_x",      ego_init_x,      0.0);
        n.param(mb+"ego_y",      ego_init_y,      0.0);
        n.param(mb+"ego_theta",  ego_init_theta,  0.0);
        n.param(mb+"adv_x",      adv_init_x,      2.0);
        n.param(mb+"adv_y",      adv_init_y,      -0.4);
        n.param(mb+"adv_theta",  adv_init_theta,  -0.1);
        n.param(mb+"adv2_x",     adv2_init_x,     3.0);
        n.param(mb+"adv2_y",     adv2_init_y,     -0.4);
        n.param(mb+"adv2_theta", adv2_init_theta, -0.1);
        n.param(mb+"goal_x",     goal_x,          ego_init_x);
        n.param(mb+"goal_y",     goal_y,          ego_init_y);
        n.param(mb+"goal_radius",goal_radius,     1.0);
        // explicit per-run overrides still win over the table
        n.param("ego_init_x",     ego_init_x,      ego_init_x);
        n.param("ego_init_y",     ego_init_y,      ego_init_y);
        n.param("ego_init_theta", ego_init_theta,  ego_init_theta);
        n.param("adv_init_x",     adv_init_x,      adv_init_x);
        n.param("adv_init_y",     adv_init_y,      adv_init_y);
        n.param("adv_init_theta", adv_init_theta,  adv_init_theta);
        n.param("adv2_init_x",    adv2_init_x,     adv2_init_x);
        n.param("adv2_init_y",    adv2_init_y,     adv2_init_y);
        n.param("adv2_init_theta",adv2_init_theta, adv2_init_theta);

        n.param("adv_speed", adv_speed0, 1.7); adv_speed=adv_speed0;
        n.param<std::string>("adv_maneuver", adv_maneuver, std::string("straight"));
        n.param("adv_brake_decel", adv_brake_decel, 2.5);
        n.param("adv_brake_start", adv_brake_start, 4.0);
        n.param("adv_swerve_rate", adv_swerve_rate, 0.3);
        n.param("adv_swerve_start", adv_swerve_start, 4.0);
        n.param("adv_swerve_dur", adv_swerve_dur, 1.0);
        n.param("adv2_enable", adv2_enable, 0);
        n.param("adv2_speed", adv2_speed, 1.5);
        n.param("beam_dropout_prob", beam_dropout_prob, 0.0);
        n.param("veh_det_length", veh_det_length, 0.5);
        n.param("veh_det_width", veh_det_width, 0.4);
        n.param("nav_mux_idx", nav_mux_idx, 4);

        // Course-completion goal + run termination. goal defaults to the ego spawn
        // (return-to-start = one lap) unless the map block overrides it.
        n.param("goal_x", goal_x, goal_x);
        n.param("goal_y", goal_y, goal_y);
        n.param("goal_radius", goal_radius, goal_radius);
        n.param("goal_arm_dist", goal_arm_dist, 2.0);
        n.param("run_timeout", run_timeout, 60.0);
        n.param("shutdown_on_goal", shutdown_on_goal, 0);

        // Per-seed randomized start (paper: +/-0.3 m, +/-10 deg), applied on top of
        // the table pose so map_starts.yaml stays the single source of truth.
        double jx=0.0, jy=0.0, jt=0.0;
        n.param("ego_jitter_x", jx, 0.0);
        n.param("ego_jitter_y", jy, 0.0);
        n.param("ego_jitter_theta", jt, 0.0);
        ego_init_x+=jx; ego_init_y+=jy; ego_init_theta+=jt;

        // Adversary placement. By default the adversary(ies) are positioned RELATIVE
        // to the ego (forward/left/heading offset), so the encounter geometry stays
        // correct on any map and for any ego heading. Geometry defaults are chosen
        // per maneuver:
        //   brake  -> lead into the right-hand bend: front-right (~3.5 m ahead, 2 m to
        //             starboard), heading angled right, that brakes to a stall in path
        //   swerve -> ~90 deg crosser from port (~4 m ahead, 2.5 m to the left)
        //   straight/occluded -> crosser(s); R3's second vehicle sits just behind the
        //                        first along the ego's line of sight so it is occluded
        // Set adv_relative:=0 to use absolute adv_init_* map coordinates instead, and
        // adv_forward/adv_left/adv_rel_theta (and adv2_*) to override the geometry.
        n.param("adv_relative", adv_relative, 1);
        const double UNSET=-999.0;
        n.param("adv_forward",    adv_forward,    UNSET);
        n.param("adv_left",       adv_left,       UNSET);
        n.param("adv_rel_theta",  adv_rel_theta,  UNSET);
        n.param("adv2_forward",   adv2_forward,   UNSET);
        n.param("adv2_left",      adv2_left,      UNSET);
        n.param("adv2_rel_theta", adv2_rel_theta, UNSET);
        double df=4.0, dl=2.5, drt=-M_PI/2;          // straight/occluded crosser default
        if(adv_maneuver=="brake"){ df=3.5; dl=-1.0; drt=-1.2; }     // lead into the right-hand bend (front-right, angled right)
        else if(adv_maneuver=="swerve"){ df=4.0; dl=2.5; drt=-M_PI/2; } // port crosser
        if(adv_forward==UNSET)    adv_forward=df;
        if(adv_left==UNSET)       adv_left=dl;
        if(adv_rel_theta==UNSET)  adv_rel_theta=drt;
        if(adv2_forward==UNSET)   adv2_forward=5.0;
        if(adv2_left==UNSET)      adv2_left=2.9;
        if(adv2_rel_theta==UNSET) adv2_rel_theta=-M_PI/2;
        if(adv_relative){
            double th=ego_init_theta;
            adv_init_x = ego_init_x + adv_forward*std::cos(th) - adv_left*std::sin(th);
            adv_init_y = ego_init_y + adv_forward*std::sin(th) + adv_left*std::cos(th);
            adv_init_theta = th + adv_rel_theta;
            adv2_init_x = ego_init_x + adv2_forward*std::cos(th) - adv2_left*std::sin(th);
            adv2_init_y = ego_init_y + adv2_forward*std::sin(th) + adv2_left*std::cos(th);
            adv2_init_theta = th + adv2_rel_theta;
        }

        // Place the ego and adversaries at their defined start poses.
        state.x=ego_init_x; state.y=ego_init_y; state.theta=ego_init_theta;
        state.velocity=0.0; state.steer_angle=0.0; state.angular_velocity=0.0;
        state.slip_angle=0.0; state.st_dyn=false;
        state_det.x=adv_init_x; state_det.y=adv_init_y; state_det.theta=adv_init_theta;
        state_det2.x=adv2_init_x; state_det2.y=adv2_init_y; state_det2.theta=adv2_init_theta;

        start_time=ros::Time::now().toSec();
        accel = 0.0;
        steer_angle_vel = 0.0;
        desired_speed = 0.0;
        desired_steer_ang = 0.0;
        previous_seconds = ros::Time::now().toSec();


        // Get the topic names
        std::string drive_topic, map_topic, scan_topic, pose_topic, gt_pose_topic, explore_topic, 
        pose_rviz_topic, odom_topic, imu_topic;
        n.getParam("drive_topic", drive_topic);
        n.getParam("map_topic", map_topic);
        n.getParam("scan_topic", scan_topic);
        n.getParam("pose_topic", pose_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("pose_rviz_topic", pose_rviz_topic);
        n.getParam("imu_topic", imu_topic);
        n.getParam("ground_truth_pose_topic", gt_pose_topic);
        explore_topic="/cmd_vel";
        n.getParam("use_manual_fwd",use_manual_fwd);

        // Get steering delay params
        n.getParam("buffer_length", buffer_length);

        // Get the transformation frame names
        n.getParam("map_frame", map_frame);
        n.getParam("base_frame", base_frame);
        n.getParam("scan_frame", scan_frame);

        // Fetch the car parameters
        int scan_beams;
        double scan_std_dev;
        n.getParam("wheelbase", params.wheelbase);
        n.getParam("update_pose_rate", update_pose_rate);
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_field_of_view", scan_fov);
        n.getParam("scan_std_dev", scan_std_dev);
        n.getParam("map_free_threshold", map_free_threshold);
        n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);
        n.getParam("max_accel", max_accel);
        n.getParam("max_decel", max_decel);
        n.getParam("max_steering_vel", max_steering_vel);
        n.getParam("friction_coeff", params.friction_coeff);
        n.getParam("height_cg", params.h_cg);
        n.getParam("l_cg2rear", params.l_r);
        n.getParam("l_cg2front", params.l_f);
        n.getParam("C_S_front", params.cs_f);
        n.getParam("C_S_rear", params.cs_r);
        n.getParam("moment_inertia", params.I_z);
        n.getParam("mass", params.mass);
        n.getParam("width", width);

        n.getParam("waypoint_distance", waypoint_distance);
        n.getParam("waypoint_count", waypoint_count);

        // clip velocity
        n.getParam("speed_clip_diff", speed_clip_diff);

        // Determine if we should broadcast
        n.getParam("broadcast_transform", broadcast_transform);
        n.getParam("publish_ground_truth_pose", pub_gt_pose);

        // Get obstacle size parameter
        n.getParam("obstacle_size", obstacle_size);

        // Initialize a simulator of the laser scanner
        scan_simulator = ScanSimulator2D(
            scan_beams,
            scan_fov,
            scan_std_dev);

        // Make a publisher for laser scan messages
        scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_topic, 1);

        // Make a publisher for odometry messages
        odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 1);

        // Make a publisher for IMU messages
        imu_pub = n.advertise<sensor_msgs::Imu>(imu_topic, 1);

        // Make a publisher for publishing map with obstacles
        map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);

        // Make a publisher for ground truth pose
        pose_pub = n.advertise<geometry_msgs::PoseStamped>(gt_pose_topic, 1);

        waypoint_pub = n.advertise<nav_msgs::Path>("/waypoints", 1);

        // Latched run-outcome flag (true=course completed, false=timed out).
        run_complete_pub = n.advertise<std_msgs::Bool>("/run_complete", 1, true);

        // Latched goal visualization (sphere + radius ring + label) for rviz, so the
        // per-map goal can be eyeballed before running.
        goal_marker_pub = n.advertise<visualization_msgs::MarkerArray>("/goal_marker", 1, true);

        // Start a timer to output the pose
        update_pose_timer = n.createTimer(ros::Duration(update_pose_rate), &RacecarSimulator::update_pose, this);

        waypoint_timer = n.createTimer(ros::Duration(0.1), &RacecarSimulator::generateWaypoints, this);

        // Start a subscriber to listen to drive commands
        drive_sub = n.subscribe(drive_topic, 1, &RacecarSimulator::drive_callback, this);

        // Start a subscriber to listen to explore commands
        explore_sub = n.subscribe(explore_topic, 1, &RacecarSimulator::explore_callback, this);

        // Start a subscriber to listen to new maps
        map_sub = n.subscribe(map_topic, 1, &RacecarSimulator::map_callback, this);

        // Start a subscriber to listen to pose messages
        pose_sub = n.subscribe(pose_topic, 1, &RacecarSimulator::pose_callback, this);
        pose_rviz_sub = n.subscribe(pose_rviz_topic, 1, &RacecarSimulator::pose_rviz_callback, this);

        // obstacle subscriber
        obs_sub = n.subscribe("/clicked_point", 1, &RacecarSimulator::obs_callback, this);

        tf_sub = n.subscribe("/tf", 20, &RacecarSimulator::tf_callback, this);

        // Watch the mux so the scripted-adversary timeline can be anchored to the
        // moment navigation is enabled (see mux_callback).
        std::string mux_topic;
        n.getParam("mux_topic", mux_topic);
        mux_sub = n.subscribe(mux_topic, 1, &RacecarSimulator::mux_callback, this);

        // get collision safety margin
        n.getParam("coll_threshold", thresh);
        n.getParam("ttc_threshold", ttc_threshold);

        scan_ang_incr = scan_simulator.get_angle_increment();

        cosines = Precompute::get_cosines(scan_beams, -scan_fov/2.0, scan_ang_incr);
        car_distances = Precompute::get_car_distances(scan_beams, params.wheelbase, width, 
                scan_distance_to_base_link, -scan_fov/2.0, scan_ang_incr);


        // steering delay buffer
        steering_buffer = std::vector<double>(buffer_length);

        // OBSTACLE BUTTON:
        // wait for one map message to get the map data array
        boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
        nav_msgs::OccupancyGrid map_msg;
        map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
        if (map_ptr != NULL) {
            map_msg = *map_ptr;
        }
        original_map = map_msg;
        current_map = map_msg;
        std::vector<int8_t> map_data_raw = map_msg.data;
        std::vector<int> map_data(map_data_raw.begin(), map_data_raw.end());

        map_width = map_msg.info.width;
        map_height = map_msg.info.height;
        origin_x = map_msg.info.origin.position.x;
        origin_y = map_msg.info.origin.position.y;
        map_resolution = map_msg.info.resolution;

        // create button for clearing obstacles
        visualization_msgs::InteractiveMarker clear_obs_button;
        clear_obs_button.header.frame_id = "map";
        // clear_obs_button.pose.position.x = origin_x+(1/3)*map_width*map_resolution;
        // clear_obs_button.pose.position.y = origin_y+(1/3)*map_height*map_resolution;
        // TODO: find better positioning of buttons
        clear_obs_button.pose.position.x = 0;
        clear_obs_button.pose.position.y = -5;
        clear_obs_button.scale = 1;
        clear_obs_button.name = "clear_obstacles";
        clear_obs_button.description = "Clear Obstacles\n(Left Click)";
        visualization_msgs::InteractiveMarkerControl clear_obs_control;
        clear_obs_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        clear_obs_control.name = "clear_obstacles_control";
        // make a box for the button
        visualization_msgs::Marker clear_obs_marker;
        clear_obs_marker.type = visualization_msgs::Marker::CUBE;
        clear_obs_marker.scale.x = clear_obs_button.scale*0.45;
        clear_obs_marker.scale.y = clear_obs_button.scale*0.65;
        clear_obs_marker.scale.z = clear_obs_button.scale*0.45;
        clear_obs_marker.color.r = 0.0;
        clear_obs_marker.color.g = 1.0;
        clear_obs_marker.color.b = 0.0;
        clear_obs_marker.color.a = 1.0;

        clear_obs_control.markers.push_back(clear_obs_marker);
        clear_obs_control.always_visible = true;
        clear_obs_button.controls.push_back(clear_obs_control);

        im_server.insert(clear_obs_button);
        im_server.setCallback(clear_obs_button.name, boost::bind(&RacecarSimulator::clear_obstacles, this, _1));

        im_server.applyChanges();

        publish_goal_marker(); // latched, so rviz shows the goal immediately

        ROS_INFO("Simulator constructed.");
    }

    void update_pose(const ros::TimerEvent&) {

        // simulate P controller
        compute_accel(desired_speed);
        double actual_ang = 0.0;
        if (steering_buffer.size() < buffer_length) {
            steering_buffer.push_back(desired_steer_ang);
            actual_ang = 0.0;
        } else {
            steering_buffer.insert(steering_buffer.begin(), desired_steer_ang);
            actual_ang = steering_buffer.back();
            steering_buffer.pop_back();
        }
        set_steer_angle_vel(compute_steer_vel(actual_ang));

        // Update the pose
        ros::Time timestamp = ros::Time::now();
        double current_seconds = timestamp.toSec();

        state = STKinematics::update(
            state,
            accel,
            steer_angle_vel,
            params,
            current_seconds - previous_seconds);
                


        state.velocity = std::min(std::max(state.velocity, -max_speed), max_speed);
        state.steer_angle = std::min(std::max(state.steer_angle, -max_steering_angle), max_steering_angle);
        
        previous_seconds = current_seconds;

        /// Publish the pose as a transformation
        pub_pose_transform(timestamp);

        /// Publish the steering angle as a transformation so the wheels move
        pub_steer_ang_transform(timestamp);

        // Make an odom message as well and publish it
        pub_odom(timestamp);

        // TODO: make and publish IMU message
        pub_imu(timestamp);

        //Update state of vehicle detected and publish
        //////////////////////////////////////////////
        double myvel=1.7;
        double timeoffset=0;

        //columbia map simulated trajectory for detected vehicle
        // if(ros::Time::now().toSec()<start_time+60+timeoffset && ros::Time::now().toSec()>start_time+timeoffset){
        //     state_det.x+=myvel*update_pose_rate*cos(state_det.theta);
        //     state_det.y+=myvel*update_pose_rate*sin(state_det.theta);
        //     if(ros::Time::now().toSec()>start_time+3+timeoffset && ros::Time::now().toSec()<start_time+100+timeoffset){
        //         state_det.theta+=0.2*update_pose_rate;
        //     }
        //     // if(ros::Time::now().toSec()>start_time+10+timeoffset && ros::Time::now().toSec()<start_time+12+timeoffset){
        //     //     state_det.theta-=0.2*update_pose_rate;
        //     // }
            
        // }

        //berlin map simulated trajectory for detected vehicle
        // if(ros::Time::now().toSec()<start_time+60+timeoffset && ros::Time::now().toSec()>start_time+timeoffset){
        //     state_det.x+=myvel*update_pose_rate*cos(state_det.theta);
        //     state_det.y+=myvel*update_pose_rate*sin(state_det.theta);
        //     if(ros::Time::now().toSec()>start_time+3+timeoffset && ros::Time::now().toSec()<start_time+10+timeoffset){
        //         state_det.theta-=0.2*update_pose_rate;
        //     }
        //     if(ros::Time::now().toSec()>start_time+10+timeoffset && ros::Time::now().toSec()<start_time+12+timeoffset){
        //         state_det.theta+=0.2*update_pose_rate;
        //     }
        //     if(ros::Time::now().toSec()>start_time+12+timeoffset && ros::Time::now().toSec()<start_time+16+timeoffset){
        //         state_det.theta-=0.3*update_pose_rate;
        //     }
        //     if(ros::Time::now().toSec()>start_time+16+timeoffset && ros::Time::now().toSec()<start_time+20+timeoffset){
        //         state_det.theta+=0.1*update_pose_rate;
        //     }
        //     if(ros::Time::now().toSec()>start_time+26+timeoffset && ros::Time::now().toSec()<start_time+30+timeoffset){
        //         state_det.theta+=0.1*update_pose_rate;
        //     }
        //     if(ros::Time::now().toSec()>start_time+32+timeoffset && ros::Time::now().toSec()<start_time+35+timeoffset){
        //         state_det.theta-=0.2*update_pose_rate;
        //     }
        //     if(ros::Time::now().toSec()>start_time+35+timeoffset && ros::Time::now().toSec()<start_time+42+timeoffset){
        //         state_det.theta-=0.4*update_pose_rate;
        //     }
        //     if(ros::Time::now().toSec()>start_time+42+timeoffset && ros::Time::now().toSec()<start_time+42.5+timeoffset){
        //         state_det.theta+=0.2*update_pose_rate;
        //     }

        // }


        // Revision (B5): parameterized scripted adversary. When adv_enable=1 the
        // adversary follows one of the maneuvers below (straight/brake/swerve),
        // deliberately violating the constant-velocity/curvature prediction model
        // to probe robustness. When adv_enable=0 the legacy levinelobby demo
        // trajectory is retained so existing sim setups are unchanged.
        if(adv_enable && nav_started){
            double tnow=ros::Time::now().toSec()-start_time;
            // Brake maneuver (R1): decelerate after adv_brake_start.
            if(adv_maneuver=="brake" && tnow>adv_brake_start){
                adv_speed=std::max(0.0, adv_speed-adv_brake_decel*update_pose_rate);
            }
            state_det.x+=adv_speed*update_pose_rate*cos(state_det.theta);
            state_det.y+=adv_speed*update_pose_rate*sin(state_det.theta);
            // Swerve maneuver (R2): apply a yaw-rate impulse during the swerve window.
            if(adv_maneuver=="swerve" && tnow>adv_swerve_start && tnow<adv_swerve_start+adv_swerve_dur){
                state_det.theta+=adv_swerve_rate*update_pose_rate;
            }
            // Second adversary (R3), straight-line; affects ego only when scan-baked.
            if(adv2_enable){
                state_det2.x+=adv2_speed*update_pose_rate*cos(state_det2.theta);
                state_det2.y+=adv2_speed*update_pose_rate*sin(state_det2.theta);
            }
        }
        else if(nav_started && ros::Time::now().toSec()<start_time+60+timeoffset && ros::Time::now().toSec()>start_time+timeoffset){
            //levinelobby map simulated trajectory for detected vehicle (legacy demo)
            state_det.x+=myvel*update_pose_rate*cos(state_det.theta);
            state_det.y+=myvel*update_pose_rate*sin(state_det.theta);
            if(ros::Time::now().toSec()>start_time+1+timeoffset && ros::Time::now().toSec()<start_time+2+timeoffset){
                state_det.theta+=0.2*update_pose_rate;
            }
            if(ros::Time::now().toSec()>start_time+5+timeoffset && ros::Time::now().toSec()<start_time+8+timeoffset){
                state_det.theta-=0.35*update_pose_rate;
            }
            if(ros::Time::now().toSec()>start_time+8+timeoffset && ros::Time::now().toSec()<start_time+11+timeoffset){
                state_det.theta-=0.2*update_pose_rate;
            }
            if(ros::Time::now().toSec()>start_time+11+timeoffset && ros::Time::now().toSec()<start_time+13+timeoffset){
                state_det.theta+=0.15*update_pose_rate;
            }
            if(ros::Time::now().toSec()>start_time+13.5+timeoffset && ros::Time::now().toSec()<start_time+23+timeoffset){
                state_det.theta+=0.35*update_pose_rate;
            }
            if(ros::Time::now().toSec()>start_time+23+timeoffset && ros::Time::now().toSec()<start_time+25+timeoffset){
                state_det.theta-=0.15*update_pose_rate;
            }
        }

        pub_pose_det_transform(timestamp);

        // Revision: course-completion / timeout check (see check_completion).
        check_completion();


        //////////////////////////////////////////////


        /// KEEP in sim
        // If we have a map, perform a scan
        lidar_call++; //This is hard-coded to ensure LIDAR callback frequency is 10 Hz, reflects physical experiment
        if(update_pose_rate!=0.01){
            ROS_INFO("Set update_pose_rate to 0.01 to ensure smooth pose update and fixed LIDAR callback fo 10 Hz");
        }
        if (map_exists && lidar_call==10) {
            lidar_call=0;
            indx++;
            // Get the pose of the lidar, given the pose of base link
            // (base link is the center of the rear axle)
            Pose2D scan_pose;
            scan_pose.x = state.x + scan_distance_to_base_link * std::cos(state.theta);
            scan_pose.y = state.y + scan_distance_to_base_link * std::sin(state.theta);
            scan_pose.theta = state.theta;

            // Compute the scan from the lidar
            std::vector<double> scan = scan_simulator.scan(scan_pose);

            // Convert to float
            std::vector<float> scan_(scan.size());
            for (size_t i = 0; i < scan.size(); i++){
                scan_[i] = scan[i];
            }

            //For simulating the static detections of another vehicle via the box outline's corners

            // std::vector<std::pair<float, float>> positions = {
            //     {detx+0.25*cos(dettheta)+0.2*sin(dettheta), dety+0.25*sin(dettheta)-0.2*cos(dettheta)},
            //     {detx+0.25*cos(dettheta)-0.2*sin(dettheta), dety+0.25*sin(dettheta)+0.2*cos(dettheta)},
            //     {detx,dety},
            //     {detx-0.25*cos(dettheta)-0.2*sin(dettheta), dety-0.25*sin(dettheta)+0.2*cos(dettheta)},
            //     {detx-0.25*cos(dettheta)+0.2*sin(dettheta), dety-0.25*sin(dettheta)-0.2*cos(dettheta)}
            // };
            
            // float angle_min = -M_PI;
            // float angle_max = M_PI;
            // size_t num_rays = scan.size();
            // float angle_increment = (angle_max - angle_min) / num_rays;

            // for (const auto& pos : positions) {
            //     float x = pos.first;
            //     float y = pos.second;

            //     float angle = std::atan2(y, x);  // angle from sensor to point
            //     float distance2 = std::sqrt(x * x + y * y);

            //     // Compute corresponding index
            //     int index = static_cast<int>(std::round((angle - angle_min) / angle_increment));

            //     // Clamp index to valid range
            //     if (index >= 0 && index < static_cast<int>(scan_.size())) {
            //         scan_[index] = distance2;
            //     }
            // }


            // TTC Calculations are done here so the car can be halted in the simulator:
            // to reset TTC
            bool no_collision = true;
            if (state.velocity != 0) {
                for (size_t i = 0; i < scan_.size(); i++) {
                    // TTC calculations

                    // calculate projected velocity
                    double proj_velocity = state.velocity * cosines[i];
                    double ttc = (scan_[i] - car_distances[i]) / proj_velocity;
                    // if it's small enough to count as a collision
                    if ((ttc < ttc_threshold) && (ttc >= 0.0)) { 
                        if (!TTC) {
                            first_ttc_actions();
                        }

                        no_collision = false;
                        TTC = true;

                        ROS_INFO("Collision detected");
                    }
                }
            }

            // reset TTC
            if (no_collision)
                TTC = false;

            // Revision (R3): bake the two-vehicle scenario's adversary outlines into
            // the ego scan so that the occlusion of one vehicle by the other is
            // physically present in the LiDAR. Only active in the adv2_enable (two
            // adversary) case; single-adversary runs keep the clean dynamic branch.
            if(adv_enable && adv2_enable){
                double angle_min=-M_PI;
                double ainc=(2*M_PI)/scan_.size();
                double hl=0.5*veh_det_length, hw=0.5*veh_det_width;
                Pose2D advs[2]={state_det,state_det2};
                for(int a=0;a<2;a++){
                    double dx=advs[a].x-state.x, dy=advs[a].y-state.y;
                    double xr= dx*cos(state.theta)+dy*sin(state.theta);
                    double yr=-dx*sin(state.theta)+dy*cos(state.theta);
                    double ct=cos(advs[a].theta-state.theta), st=sin(advs[a].theta-state.theta);
                    double ox[5]={ hl, hl, 0.0, -hl, -hl};
                    double oy[5]={-hw, hw, 0.0,  hw, -hw};
                    for(int c=0;c<5;c++){
                        double px=xr+ox[c]*ct-oy[c]*st;
                        double py=yr+ox[c]*st+oy[c]*ct;
                        double ang=atan2(py,px), dist=sqrt(px*px+py*py);
                        int idx=(int)std::round((ang-angle_min)/ainc);
                        if(idx>=0 && idx<(int)scan_.size() && dist<scan_[idx]) scan_[idx]=dist;
                    }
                }
            }

            // Revision (R4): random per-beam dropout (lost returns set to max range).
            if(beam_dropout_prob>0.0){
                for(size_t bi=0; bi<scan_.size(); bi++){
                    if(((double)rand()/RAND_MAX) < beam_dropout_prob) scan_[bi]=100.0;
                }
            }

            // Publish the laser message
            sensor_msgs::LaserScan scan_msg;
            scan_msg.header.stamp = timestamp;
            scan_msg.header.frame_id = scan_frame;
            scan_msg.angle_min = -scan_simulator.get_field_of_view()/2.;
            scan_msg.angle_max =  scan_simulator.get_field_of_view()/2.;
            scan_msg.angle_increment = scan_simulator.get_angle_increment();
            scan_msg.range_max = 100;
            scan_msg.ranges = scan_;
            scan_msg.intensities = scan_;

            scan_pub.publish(scan_msg);

            // Publish a transformation between base link and laser
            pub_laser_link_transform(timestamp);

        }

    } // end of update_pose

    void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg){ //Update the localization transforms
			int updated=0;
			 for (const geometry_msgs::TransformStamped& transform : msg->transforms)
			{
				if (transform.header.frame_id == "map" && transform.child_frame_id == "det_racecar_base_link") //Simulation detection of other vehicle
				{
					//Just for the one vehicle detection case
					double robx=transform.transform.translation.x;
					double roby=transform.transform.translation.y;
					// 		transform.transform.translation.z);
					double x=transform.transform.rotation.x;
					double y=transform.transform.rotation.y;
					double z=transform.transform.rotation.z;
					double w=transform.transform.rotation.w;
					double robtheta = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

					detx=(robx-state.x)*cos(state.theta)+(roby-state.y)*sin(state.theta);
					dety=-(robx-state.x)*sin(state.theta)+(roby-state.y)*cos(state.theta);
                    dettheta=robtheta-state.theta;

				}
			}


		}

    // Revision (B5): anchor the scripted-adversary timeline to the moment autonomous
    // navigation is enabled, and (re)place the ego + adversaries at their defined
    // start poses. Without this the adversary's maneuver runs off the sim-launch
    // clock and finishes while the car is still parked waiting for the "n" key, so
    // the ego only ever sees an already-stopped obstacle. Anchoring here makes the
    // dynamic scenario reproducible: the ego always witnesses the full maneuver from
    // an identical initial configuration, no matter when nav is switched on.
    void mux_callback(const std_msgs::Int32MultiArray::ConstPtr& msg){
        int active = (nav_mux_idx>=0 && nav_mux_idx<(int)msg->data.size()) ? msg->data[nav_mux_idx] : 0;
        if(active && !nav_started){
            nav_started=1;
            start_time=ros::Time::now().toSec();
            adv_speed=adv_speed0;
            // reset ego to its defined spawn
            state.x=ego_init_x; state.y=ego_init_y; state.theta=ego_init_theta;
            state.velocity=0.0; state.steer_angle=0.0; state.angular_velocity=0.0;
            state.slip_angle=0.0;
            // reset adversaries to their defined spawns
            state_det.x=adv_init_x; state_det.y=adv_init_y; state_det.theta=adv_init_theta;
            state_det2.x=adv2_init_x; state_det2.y=adv2_init_y; state_det2.theta=adv2_init_theta;
            goal_armed=0; goal_reached=0; run_over=0;
            ROS_INFO("Nav enabled: adversary timeline started; ego + adversaries reset to map '%s' start poses.", map_name.c_str());
        }
    }

    // Revision: end the run on course completion or timeout, so a reached goal is
    // logged as a lap/course time instead of every trial running to the wall-clock
    // cap. Completion requires the ego to first leave the goal region by
    // goal_arm_dist (so a lap whose finish == start is not "completed" at t=0), then
    // return within goal_radius. On timeout the outcome is published as failure.
    void check_completion(){
        if(!nav_started || run_over) return;
        double d = std::sqrt(std::pow(state.x-goal_x,2)+std::pow(state.y-goal_y,2));
        if(!goal_armed && d>goal_arm_dist) goal_armed=1;
        if(goal_armed && d<goal_radius){
            goal_reached=1;
            ROS_INFO("[completion] COURSE COMPLETE on map '%s' in %.2f s (goal within %.2f m).",
                     map_name.c_str(), ros::Time::now().toSec()-start_time, goal_radius);
            finish_run(true);
        }
        else if(ros::Time::now().toSec()-start_time > run_timeout){
            ROS_WARN("[completion] TIMEOUT on map '%s' after %.1f s (goal not reached).",
                     map_name.c_str(), run_timeout);
            finish_run(false);
        }
    }

    void finish_run(bool completed){
        run_over=1;
        // halt the ego and hold it (drive_callback is ignored while run_over)
        first_ttc_actions();
        std_msgs::Bool m; m.data=completed; run_complete_pub.publish(m);
        if(shutdown_on_goal){
            // brief grace so the planner flushes its final CSV row, then end the run
            shutdown_timer = n.createTimer(ros::Duration(1.0),
                &RacecarSimulator::shutdown_cb, this, true); // oneshot
        }
    }

    void shutdown_cb(const ros::TimerEvent&){ ros::shutdown(); }

    // Revision: draw the course goal in rviz -- a filled marker at (goal_x, goal_y)
    // and a ring of radius goal_radius around it -- so the per-map goal can be
    // verified visually before a run. Green when a run is active/pending, dimmed
    // once the goal has been reached.
    void publish_goal_marker(){
        visualization_msgs::MarkerArray arr;
        ros::Time now=ros::Time::now();
        double g = goal_reached ? 0.4 : 1.0; // dim after completion

        visualization_msgs::Marker center;
        center.header.frame_id="map"; center.header.stamp=now;
        center.ns="goal"; center.id=0;
        center.type=visualization_msgs::Marker::SPHERE;
        center.action=visualization_msgs::Marker::ADD;
        center.pose.position.x=goal_x; center.pose.position.y=goal_y; center.pose.position.z=0.15;
        center.pose.orientation.w=1.0;
        center.scale.x=center.scale.y=center.scale.z=0.35;
        center.color.r=0.0; center.color.g=g; center.color.b=0.0; center.color.a=0.9;
        arr.markers.push_back(center);

        visualization_msgs::Marker ring;
        ring.header.frame_id="map"; ring.header.stamp=now;
        ring.ns="goal"; ring.id=1;
        ring.type=visualization_msgs::Marker::LINE_STRIP;
        ring.action=visualization_msgs::Marker::ADD;
        ring.pose.orientation.w=1.0;
        ring.scale.x=0.06; // line width
        ring.color.r=0.0; ring.color.g=g; ring.color.b=0.0; ring.color.a=0.9;
        const int N=64;
        for(int i=0;i<=N;i++){
            double a=2.0*M_PI*i/N;
            geometry_msgs::Point p;
            p.x=goal_x+goal_radius*std::cos(a);
            p.y=goal_y+goal_radius*std::sin(a);
            p.z=0.05;
            ring.points.push_back(p);
        }
        arr.markers.push_back(ring);

        visualization_msgs::Marker label;
        label.header.frame_id="map"; label.header.stamp=now;
        label.ns="goal"; label.id=2;
        label.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.action=visualization_msgs::Marker::ADD;
        label.pose.position.x=goal_x; label.pose.position.y=goal_y; label.pose.position.z=0.6;
        label.pose.orientation.w=1.0;
        label.scale.z=0.4;
        label.color.r=0.0; label.color.g=g; label.color.b=0.0; label.color.a=1.0;
        char buf[80];
        snprintf(buf,sizeof(buf),"GOAL (r=%.2f m)",goal_radius);
        label.text=buf;
        arr.markers.push_back(label);

        goal_marker_pub.publish(arr);
    }

        /// ---------------------- GENERAL HELPER FUNCTIONS ----------------------

    std::vector<int> ind_2_rc(int ind) {
        std::vector<int> rc;
        int row = floor(ind/map_width);
        int col = ind%map_width - 1;
        rc.push_back(row);
        rc.push_back(col);
        return rc;
    }

    int rc_2_ind(int r, int c) {
        return r*map_width + c;

    }

    std::vector<int> coord_2_cell_rc(double x, double y) {
        std::vector<int> rc;
        rc.push_back(static_cast<int>((y-origin_y)/map_resolution));
        rc.push_back(static_cast<int>((x-origin_x)/map_resolution));
        return rc;
    }

    void first_ttc_actions() {
        // completely stop vehicle
        state.velocity = 0.0;
        state.angular_velocity = 0.0;
        state.slip_angle = 0.0;
        state.steer_angle = 0.0;
        steer_angle_vel = 0.0;
        accel = 0.0;
        desired_speed = 0.0;
        desired_steer_ang = 0.0;
    }

    void set_accel(double accel_) {
        accel = std::min(std::max(accel_, -max_accel), max_accel);
    }

    void set_steer_angle_vel(double steer_angle_vel_) {
        steer_angle_vel = std::min(std::max(steer_angle_vel_, -max_steering_vel), max_steering_vel);
    }

    void add_obs(int ind) {
        std::vector<int> rc = ind_2_rc(ind);
        for (int i=-obstacle_size; i<obstacle_size; i++) {
            for (int j=-obstacle_size; j<obstacle_size; j++) {
                int current_r = rc[0]+i;
                int current_c = rc[1]+j;
                int current_ind = rc_2_ind(current_r, current_c);
                current_map.data[current_ind] = 100;
            }
        }
        map_pub.publish(current_map);
    }

    void clear_obs(int ind) {
        std::vector<int> rc = ind_2_rc(ind);
        for (int i=-obstacle_size; i<obstacle_size; i++) {
            for (int j=-obstacle_size; j<obstacle_size; j++) {
                int current_r = rc[0]+i;
                int current_c = rc[1]+j;
                int current_ind = rc_2_ind(current_r, current_c);
                current_map.data[current_ind] = 0;

            }
        }
        map_pub.publish(current_map);
    }

    double compute_steer_vel(double desired_angle) {
        // get difference between current and desired
        double dif = (desired_angle - state.steer_angle);

        // calculate velocity
        double steer_vel;
        if (std::abs(dif) > .0001)  // if the difference is not trivial
            steer_vel = dif / std::abs(dif) * max_steering_vel;
        else {
            steer_vel = 0;
        }

        return steer_vel;
    }

    void compute_accel(double desired_velocity) {
        // get difference between current and desired
        double dif = (desired_velocity - state.velocity);

        if (state.velocity > 0) {
            if (dif > 0) {
                // accelerate
                double kp = 2.0 * max_accel / max_speed;
                set_accel(kp * dif);
            } else {
                // brake
                accel = -max_decel; 
            }    
        } else if (state.velocity < 0) {
            if (dif > 0) {
                // brake
                accel = max_decel;

            } else {
                // accelerate
                double kp = 2.0 * max_accel / max_speed;
                set_accel(kp * dif);
            }   
        } else {
	    // zero speed, accel either way
	    double kp = 2.0 * max_accel / max_speed;
	    set_accel(kp * dif);
	}
    }

        /// ---------------------- CALLBACK FUNCTIONS ----------------------

    void obs_callback(const geometry_msgs::PointStamped &msg) {
        double x = msg.point.x;
        double y = msg.point.y;
        std::vector<int> rc = coord_2_cell_rc(x, y);
        int ind = rc_2_ind(rc[0], rc[1]);
        added_obs.push_back(ind);
        add_obs(ind);
    }

    void pose_callback(const geometry_msgs::PoseStamped & msg) {
        state.x = msg.pose.position.x;
        state.y = msg.pose.position.y;
        geometry_msgs::Quaternion q = msg.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        state.theta = tf2::impl::getYaw(quat);
    }

    void pose_rviz_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg) {
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header = msg->header;
        temp_pose.pose = msg->pose.pose;
        pose_callback(temp_pose);

        // Revision: echo the dropped pose in map_starts.yaml form so per-map start
        // poses can be captured once from rviz ("2D Pose Estimate") and pasted in.
        tf2::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double yaw = tf2::impl::getYaw(quat);
        ROS_INFO("[start-pose capture] paste into map_starts.yaml under '%s':\n  ego_x: %.4f\n  ego_y: %.4f\n  ego_theta: %.4f",
                 map_name.c_str(), msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    }

    void drive_callback(const ackermann_msgs::AckermannDriveStamped & msg) {
        if(run_over){ desired_speed=0.0; desired_steer_ang=0.0; return; } // hold after run end
        desired_speed = msg.drive.speed;
        desired_steer_ang = msg.drive.steering_angle;
    }

    void explore_callback(const geometry_msgs::Twist::ConstPtr& msg) {

        desired_speed = msg->linear.x;
        if(msg->linear.x!=0){
            desired_steer_ang = atan2(msg->angular.z * params.wheelbase, msg->linear.x);
        }
        else{
            desired_steer_ang=0;
        }
        desired_speed=std::max(desired_speed,0.5); //Some exploration methods have very slow speeds even when higher speeds are allowed, min bound to ensure forward motion
        desired_steer_ang=std::min(std::max(-max_steering_angle,desired_steer_ang),max_steering_angle);
    }

      void generateWaypoints(const ros::TimerEvent& event) {
        // Republish the goal marker at ~1 Hz (this timer runs at 10 Hz) so late rviz
        // subscribers and the post-completion dim are reflected.
        if(goal_marker_tick++ % 10 == 0) publish_goal_marker();

        if(use_manual_fwd==0){
            return;
        }

        // Generate waypoints
        nav_msgs::Path waypoints;
        waypoints.header.frame_id = "map"; // Use appropriate frame
        waypoints.header.stamp = ros::Time::now();

        for (int i = 1; i <= waypoint_count; ++i) {
            geometry_msgs::PoseStamped waypoint;
            waypoint.header = waypoints.header;

            // Calculate new waypoint position based on current position
            waypoint.pose.position.x = state.x + i * waypoint_distance * cos(state.theta);
            waypoint.pose.position.y = state.y + i * waypoint_distance * sin(state.theta);
            waypoint.pose.position.z = state.theta;

            // Orientation can be set to the current orientation or adjusted as needed
            waypoint.pose.orientation = tf::createQuaternionMsgFromYaw(state.theta);

            // Publish the waypoint to the move_base goal
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = waypoint.pose;

            // Send the goal to the move_base action server
            move_base_client->sendGoal(goal);

            // Add waypoint to the path
            waypoints.poses.push_back(waypoint);
        }

        // Publish the waypoints
        waypoint_pub.publish(waypoints);
    }

    // button callbacks
    void clear_obstacles(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        bool clear_obs_clicked = false;
        if (feedback->event_type == 3) {
            clear_obs_clicked = true;
        }
        if (clear_obs_clicked) {
            ROS_INFO("Clearing obstacles.");
            current_map = original_map;
            map_pub.publish(current_map);

            clear_obs_clicked = false;
        }
    }

        void map_callback(const nav_msgs::OccupancyGrid & msg) {
            // Fetch the map parameters
            size_t height = msg.info.height;
            size_t width = msg.info.width;
            double resolution = msg.info.resolution;
            // Convert the ROS origin to a pose
            Pose2D origin;
            origin.x = msg.info.origin.position.x;
            origin.y = msg.info.origin.position.y;
            geometry_msgs::Quaternion q = msg.info.origin.orientation;
            tf2::Quaternion quat(q.x, q.y, q.z, q.w);
            origin.theta = tf2::impl::getYaw(quat);

            // Convert the map to probability values
            std::vector<double> map(msg.data.size());
            for (size_t i = 0; i < height * width; i++) {
                if (msg.data[i] > 100 or msg.data[i] < 0) {
                    map[i] = 0.5; // Unknown
                } else {
                    map[i] = msg.data[i]/100.;
                }
            }

            // Send the map to the scanner
            scan_simulator.set_map(
                map,
                height,
                width,
                resolution,
                origin,
                map_free_threshold);
            map_exists = true;
        }

        /// ---------------------- PUBLISHING HELPER FUNCTIONS ----------------------

        void pub_pose_transform(ros::Time timestamp) {
            // Convert the pose into a transformation
            geometry_msgs::Transform t;
            t.translation.x = state.x;
            t.translation.y = state.y;
            tf2::Quaternion quat;
            quat.setEuler(0., 0., state.theta);
            t.rotation.x = quat.x();
            t.rotation.y = quat.y();
            t.rotation.z = quat.z();
            t.rotation.w = quat.w();

            // publish ground truth pose
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = "/map";
            ps.pose.position.x = state.x;
            ps.pose.position.y = state.y;
            ps.pose.orientation.x = quat.x();
            ps.pose.orientation.y = quat.y();
            ps.pose.orientation.z = quat.z();
            ps.pose.orientation.w = quat.w();

            // Add a header to the transformation
            geometry_msgs::TransformStamped ts;
            ts.transform = t;
            ts.header.stamp = timestamp;
            ts.header.frame_id = map_frame;
            ts.child_frame_id = base_frame;

            // Publish them
            if (broadcast_transform) {
                br.sendTransform(ts);
            }
            if (pub_gt_pose) {
                pose_pub.publish(ps);
            }
        }


        void pub_pose_det_transform(ros::Time timestamp) {
            // Convert the pose into a transformation
            geometry_msgs::Transform t;
            t.translation.x = state_det.x;
            t.translation.y = state_det.y;
            tf2::Quaternion quat;
            quat.setEuler(0., 0., state_det.theta);
            t.rotation.x = quat.x();
            t.rotation.y = quat.y();
            t.rotation.z = quat.z();
            t.rotation.w = quat.w();

            // publish ground truth pose
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = "/map";
            ps.pose.position.x = state_det.x;
            ps.pose.position.y = state_det.y;
            ps.pose.orientation.x = quat.x();
            ps.pose.orientation.y = quat.y();
            ps.pose.orientation.z = quat.z();
            ps.pose.orientation.w = quat.w();

            // Add a header to the transformation
            geometry_msgs::TransformStamped ts;
            ts.transform = t;
            ts.header.stamp = timestamp;
            ts.header.frame_id = map_frame;
            ts.child_frame_id = "det_racecar_base_link";

            // Publish them
            if (broadcast_transform) {
                br.sendTransform(ts);
            }
            if (pub_gt_pose) {
                pose_pub.publish(ps);
            }
        }

        void pub_steer_ang_transform(ros::Time timestamp) {
            // Set the steering angle to make the wheels move
            // Publish the steering angle
            tf2::Quaternion quat_wheel;
            quat_wheel.setEuler(0., 0., state.steer_angle);
            geometry_msgs::TransformStamped ts_wheel;
            ts_wheel.transform.rotation.x = quat_wheel.x();
            ts_wheel.transform.rotation.y = quat_wheel.y();
            ts_wheel.transform.rotation.z = quat_wheel.z();
            ts_wheel.transform.rotation.w = quat_wheel.w();
            ts_wheel.header.stamp = timestamp;
            ts_wheel.header.frame_id = "front_left_hinge";
            ts_wheel.child_frame_id = "front_left_wheel";
            br.sendTransform(ts_wheel);
            ts_wheel.header.frame_id = "front_right_hinge";
            ts_wheel.child_frame_id = "front_right_wheel";
            br.sendTransform(ts_wheel);

            quat_wheel.setEuler(0., 0., 0);
            ts_wheel.transform.rotation.x = 0;
            ts_wheel.transform.rotation.y = 0;
            ts_wheel.transform.rotation.z = 0;
            ts_wheel.transform.rotation.w = 1;
            ts_wheel.header.stamp = timestamp;
            ts_wheel.header.frame_id = "det_racecar_front_left_hinge";
            ts_wheel.child_frame_id = "det_racecar_front_left_wheel";
            br.sendTransform(ts_wheel);
            ts_wheel.header.frame_id = "det_racecar_front_right_hinge";
            ts_wheel.child_frame_id = "det_racecar_front_right_wheel";
            br.sendTransform(ts_wheel);


        }

        void pub_laser_link_transform(ros::Time timestamp) {
            // Publish a transformation between base link and laser
            geometry_msgs::TransformStamped scan_ts;
            scan_ts.transform.translation.x = scan_distance_to_base_link;
            scan_ts.transform.rotation.w = 1;
            scan_ts.header.stamp = timestamp;
            scan_ts.header.frame_id = base_frame;
            scan_ts.child_frame_id = scan_frame;
            br.sendTransform(scan_ts);
        }

        void pub_odom(ros::Time timestamp) {
            // Make an odom message and publish it
            nav_msgs::Odometry odom;
            odom.header.stamp = timestamp;
            odom.header.frame_id = map_frame;
            odom.child_frame_id = base_frame;
            odom.pose.pose.position.x = state.x;
            odom.pose.pose.position.y = state.y;
            tf2::Quaternion quat;
            quat.setEuler(0., 0., state.theta);
            odom.pose.pose.orientation.x = quat.x();
            odom.pose.pose.orientation.y = quat.y();
            odom.pose.pose.orientation.z = quat.z();
            odom.pose.pose.orientation.w = quat.w();
            odom.twist.twist.linear.x = state.velocity;
            odom.twist.twist.angular.z = state.angular_velocity;
            odom_pub.publish(odom);
        }

        void pub_imu(ros::Time timestamp) {
            // Make an IMU message and publish it
            // TODO: make imu message
            sensor_msgs::Imu imu;
            imu.header.stamp = timestamp;
            imu.header.frame_id = map_frame;


            imu_pub.publish(imu);
        }

};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "racecar_simulator");
    RacecarSimulator rs;
    ros::spin();
    return 0;
}
