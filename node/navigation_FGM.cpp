// navigation_FGM.cpp -- Follow-the-Gap Method (FGM) baseline for the STLMPC
// revision simulation campaign (study B4).
//
// This node implements the Follow-the-Gap Method of Sezer & Gokasan (2012), the
// gap-following heuristic that inspired STLMPC's heading selection. It uses the
// paper's true geometric gap-CENTRE angle (Eq. 8 -- the median-vector to the
// midpoint of the two gap-bounding obstacles), NOT the less-safe "FGM-basic"
// midpoint-of-angles (Eq. 9). The goal-fusion term (Eq. 11) is omitted because
// STLMPC operates without a global goal, so the final heading reduces to the
// gap-centre angle. FGM deliberately OMITS the tracking-line QP and the non-convex
// MPC tracking layers, so the comparison against STLMPC isolates the value those
// layers add over canonical gap-following. Per scan it (1) thresholds ranges at
// d_safe, (2) finds the widest angular free gap in the front pi-window, (3) aims
// at the geometric gap centre, and (4) steers there by pure-pursuit / feedback
// linearization at constant speed.
//
// It publishes AckermannDriveStamped on nav_drive_topic (the same channel the
// STLMPC nav node uses) so it slots into the existing mux unchanged. Per-step
// telemetry uses the shared RunLogger, so aggregate_runs.py treats FGM logs
// identically to STLMPC logs. Pose (x,y,theta) is taken from the map->base_link
// TF published by the simulator, matching the STLMPC nodes' logged pose.

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <f1tenth_simulator/run_logger.h>

class FollowTheGap {
    ros::NodeHandle nf;
    ros::Subscriber scan_sub;
    ros::Publisher  drive_pub;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // Parameters
    std::string scan_topic, drive_topic, map_frame, base_frame;
    double wheelbase=0.287;
    double max_steering_angle=0.4189;
    double vehicle_velocity=1.5;   // constant speed (matches Table I "v")
    double safe_distance=2.0;      // d_safe: hazard threshold (m)
    double heading_beam_angle=M_PI/8; // half-window used for the forward proximity stop
    double stop_distance=0.5;      // brake to rest if closest forward obstacle is nearer than this
    double lookahead=1.0;          // pure-pursuit lookahead distance (m)

    // Logging
    int enable_logging=0;
    std::string log_file="";
    RunLogger run_logger;
    double log_t0=-1;

public:
    FollowTheGap(): tf_listener(tf_buffer) {
        // Params are loaded into this node's PRIVATE namespace (via the launch
        // file's <rosparam command="load"> + <param> overrides), so read them
        // from a private handle, matching the STLMPC nodes' nf = NodeHandle("~").
        ros::NodeHandle pn("~");
        pn.getParam("scan_topic", scan_topic);
        pn.getParam("nav_drive_topic", drive_topic);
        pn.getParam("map_frame", map_frame);
        pn.getParam("base_frame", base_frame);
        pn.getParam("wheelbase", wheelbase);
        pn.getParam("max_steering_angle", max_steering_angle);
        pn.getParam("vehicle_velocity", vehicle_velocity);
        pn.getParam("safe_distance", safe_distance);
        pn.getParam("heading_beam_angle", heading_beam_angle);
        pn.getParam("stop_distance", stop_distance);
        pn.param("fgm_lookahead", lookahead, 1.0);

        pn.param("enable_logging", enable_logging, 0);
        pn.param<std::string>("log_file", log_file, std::string(""));
        run_logger.init(log_file, enable_logging!=0);

        scan_sub  = nf.subscribe(scan_topic, 1, &FollowTheGap::scan_callback, this);
        drive_pub = nf.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    }

    // Return the map-frame ego pose from TF; false if unavailable.
    bool ego_pose(double& x, double& y, double& th) {
        geometry_msgs::TransformStamped tf;
        try { tf = tf_buffer.lookupTransform(map_frame, base_frame, ros::Time(0)); }
        catch (tf2::TransformException&) { return false; }
        x = tf.transform.translation.x;
        y = tf.transform.translation.y;
        double qx=tf.transform.rotation.x, qy=tf.transform.rotation.y;
        double qz=tf.transform.rotation.z, qw=tf.transform.rotation.w;
        th = atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz));
        return true;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        const int n = scan->ranges.size();
        if (n < 3) return;
        const double amin = scan->angle_min;
        const double ainc = scan->angle_increment;

        // Restrict to the front pi-window (-pi/2 .. pi/2).
        auto angle_of = [&](int i){ return amin + i*ainc; };
        auto range_of = [&](int i){ double r=scan->ranges[i];
            return (std::isfinite(r) && r>0.0) ? (double)r : (double)scan->range_max; };

        // FGM step 1: find the WIDEST ANGULAR gap of "free" beams (range > d_safe)
        // in the front window (Sezer & Gokasan 2012, Sec. 3.1). Range weighting is
        // deliberately NOT used here so the baseline stays faithful to canonical FGM.
        int best_start=-1, best_end=-1; double best_width=-1;
        int cur_start=-1;
        double fwd_min = 1e9;
        for (int i=0; i<n; ++i) {
            double ang = angle_of(i);
            double r = range_of(i);
            if (std::fabs(ang) < heading_beam_angle && r < fwd_min) fwd_min = r; // stop proximity

            bool in_front = (ang >= -M_PI/2 && ang <= M_PI/2);
            bool free = in_front && (r > safe_distance);
            if (free) {
                if (cur_start<0) cur_start=i;
                bool last = (i==n-1) || (angle_of(i+1) > M_PI/2);
                if (last) {
                    double w = angle_of(i)-angle_of(cur_start);
                    if (w>best_width){best_width=w;best_start=cur_start;best_end=i;}
                    cur_start=-1;
                }
            } else if (cur_start>=0) {           // close the current gap
                double w = angle_of(i-1)-angle_of(cur_start);
                if (w>best_width){best_width=w;best_start=cur_start;best_end=i-1;}
                cur_start=-1;
            }
        }

        // FGM step 2: gap-CENTRE angle (Eq. 8) = the angle of the median vector from
        // the robot to the midpoint of the segment joining the two obstacles that
        // bound the gap. This is the true FGM heading; the simple (a1+a2)/2 midpoint
        // is the less-safe "FGM-basic" variant the paper explicitly distinguishes.
        // Goal fusion (Eq. 11) is omitted because STLMPC operates without a global
        // goal, so phi_final reduces to phi_gap_c.
        double theta_head;
        if (best_start>=0) {
            int iR = (best_start-1>=0)      ? best_start-1 : best_start; // right-bounding obstacle
            int iL = (best_end+1<n)         ? best_end+1   : best_end;   // left-bounding obstacle
            double aR=angle_of(iR), dR=range_of(iR);
            double aL=angle_of(iL), dL=range_of(iL);
            // median vector to the midpoint of the two obstacle points (P1+P2)/2:
            theta_head = atan2(dR*sin(aR)+dL*sin(aL), dR*cos(aR)+dL*cos(aL));
        } else {
            theta_head = 0.0; // no free gap: aim straight, rely on the supervisory stop
        }

        // Pure-pursuit / feedback-linearization steering toward theta_head.
        double delta = atan2(2.0*wheelbase*sin(theta_head), lookahead);
        delta = std::max(-max_steering_angle, std::min(max_steering_angle, delta));

        double v = vehicle_velocity;
        if (fwd_min < stop_distance) v = 0.0; // supervisory stop, mirrors STLMPC

        ackermann_msgs::AckermannDriveStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = base_frame;
        msg.drive.steering_angle = delta;
        msg.drive.speed = v;
        drive_pub.publish(msg);

        if (run_logger.enabled()) {
            if (log_t0<0) log_t0 = ros::Time::now().toSec();
            double d_min=1e9;
            for (int i=0;i<n;++i){ double r=scan->ranges[i]; if(std::isfinite(r)&&r>0.01&&r<d_min) d_min=r; }
            double ex=0,ey=0,eth=0; ego_pose(ex,ey,eth);
            run_logger.row({
                {"t", ros::Time::now().toSec()-log_t0},
                {"x", ex}, {"y", ey}, {"theta", eth},
                {"v_cmd", v}, {"delta_cmd", delta},
                {"d_min", d_min}, {"fwd_min", fwd_min},
                {"theta_head", theta_head}
            });
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_fgm");
    FollowTheGap fgm;
    ros::spin();
    return 0;
}
