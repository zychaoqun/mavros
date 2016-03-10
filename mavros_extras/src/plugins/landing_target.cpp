/**
 * @brief LandingTarget plugin
 * @file landing_target.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Alex Buyval <alexbuyval@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 M.H.Kabir., 2016 Alex Buyval
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace mavplugin {
/**
 * @brief Landing target plugin
 *
 * Send ladning target pose from some visual marker detectors to FCU
 *
 */
class LandingTargetPlugin : public MavRosPlugin
	{
public:
    LandingTargetPlugin() :
		sp_nh("~landing_target"),
        uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

        pose_sub = sp_nh.subscribe("pose", 10, &LandingTargetPlugin::pose_cb, this);

	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber pose_sub;

    ros::Time last_transform_stamp;
	/* -*- low-level send -*- */

	void landing_target(uint64_t usec,
			float angle_x, float angle_y, float distance,
			uint frame, float size_x, float size_y, uint target_num) {
		mavlink_message_t msg;
		mavlink_msg_landing_target_pack_chan(UAS_PACK_CHAN(uas), &msg,
				usec,
				target_num,
				frame,				
				angle_x,
				angle_y,
				distance,
				size_x,
				size_y);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send landing_target to FCU controller
	 */
    void send_landing_target(const ros::Time &stamp, const geometry_msgs::Point p) {

//		if (last_transform_stamp == stamp) {
//			ROS_DEBUG_THROTTLE_NAMED(10, "landing_target", "Landing target: Same transform as last one, dropped.");
//			return;
//		}
//		last_transform_stamp = stamp;


        landing_target(stamp.toNSec() / 1000, p.x/p.z, p.y/p.z, p.z, 8 /*MAV_FRAME_BODY_NED*/, 1, 1, 1);
	}

	/* -*- callbacks -*- */

	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {        
        send_landing_target(req->header.stamp, req->pose.position);
	}

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::LandingTargetPlugin, mavplugin::MavRosPlugin)
