/**
 * @file   tactile_state_calibrator.h
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Jan 17 2020
 *
 * @brief  tactile state calibrator
 */

#pragma once
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include <tactile_msgs/TactileState.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <std_srvs/Trigger.h>
#include <tactile_filters/PieceWiseLinearCalib.h>
#include <tactile_filters/Calibration.h>

#define DEFAULT_TARE_RECORDINDS 50  // number of samples for tare value computing

class TactileStateCalibrator
{
	ros::NodeHandle nh_;
	ros::Subscriber tactile_sub_;  //! source subscriber
	ros::Publisher tactile_pub_;  //! output publisher
	ros::Publisher tare_offset_pub_;  //! tare offset publisher
	ros::ServiceServer tare_srv_;  //! tare service

public:
	TactileStateCalibrator(const std::string &calib_filename);
	~TactileStateCalibrator();

	enum CalibType
	{
		RAW = 0,
		PWL = 1
	};

private:
	/**
	 * initiliaze subscribers and publisher
	 */
	void init(const std::string &calib_filename);

	/**
	 * fill calibs_ vector from a map of index
	 */
	bool fill_calibs(const std::map<unsigned int, std::shared_ptr<tactile::Calibration>> &map);
	/**
	 * extract indices of taxels a calib applies to
	 */
	bool extract_idx_range(const YAML::Node &node, std::map<unsigned int, std::shared_ptr<tactile::Calibration>> &map,
	                       const std::shared_ptr<tactile::Calibration> &calib);

	/**
	 * record or process the tare values
	 */
	void process_tare(const tactile_msgs::TactileState &msg, size_t sensor_idx = 0);
	/**
	 * generic tactile callback
	 */
	void tactile_state_cb(const tactile_msgs::TactileStateConstPtr &msg);
	/**
	 * service callback
	 */
	bool tare_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
	/**
	 * wrapper to call calibration map operator with different calib
	 */
	float map(float val, const std::shared_ptr<tactile::Calibration> &calib);

	std::vector<std::shared_ptr<tactile::Calibration>> calibs_;  //! one calibration pointer per index
	std::shared_ptr<tactile::Calibration> single_calib_;
	bool tare_requested_;
	std::vector<float> tare_offsets_;  //! one offset per index
	size_t tare_recordings_count_;
	std::vector<float> tare_recordings_;  //! vector of accumulated sensor values for computing tare
};
