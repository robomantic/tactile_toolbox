/**
 * @file   tactile_state_calibrator.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Jan 17 2020
 *
 * @brief  tactile state calibrator
 */

#include "tactile_state_calibrator.h"
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <exception>
#include <yaml-cpp/yaml.h>

using namespace tactile;

constexpr char DETAIL[] = "detail";

TactileStateCalibrator::TactileStateCalibrator(const std::string &calib_filename)
{
	// init publisher/subscribers
	init(calib_filename);
}

bool TactileStateCalibrator::fill_calibs(const std::map<unsigned int, std::shared_ptr<Calibration>> &map)
{
	// check for the smallest and largest key
	unsigned int max_key = 0, min_key = std::numeric_limits<unsigned int>::max();
	for (const auto &el : map) {
		if (el.first > max_key)
			max_key = el.first;
		if (el.first < min_key)
			min_key = el.first;
	}
	ROS_DEBUG_STREAM_NAMED(DETAIL, " min max are " << min_key << ", " << max_key);
	// validate that we start at 0
	if (min_key != 0) {
		ROS_ERROR("the range should start at zero");
		return false;
	}
	// check validity, all index of the range should be covered
	if (map.size() < (max_key - min_key + 1)) {
		ROS_ERROR("not all indices in the range are defined");
		ROS_ERROR_STREAM("  length of the calib map: " << map.size());
		ROS_ERROR_STREAM("  length of the range:  (" << max_key << "- " << min_key
		                                             << " + 1) =" << (max_key - min_key + 1));
		return false;
	} else {
		calibs_.clear();
		calibs_.resize(max_key + 1);
		for (const auto &el : map) {
			calibs_[el.first] = el.second;
		}
		return true;
	}
	return false;
}

bool TactileStateCalibrator::extract_idx_range(const YAML::Node &node,
                                               std::map<unsigned int, std::shared_ptr<Calibration>> &map,
                                               const std::shared_ptr<Calibration> &calib)
{
	if (node.Type() == YAML::NodeType::Sequence) {
		ROS_DEBUG_STREAM_NAMED(DETAIL, " found an index range");
		std::vector<unsigned int> idx_range;
		// Break from loop as soon as single_calib_ was configured to handle all taxels
		for (YAML::const_iterator idx_range_it = node.begin(); idx_range_it != node.end() && !single_calib_;
		     ++idx_range_it) {
			idx_range.clear();
			switch ((*idx_range_it).Type()) {
				case YAML::NodeType::Sequence: {
					ROS_DEBUG_STREAM_NAMED(DETAIL, "  found a sequence");
					const YAML::Node yaml_vec = (*idx_range_it);
					ROS_DEBUG_STREAM_NAMED(DETAIL, "  extract sequence");
					idx_range = yaml_vec.as<std::vector<unsigned int>>();
					// must be pairs
					if (idx_range.size() == 2) {
						ROS_DEBUG_STREAM_NAMED(DETAIL, "  found size 2");
						// check the start and end are valid
						if (idx_range[0] < idx_range[1]) {
							ROS_DEBUG_STREAM_NAMED(DETAIL, "  index range valid");
							// add the range to the idx_map and associate it to the calibration pointer
							for (unsigned int i = idx_range[0]; i <= idx_range[1]; ++i) {
								map[i] = calib;
							}
							ROS_DEBUG_STREAM_NAMED(DETAIL,
							                       "  added index in range [" << idx_range[0] << ", " << idx_range[1] << "]");
						} else {
							ROS_ERROR("Invalid range, start of range should be smaller then end of range and positive");
							return false;
						}
					} else {
						ROS_WARN("Wrong idx_range, must be pairs");
						return false;
					}
					break;
				}
				case YAML::NodeType::Scalar: {
					ROS_DEBUG_STREAM_NAMED(DETAIL, "  found a scalar");
					const YAML::Node yaml_scal = (*idx_range_it);
					if (yaml_scal.as<int>() >= 0)
						map[yaml_scal.as<unsigned int>()] = calib;
					else  // negative value == apply to all == single calib
						single_calib_ = calib;
				}
				default:
					break;
			}
		}
		return true;
	} else  // not an accepted range
	{
		ROS_ERROR("Index range should be provided as a sequence");
		return false;
	}
}

void TactileStateCalibrator::init(const std::string &calib_filename)
{
	if (!calib_filename.empty()) {
		calibs_.clear();
		const YAML::Node yaml_node = YAML::LoadFile(calib_filename);
		if (yaml_node["calib"]) {
			const YAML::Node yaml_calibs = yaml_node["calib"];
			std::map<unsigned int, std::shared_ptr<tactile::Calibration>> idx_to_calib_map;
			std::shared_ptr<tactile::Calibration> calib_ptr;
			single_calib_.reset();
			calibs_.clear();

			switch (yaml_calibs.Type()) {
				case YAML::NodeType::Sequence:  // a sequence of calibration configuration
					ROS_DEBUG_NAMED(DETAIL, "found calibs");
					// for each calib
					for (YAML::const_iterator calib_it = yaml_calibs.begin(); calib_it != yaml_calibs.end(); ++calib_it) {
						YAML::Node yaml_calib = *calib_it;
						// extract sensor name
						std::string sensor_name = "None";  // TODO: sensor_name is not yet used.
						if (yaml_calib["sensor_name"])
							sensor_name = yaml_calib["sensor_name"].as<std::string>();

						// extract calibration type
						TactileStateCalibrator::CalibType calib_type = TactileStateCalibrator::CalibType::RAW;
						if (yaml_calib["type"]) {
							const std::string calib_type_str = yaml_calib["type"].as<std::string>();
							if (calib_type_str == "PWL" || calib_type_str == "pwl" || calib_type_str == "PieceWiseLinear")
								calib_type = TactileStateCalibrator::CalibType::PWL;
						}

						// search for parameters/values depending on calib_type
						switch (calib_type) {
							case TactileStateCalibrator::CalibType::PWL:
								// search for mapping values
								// check if the sequence is defining calibration mappings
								if (yaml_calib["values"]) {
									if (yaml_calib["values"].Type() == YAML::NodeType::Map) {
										ROS_DEBUG_STREAM(" loading map");
										calib_ptr = std::make_shared<PieceWiseLinearCalib>(
										    PieceWiseLinearCalib::load(yaml_calib["values"]));
										ROS_DEBUG_STREAM(" input range is :" << calib_ptr->input_range());
										ROS_DEBUG_STREAM(" output range is :" << calib_ptr->output_range());
									} else
										ROS_ERROR("Values must be provided as a map in yaml");
								} else
									ROS_ERROR("Values not found but Piece Wise Linear requires values");
								break;

							case TactileStateCalibrator::CalibType::RAW:
							default:
								calib_ptr.reset();
						}

						// extract range
						if (yaml_calib["idx_range"]) {
							ROS_DEBUG_STREAM_NAMED(DETAIL, " extracting index range");
							if (extract_idx_range(yaml_calib["idx_range"], idx_to_calib_map, calib_ptr)) {
								// check the result
								if (!single_calib_) {
									if (idx_to_calib_map.empty()) {
										ROS_ERROR(" No calibration assigned, something went wrong");
										throw std::runtime_error("No calibration assigned, something went wrong");
										return;
									}
								} else {  // one can break the for loop here
									break;
								}
							} else {
								ROS_ERROR(" extracting index range failed");
								break;
							}
						} else  // no range provided, apply to all
						{
							single_calib_ = calib_ptr;
							// break the for loop
							break;
						}
					}

					// if required, prepare the calibs_ vector from the idx_to_calib map
					if (!single_calib_ && !idx_to_calib_map.empty()) {
						// fill calibs_
						ROS_DEBUG_STREAM_NAMED(DETAIL, " filling range");
						fill_calibs(idx_to_calib_map);
					}
					break;

				case YAML::NodeType::Map:  // wrong type
					ROS_FATAL("Wrong format, sequence calibration configuration expected");
					break;

				default:
					break;
			}
		} else {
			ROS_DEBUG_STREAM_NAMED(DETAIL, "found a single map");
			single_calib_ = std::make_shared<PieceWiseLinearCalib>(PieceWiseLinearCalib::load(yaml_node));
		}
		if (!single_calib_ && calibs_.empty()) {
			ROS_DEBUG_STREAM_NAMED(DETAIL, " calibs and single_calib empty");
			throw std::runtime_error("unable to create PieceWiseLinearCalib");
			return;
		}
	} else {
		throw std::runtime_error("calibration file cannot be empty");
		return;
	}
	ROS_DEBUG_STREAM("mapping loaded");

	tare_requested_ = false;
	tare_recordings_count_ = 0;
	// initialize publishers
	tactile_pub_ = nh_.advertise<tactile_msgs::TactileState>("out_tactile_states", 5);
	tare_offset_pub_ = nh_.advertise<sensor_msgs::ChannelFloat32>("out_tare_offset", 5);

	// initialize subscriber
	tactile_sub_ = nh_.subscribe("in_tactile_states", 5, &TactileStateCalibrator::tactile_state_cb, this);
	ROS_INFO("tactile_state_calibrator initialized");
	// initialize service
	tare_srv_ = nh_.advertiseService("tare_request", &TactileStateCalibrator::tare_cb, this);
}

float TactileStateCalibrator::map(float val, const std::shared_ptr<Calibration> &calib)
{
	if (!calib)
		return val;
	else
		return calib->map(val);
}

void TactileStateCalibrator::process_tare(const tactile_msgs::TactileState &msg, size_t sensor_idx)
{
	if (tare_requested_) {
		if (tare_recordings_count_ >= DEFAULT_TARE_RECORDINDS) {
			tare_requested_ = false;
			ROS_INFO_STREAM("tared");
			tare_offsets_.clear();
			tare_offsets_.resize(tare_recordings_.size());
			// compute tare values
			std::transform(tare_recordings_.begin(), tare_recordings_.end(), tare_offsets_.begin(),
			               [](float val) -> float { return val / (float)DEFAULT_TARE_RECORDINDS; });
			// reset recordings
			tare_recordings_.clear();
			tare_recordings_count_ = 0;
			// publish tare offsets
			sensor_msgs::ChannelFloat32 tare_msg;
			tare_msg.values = tare_offsets_;
			tare_offset_pub_.publish(tare_msg);
		} else {
			// store values (how to handle multiple sensors ? one tare per sensor per value ?
			// now only handle a single sensor since we also use the same calib map for all sensors
			if (tare_recordings_.size() != msg.sensors[sensor_idx].values.size())
				tare_recordings_.resize(msg.sensors[sensor_idx].values.size(), .0);
			// use non-tared reading for future tare
			std::transform(msg.sensors[sensor_idx].values.begin(), msg.sensors[sensor_idx].values.end(),
			               tare_recordings_.begin(), tare_recordings_.begin(),
			               [](float val, float acc) -> float { return acc + val; });
			tare_recordings_count_++;
		}
	}
}

void TactileStateCalibrator::tactile_state_cb(const tactile_msgs::TactileStateConstPtr &msg)
{
	tactile_msgs::TactileState out_msg;
	out_msg = *msg;
	// TODO Guillaume: use a different calib file for each type of sensor (maybe regex on the name)
	for (size_t i = 0; i < msg->sensors.size(); ++i) {
		// if one calibmap for this sensor,
		if (calibs_.empty() && single_calib_) {
			// same function as in the previous glove console
			std::transform(msg->sensors[i].values.begin(), msg->sensors[i].values.end(), out_msg.sensors[i].values.begin(),
			               std::bind(&tactile::Calibration::map, single_calib_.get(), std::placeholders::_1));
			// process new tare before taring the value
			if (i == 0)
				process_tare(out_msg, i);
			// do an additional offset transformation unless tare_offsets_ is empty();
			if (tare_offsets_.size() == out_msg.sensors[i].values.size()) {
				std::transform(out_msg.sensors[i].values.begin(), out_msg.sensors[i].values.end(), tare_offsets_.begin(),
				               out_msg.sensors[i].values.begin(),
				               [](float val, float offset) -> float { return val - offset; });
			}
		} else  // if more than one calibmap for this sensor
		{
			// TODO Guillaume: support a more advanced scheme with taxel start and end, and different calib for different
			// parts of the vector check if as many calib maps as values
			if (calibs_.size() == msg->sensors[i].values.size()) {
				// binary operator, first argument is the value (of that taxel), second is the pointer to which map to use
				// (for that taxel)
				std::transform(msg->sensors[i].values.begin(), msg->sensors[i].values.end(), calibs_.begin(),
				               out_msg.sensors[i].values.begin(),
				               std::bind(&TactileStateCalibrator::map, this, std::placeholders::_1, std::placeholders::_2));
				// process new tare before taring the value
				if (i == 0)
					process_tare(out_msg, i);
				// do an additional offset transformation unless tare_offsets_ is empty();
				if (tare_offsets_.size() == out_msg.sensors[i].values.size()) {
					// binary operator, first argument is the value of that taxel, second is the offset value for that taxel
					std::transform(out_msg.sensors[i].values.begin(), out_msg.sensors[i].values.end(), tare_offsets_.begin(),
					               out_msg.sensors[i].values.begin(),
					               [](float val, float offset) -> float { return val - offset; });
				}
			} else {
				ROS_WARN_STREAM_ONCE("Only " << calibs_.size() << " maps found, " << msg->sensors[i].values.size()
				                             << " expected");
			}
			// else no calib
		}
	}
	tactile_pub_.publish(out_msg);
}

bool TactileStateCalibrator::tare_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
	(void)req;  // fixes unused parameter warning
	if (tare_requested_) {
		resp.success = false;
		resp.message = "tare ongoing";
	} else {
		resp.success = true;
		// start tare
		tare_requested_ = true;
	}
	return true;
}

TactileStateCalibrator::~TactileStateCalibrator()
{
	// unregister
	tactile_sub_.shutdown();
	// cleanup
	calibs_.clear();
	single_calib_.reset();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tactile_state_calibrator");
	// read parameters
	ros::NodeHandle nh_priv("~");
	std::string calib_filename = "";
	if (!nh_priv.getParam("calib", calib_filename)) {
		ROS_ERROR_STREAM("No calibration file provided");
		return EFAULT;
	}

	int verbose;
	nh_priv.getParam("verbose", verbose);
	if (verbose > 0) {
		// switch to debug mode
		bool changed = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
		if (not(verbose > 1))  // if verbosity = 1, but not > 1, silent detailed messages
			changed |= ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME ".detail", ros::console::levels::Info);
		if (changed)
			ros::console::notifyLoggerLevelsChanged();
	}

	try {
		TactileStateCalibrator tsc(calib_filename);
		ros::spin();
	} catch (const std::exception &e) {
		ROS_ERROR_STREAM(e.what());
		return EFAULT;
	}
	return 0;
}
