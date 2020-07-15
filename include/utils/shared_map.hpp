/**
 * \file aruco_data.hpp
 *
 */

#ifndef _SHARED_MAP_HPP_
#define _SHARED_MAP_HPP_ 

#include <unordered_map>
#include <mutex>
#include <eigen3/Eigen/Dense>

struct MapData {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	uint64_t timestamp; // Timestamp
	Eigen::Vector3d tg; // Position target 
};

class SharedMap {
	public:
		SharedMap();
		~SharedMap();

		int add_target_data(int id, const Eigen::Vector3d& tg);
		bool get_target_data(int, Eigen::Vector3d& tg);
		std::unordered_map<int, MapData> getMap();

	private:
		std::unordered_map<int, MapData> data; 
		std::mutex _mx;
};
#endif
