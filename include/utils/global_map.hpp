/**
 * \file global_data.hpp
 *
 */

#ifndef _GLOBAL_MAP_HPP_
#define _GLOBAL_MAP_HPP_ 

#include <unordered_map>
#include <mutex>
#include <eigen3/Eigen/Dense>

struct MapItem {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	uint64_t timestamp; // Timestamp

	Eigen::Vector3d pos; // Position 
	Eigen::Vector3d vel; // Velocity

	int Nobservers; // Number of observers
};

class  GlobalMap {
	public:
		GlobalMap();
		~GlobalMap();

		// Register the source of an Item
		void register_source(int id);
		// Unregister the source of an Item
		void unregister_source(int id);

		int add_target_data(int id,
				const Eigen::Vector3d& t,
				const Eigen::Vector3d& v,
				uint64_t timestamp = 0);

		bool get_target_data(int id,
				Eigen::Vector3d& p,
				Eigen::Vector3d& v,
				uint64_t* timestamp);

		std::unordered_map<int, MapItem> getMap();

	private:
		std::unordered_map<int, MapItem> _data; 
		std::mutex _mx;
};
#endif
