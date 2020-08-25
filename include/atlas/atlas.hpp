/**
 * \file atlas.hpp
 *
 */

#ifndef _ATLAS_HPP_
#define _ATLAS_HPP_ 

#include <unordered_map>
#include <mutex>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "rpcclient/rpc_client.hpp"

struct AtlasItem {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	int id;

	uint64_t timestamp; // Timestamp

	Eigen::Vector3d pos; // Position 
	Eigen::Vector3d vel; // Velocity

	int Nobservers; // Number of observers
};

class  Atlas {
	public:
		Atlas();
		~Atlas();

		Atlas(std::string& ip, int port);

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

		int get_items(std::vector<AtlasItem>& vout);

		int send_map_transform(int id_i, int id_j,
				Eigen::Vector3d& v_ij,
				Eigen::Quaterniond& q_ij);

		int get_map_transform(int src, int dst,
				Eigen::Vector3d& v_ij, Eigen::Quaterniond& q_ij);

		std::unordered_map<int, AtlasItem> getMap();

	private:
		/**
		 * Reference to a RPCClient class to communicate
		 */
		RPCClient* client;

		std::unordered_map<int, AtlasItem> _data; 
		std::mutex _mx;
};
#endif
