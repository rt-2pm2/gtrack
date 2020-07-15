/**
 * \file shared_map.cpp
 *
 */
#include "utils/shared_map.hpp"

SharedMap::SharedMap() {}

SharedMap::~SharedMap() {
}

int SharedMap::add_target_data(int id, const Eigen::Vector3d& tg) {
	int out = -1;
	
	_mx.lock();

	if (data.count(id) > 0) {
		// Update
		out = 0;
		data[id].tg = tg;
	} else {
		// Create
		out = 1;
		MapData ptg;
		ptg.tg = tg;
		data.insert(std::pair<int, MapData>(id, ptg));
	}
	// XXX Should deal with the timestamp...
	_mx.unlock();
	return out;
}

bool SharedMap::get_target_data(int id, Eigen::Vector3d& tg) {
	bool out = false;

	_mx.lock();
	if (data.count(id) > 0) {
		out = true;
		tg = data[id].tg;
	} else {
		out = false;
	}	
	_mx.unlock();

	return out;
}


std::unordered_map<int, MapData> SharedMap::getMap() {
	std::unordered_map<int, MapData> out;

	_mx.lock();
	out = data;
	_mx.unlock();

	return out;
}

