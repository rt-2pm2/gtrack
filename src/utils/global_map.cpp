/**
 * \file global_map.cpp
 *
 */
#include "utils/global_map.hpp"

GlobalMap::GlobalMap() {}

GlobalMap::~GlobalMap() {
}

int GlobalMap::add_target_data(
		int id,
		const Eigen::Vector3d& p,
		const Eigen::Vector3d& v,
		uint64_t timestamp) {
	int out = -1;
	
	_mx.lock();

	if (_data.count(id) > 0) {
		// Update
		out = 0;
		_data[id].pos = p + 0.4 * (_data[id].pos - p);
		_data[id].vel = v + 0.4 * (_data[id].vel - v);
		_data[id].timestamp = timestamp;
	} else {
		// Create
		out = 1;
		MapItem ptg;
		ptg.pos = p;
		ptg.vel = v;
		ptg.Nobservers = 1;
		ptg.timestamp = timestamp;
		_data.insert(std::pair<int, MapItem>(id, ptg));
	}

	_mx.unlock();
	return out;
}

bool GlobalMap::get_target_data(
		int id,
		Eigen::Vector3d& p,
		Eigen::Vector3d& v,
		uint64_t* timestamp) {
	bool out = false;

	_mx.lock();

	if (_data.count(id) > 0) {
		out = true;
		p = _data[id].pos;
		v = _data[id].vel;
		*timestamp = _data[id].timestamp;
	} else {
		out = false;
	}	

	_mx.unlock();

	return out;
}


std::unordered_map<int, MapItem> GlobalMap::getMap() {
	std::lock_guard<std::mutex> {_mx};
	return _data;	
}

