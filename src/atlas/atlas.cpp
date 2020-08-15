/**
 * \file atlas.cpp
 *
 */
#include "atlas/atlas.hpp"

Atlas::Atlas() {
	client = nullptr;
}

Atlas::Atlas(std::string& ip, int port) {
	client = new RPCClient(ip, port);
}

Atlas::~Atlas() {
}

int Atlas::add_target_data(
		int id,
		const Eigen::Vector3d& p,
		const Eigen::Vector3d& v,
		uint64_t timestamp) {
	int out = -1;
	
	_mx.lock();

	if (_data.count(id) > 0) { // Update
		out = 0;
		_data[id].pos = p + 0.8 * (_data[id].pos - p);
		_data[id].vel = v + 0.8 * (_data[id].vel - v);
		_data[id].timestamp = timestamp;
	} else { // Create
		out = 1;
		AtlasItem ptg;
		ptg.pos = p;
		ptg.vel = v;
		ptg.Nobservers = 1;
		ptg.timestamp = timestamp;
		_data.insert(std::pair<int, AtlasItem>(id, ptg));
	}
	_mx.unlock();

	RpcData data2send;
	data2send.t = timestamp;
	data2send.id = id;
	data2send.xx = p(0);
	data2send.yy = p(1);
	data2send.zz = p(2);

	client->send_data(data2send);

	return out;
}

bool Atlas::get_target_data(
		int id,
		Eigen::Vector3d& p,
		Eigen::Vector3d& v,
		uint64_t* timestamp) {
	bool out = false;

	_mx.lock();
	if (_data.count(id) > 0) { // In memory
		out = true;
		p = _data[id].pos;
		v = _data[id].vel;
		*timestamp = _data[id].timestamp;
	} else { // Ask the server
		if (client) {
			RpcData_v worlddata;
			client->get_worldmap(worlddata);
			for (auto el : worlddata.data) {
				if (id == el.id) {
					p = Eigen::Vector3d(el.xx, el.yy, el.zz);
					v = Eigen::Vector3d::Zero();
					out = true;
					break;
				}
			}
		}
	}
	_mx.unlock();

	return out;
}

int Atlas::get_items(std::vector<AtlasItem>& vout) {
	std::lock_guard<std::mutex> {_mx};
	// Download the full map and update the elements
	if (client) {
		RpcData_v worlddata;
		client->get_worldmap(worlddata);
		for (auto el : worlddata.data) {
			if (_data.count(el.id) == 0) {
				AtlasItem ptg;
				ptg.id = el.id;
				ptg.pos = Eigen::Vector3d(el.xx, el.yy, el.zz);
				ptg.vel = Eigen::Vector3d::Zero();
				ptg.timestamp = 0;
				_data.insert(std::pair<int, AtlasItem>(el.id, ptg));
			} else {
				_data[el.id].pos = Eigen::Vector3d(el.xx, el.yy, el.zz);
			}
		}
	}

	int counter = 0;
	for (auto el : _data) {
		vout.push_back(el.second);
		counter++;
	}
	return counter;
}


std::unordered_map<int, AtlasItem> Atlas::getMap() {
	std::lock_guard<std::mutex> {_mx};
	return _data;	
}

