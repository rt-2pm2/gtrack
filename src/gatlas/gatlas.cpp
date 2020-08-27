/**
 * \file gatlas.cpp
 *
 */
#ifdef GATLAS_DEBUG
#include <iostream>
#endif

#include "gatlas/gatlas.hpp"
#include "rpc_classes/rpc_data.hpp"

using namespace gatlas;

GAtlas::GAtlas() {
	client = nullptr;
}

GAtlas::GAtlas(std::string& ip, int port) {
	client = new GTrackClient(ip, port);
}

GAtlas::~GAtlas() {
}

int GAtlas::update_target_data(
		int id,
		const Eigen::Vector3d& p,
		const Eigen::Vector3d& v,
		uint64_t timestamp) {
	int out = -1;
	
	_targetdata_mx.lock();
	if (_targetdata.count(id) > 0) { // Update
		out = 0;
		_targetdata[id].pos = p + 0.8 * (_targetdata[id].pos - p);
		_targetdata[id].vel = v + 0.8 * (_targetdata[id].vel - v);
		_targetdata[id].timestamp = timestamp;
	} else { // Create
		out = 1;
		TargetData ptg;
		ptg.pos = p;
		ptg.vel = v;
		ptg.Nobservers = 1;
		ptg.timestamp = timestamp;
		_targetdata.insert(std::pair<int, TargetData>(id, ptg));
	}
	_targetdata_mx.unlock();


	// If the remote connection is enable send the local information
	if (client) {
		RpcPointData data2send;
		data2send.t = timestamp;
		data2send.target_id = id;
		data2send.xx = p(0);
		data2send.yy = p(1);
		data2send.zz = p(2);

		client->send_targetdata(data2send);
	}

	return out;
}

bool GAtlas::get_target_data(
		int id,
		Eigen::Vector3d& p,
		Eigen::Vector3d& v,
		uint64_t* timestamp) {
	bool out = false;

	_targetdata_mx.lock();
	if (_targetdata.count(id) > 0) { // In memory
		out = true;
		p = _targetdata[id].pos;
		v = _targetdata[id].vel;
		*timestamp = _targetdata[id].timestamp;
		_targetdata_mx.unlock();
	} else { // Ask the server
		if (client) {
			RpcPointData_v worlddata;
			client->get_worldmap(worlddata);
			for (auto el : worlddata.data) {
				if (id == el.target_id) {
					p = Eigen::Vector3d(el.xx, el.yy, el.zz);
					v = Eigen::Vector3d::Zero();
					out = true;
					break;
				}
			}
		}
	}

	return out;
}


void GAtlas::setTransform(int a, int b,
		const TransformData& TR) { // Insert the direct transformation

	_atlasdata_mx.lock();

	if (_gatlas.count(a) == 0) {
		GAtlasElement el;
		el.id = a;
		el.relations.insert(std::pair<int, TransformData>(b, TR)); 
		_gatlas.insert(std::pair<int, GAtlasElement>(a, el));
	} else {
		_gatlas[a].relations[b] = TR;
	}
	// Insert the reversed transformation
	TransformData invTR;
	invTR.rot = TR.rot.inverse();
	invTR.t = invTR.rot * (-TR.t);

	if (_gatlas.count(b) == 0) {
		GAtlasElement el;
		el.id = b;
		el.relations.insert(std::pair<int, TransformData>(a, invTR)); 
		_gatlas.insert(std::pair<int, GAtlasElement>(b, el));
	} else {
		_gatlas[b].relations[a] = invTR;
	}

	_atlasdata_mx.unlock();

	if (client) {
		send_map_transform(a, b, TR.t, TR.rot);
	}
}

std::vector<int> GAtlas::get_fiducialsIds() {
    std::vector<int> out;

	_atlasdata_mx.lock();
    for (auto el : _gatlas) {
        out.push_back(el.first);
    }
	_atlasdata_mx.unlock();

    return out;
}


bool GAtlas::getTransform(int orig, int dest, TransformData& tr) {
	bool success = false;

	std::unique_lock<std::mutex> lk(_atlasdata_mx);

	//lk.lock();
	// Check whether the nodes are in the map
	if (_gatlas.count(orig) == 0 || _gatlas.count(dest) == 0) {
		return success;
	}

	_visited.clear(); 
	if (find_path(orig, dest, tr)) {
		success = true;;
	}
	// Just to be sure...
	_visited.clear();

	lk.unlock();

	if (!success && client) { // If not locally... 
		success = fetch_map_transform(orig, dest, tr.t, tr.rot);
	}

	return success;
}


std::unordered_map<int, TargetData> GAtlas::getMap() {
	std::lock_guard<std::mutex> {_targetdata_mx};
	return _targetdata;	
}


int GAtlas::get_items(std::vector<TargetData>& vout) {
	std::lock_guard<std::mutex> {_targetdata_mx};
	// Download the full map and update the elements
	if (client) {
		RpcPointData_v worlddata;
		client->get_worldmap(worlddata);
		for (auto el : worlddata.data) {
			if (_targetdata.count(el.target_id) == 0) {
				TargetData ptg;
				ptg.id = el.target_id;
				ptg.pos = Eigen::Vector3d(el.xx, el.yy, el.zz);
				ptg.vel = Eigen::Vector3d::Zero();
				ptg.timestamp = 0;
				_targetdata.insert(
						std::pair<int, TargetData>(el.target_id, ptg)
						);
			} else {
				_targetdata[el.target_id].pos =
					Eigen::Vector3d(el.xx, el.yy, el.zz);
			}
		}
	}

	int counter = 0;
	for (auto el : _targetdata) {
		vout.push_back(el.second);
		counter++;
	}
	return counter;
}

// PRIVATE
bool GAtlas::find_path(int s, int e, TransformData& TR) {
	bool out = false;

	if (s == e) {
		TR.rot = Eigen::Quaterniond::Identity();
		TR.t = Eigen::Vector3d::Zero();
		return true;
	}

	// Nodes connected to the current node 's' 
	std::unordered_map<int, TransformData> rels = _gatlas[s].relations;
	// Add the current node to the set of visited nodes
	_visited.insert(s);

	// For each connected node
	for (auto el : rels) {
		int nextId = el.first;
		TransformData locTR = el.second;

		TransformData downstreamTR;

		if (_visited.count(nextId) > 0) 
			continue;

		out = find_path(nextId, e, downstreamTR);
		if (out) {
#ifdef GATLAS_DEBUG
			std::cout << "Path: " << nextId << std::endl; 
			std::cout << "      " << locTR.t.transpose() << std::endl; 
			std::cout << "      " << locTR.rot.w() << " " <<
				locTR.rot.vec().transpose() << std::endl;
#endif
			TR.t = locTR.rot * downstreamTR.t + locTR.t;
			TR.rot = locTR.rot * downstreamTR.rot;
			return true;
		}
	}
	return false;
}






int GAtlas::send_map_transform(int id_i, int id_j,
		const Eigen::Vector3d& v_ij, const Eigen::Quaterniond& q_ij) {

	bool out = false;

	if (client) {
		RpcGAtlasTrsfData data;
		data.origin = id_i;
		data.dest = id_j;
		data.pos = std::vector<double>(3);
		data.quat = std::vector<double>(4);

		data.quat[0] = q_ij.w();
		for (int kk = 0; kk < 3; kk++) {
			data.pos[kk] = v_ij(kk); 
			data.quat[kk + 1] = q_ij.vec()(kk);
		}

		std::cout << "[" << id_i << "-->" << id_j << "]" << std::endl;
		std::cout << "p = " << v_ij.transpose() << std::endl; 

		out = client->send_atlas_data(data);
	}
	

	return out;
}

int GAtlas::fetch_map_transform(int src, int dst,
		Eigen::Vector3d& v_ij, Eigen::Quaterniond& q_ij) {

	bool out = false;
	if (client) {
		RpcGAtlasTrsfData data;
		out = client->get_atlas_data(src, dst, data);

		if (data.quat.size() > 0) { 
			q_ij.w() = data.quat[0];
			for (int kk = 0; kk < 3; kk++) {
				v_ij(kk) = data.pos[kk]; 
				q_ij.vec()(kk) = data.quat[kk + 1];
			}
		}
	}
	return out;
}



