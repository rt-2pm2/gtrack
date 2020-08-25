#include "rpcclient/rpc_client.hpp"
#include "rpc/rpc_error.h"
#include "utils/timelib.hpp"

RPCClient::RPCClient(std::string ip, int port) :
	pclient(new rpc::client(ip, port)),
	_server_ip(ip) {
	_server_port = port;

    pclient->set_timeout(50); 
}


RPCClient::~RPCClient() {}

void RPCClient::reset_connection() {
	delete pclient;
	pclient = new rpc::client(_server_ip, _server_port),
    pclient->set_timeout(50); 
}

bool RPCClient::send_data(RpcData data) {
	rpc::client::connection_state cs = pclient->get_connection_state();

	if (cs != rpc::client::connection_state::connected) {
		std::cout << "Client not connected!" << std::endl;
		reset_connection();
		return false;
	}

	try {
		pclient->async_call("new_data", data);
	} catch (rpc::timeout &t) {
        std::cout << t.what() << std::endl;
		return false;
    }
	return true;
}

bool RPCClient::send_atlas_data(RpcAtlasTrsfData data) {
	rpc::client::connection_state cs = pclient->get_connection_state();

	if (cs != rpc::client::connection_state::connected) {
		std::cout << "Client not connected!" << std::endl;
		reset_connection();
		return false;
	}

	try {
		pclient->async_call("add_atlas_trf_data", data);
	} catch (rpc::timeout &t) {
        std::cout << t.what() << std::endl;
		return false;
    } 
	return true;
}

bool RPCClient::get_atlas_data(int src, int dst, RpcAtlasTrsfData& data) {
	rpc::client::connection_state cs = pclient->get_connection_state();

	if (cs != rpc::client::connection_state::connected) {
		std::cout << "Client not connected!" << std::endl;
		reset_connection();
		return false;
	}

	try {
		data = pclient->call("get_atlas_trf_data", src, dst).as<RpcAtlasTrsfData>();
	} catch (rpc::timeout &t) {
        std::cout << t.what() << std::endl;
		return false;
    } 

	return true;
}

bool RPCClient::get_worldmap(RpcData_v& vdata) {
	rpc::client::connection_state cs = pclient->get_connection_state();

	if (cs != rpc::client::connection_state::connected) {
		std::cout << "Client not connected!" << std::endl;
		reset_connection();
		return false;
	}

	// Ask for data
	int i = 0;
	try {
		vdata = pclient->call("get_data", i).as<RpcData_v>();
	} catch (rpc::timeout &t) {
        std::cout << t.what() << std::endl;
		return false;
    }
	return true;
}

bool RPCClient::sync() {
	rpc::client::connection_state cs = pclient->get_connection_state();

	if (cs != rpc::client::connection_state::connected) {
		std::cout << "Client not connected!" << std::endl;
		reset_connection();
		return false;
	}

	RpcSynchData time_data;
	timespec t_send, t_rec;
	int good_attempt = 0;
	double net_lat = 0.0;

	while (good_attempt < 5) {
		try {
			clock_gettime(CLOCK_MONOTONIC, &t_send);
			time_data.sec = t_send.tv_sec;
			time_data.nsec = t_send.tv_nsec;
			pclient->call("synch", time_data);
			clock_gettime(CLOCK_MONOTONIC, &t_rec);
			net_lat = sub_timespec(&t_rec, &t_send) / 2.0;
			std::cout << net_lat << std::endl;
			good_attempt++;
		} catch (rpc::timeout &t) {
			good_attempt = 0;
		}
	}
	std::cout << "Synchronized!" << std::endl;
	return true;
}

