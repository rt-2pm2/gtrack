#include "rpcclient/rpc_client.hpp"
#include "rpc/rpc_error.h"
#include "utils/timelib.hpp"

RPCClient::RPCClient(std::string ip, int port) :
	pclient(new rpc::client(ip, port)),
	_server_ip(ip) {
	_server_port = port;

    pclient->set_timeout(1); 
}


RPCClient::~RPCClient() {}

void RPCClient::send_data(RpcData data) {
	try {
		pclient->async_call("new_data", data);
	} catch (rpc::timeout &t) {
        std::cout << t.what() << std::endl;
    }
}

void RPCClient::get_worldmap(RpcData_v& vdata) {
	// Ask for data
	int i = 0;
	vdata = pclient->call("get_data", i).as<RpcData_v>();
}

bool RPCClient::sync() {
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
}

