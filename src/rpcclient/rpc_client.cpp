#include "rpcclient/rpc_client.hpp"
#include "rpc/rpc_error.h"

RPCClient::RPCClient(std::string ip, int port) :
	pclient(new rpc::client(ip, port)),
	_server_ip(ip) {
	_server_port = port;

    pclient->set_timeout(0); 
}


RPCClient::~RPCClient() {}

void RPCClient::send_data(RpcData data) {
	try {
		pclient->call("new_data", data);
	} catch (rpc::timeout &t) {
        std::cout << t.what() << std::endl;
    }
}
