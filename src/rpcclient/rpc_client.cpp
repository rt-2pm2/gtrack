#include "rpcclient/rpc_client.hpp"
#include "rpc/rpc_error.h"

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
