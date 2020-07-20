#include "rpcclient/rpc_client.hpp"

RPCClient::RPCClient(std::string ip, int port) :
	pclient(new rpc::client(ip, port)),
	_server_ip(ip) {
	_server_port = port;
}


RPCClient::~RPCClient() {}

void RPCClient::send_data(MapData data) {
	pclient->call("new_data", data);
}
