#include "rpc/client.h"
#include <iostream>
#include <string>
#include <vector>

#include "rpc_data.hpp"

class RPCClient {
	public: 
		RPCClient(std::string ip, int port);
		~RPCClient();

		bool sync();

		void send_data(RpcData map_data);
		void get_worldmap(RpcData_v& vdata);

	private:
		int _server_port;
		std::string _server_ip;

		rpc::client* pclient;
};
