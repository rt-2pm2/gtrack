#include "rpc/client.h"
#include <iostream>
#include <string>

#include "rpc_data.hpp"

class RPCClient {
	public: 
		RPCClient(std::string ip, int port);
		~RPCClient();

		void send_data(MapData map_data);

	private:
		int _server_port;
		std::string _server_ip;

		rpc::client* pclient;
};
