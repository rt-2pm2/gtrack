#ifndef _RPC_CLIENT_HPP_
#define _RPC_CLIENT_HPP_

#include "rpc/client.h"
#include <iostream>
#include <string>
#include <vector>

#include "rpc_data.hpp"

class RPCClient {
	public: 
		RPCClient(std::string ip, int port);
		~RPCClient();

		void reset_connection();

		bool sync();

		bool send_data(RpcData map_data);
		bool send_atlas_data(RpcAtlasTrsfData data);

		bool get_worldmap(RpcData_v& vdata);
		bool get_atlas_data(int src, int dst, RpcAtlasTrsfData& data);
	private:
		int _server_port;
		std::string _server_ip;

		rpc::client* pclient;
};

#endif
