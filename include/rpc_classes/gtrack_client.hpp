#ifndef _RPC_CLIENT_HPP_
#define _RPC_CLIENT_HPP_

#include "rpc/client.h"
#include <iostream>
#include <string>
#include <vector>

#include "rpc_data.hpp"

class GTrackClient {
	public: 
		GTrackClient(std::string ip, int port);
		~GTrackClient();

		void reset_connection();

		bool sync();

		bool send_targetdata(RpcPointData map_data);
		bool send_atlas_data(RpcGAtlasTrsfData data);

		bool get_worldmap(RpcPointData_v& vdata);
		bool get_atlas_data(int src, int dst, RpcGAtlasTrsfData& data);
	private:
		int _server_port;
		std::string _server_ip;

		rpc::client* pclient;
};

#endif
