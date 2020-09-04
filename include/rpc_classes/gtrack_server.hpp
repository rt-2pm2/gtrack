#ifndef GTRACK_SERVER_HPP
#define GTRACK_SERVER_HPP

#include "rpc/server.h"
#include <iostream>
#include <string>
#include <time.h>
#include <mutex>

#include <unordered_map>
#include "rpc_data.hpp"
#include "gatlas/gatlas.hpp"

/**
 * This class support the sharing of information 
 * of the localization system in a distributed 
 * system
 *
 * The class includes the GAtlas class which takes care of the
 * graph of reference frames (Arucos) in the environment.
 */

class GTrackServer {
	public: 
        GTrackServer();
		GTrackServer(int port);
		~GTrackServer();

		bool Initialize();
        void start();
		rpc::server* pserver;

	private:
        bool initialized_;
		int server_port_;

        std::mutex mx;

		/**
		 */
		GAtlas ga;
        std::unordered_map<int, RpcPointData> world_map_;

        /**
         * This function takes the RPC data, converts it to the 
         * atlas format and insert it into the Atlas.
         */
        void onNewTrfData(RpcGAtlasTrsfData data);

		virtual void onNewData(RpcPointData data);
};

#endif
