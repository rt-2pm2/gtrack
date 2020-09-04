#include "rpc/this_handler.h"
#include "rpc_classes/gtrack_server.hpp"

#define _GTRACK_SERVER_DEBUG 


GTrackServer::GTrackServer() :
    pserver(nullptr) {
        initialized_ = false;
		server_port_ = 8080;
}

GTrackServer::GTrackServer(int port) :
    pserver(nullptr) {
        initialized_ = false;
		server_port_ = port;
}

GTrackServer::~GTrackServer() {}

void GTrackServer::start() {
    if (pserver)
        pserver->async_run(2);
}

bool GTrackServer::Initialize() {
    pserver = new rpc::server(server_port_);
    
	pserver->bind("new_data", [this](RpcPointData data) {
            onNewData(data);
			// Disable the response
			rpc::this_handler().disable_response();
			return 0;
            });

    pserver->bind("get_data", [this](int i){
            //std::cout << "Answering [" << i << "]" <<
            //std::endl;
            RpcPointData_v outdata;
            mx.lock();
            for (auto el : world_map_) {
                RpcPointData d;
                d.target_id = 3;
                d.xx = el.second.yy + 0.115; 
                d.yy = -el.second.xx + 0.14; 
                d.zz = el.second.zz;
                outdata.data.push_back(d);
            }
            mx.unlock();
            //rpc::this_handler().respond(outdata);
            return outdata;
            });

	// Service to add Tranformation between atlas items
	pserver->bind("add_atlas_trf_data", [this](RpcGAtlasTrsfData data) {
			onNewTrfData(data);
			rpc::this_handler().disable_response();
			return 0;	
			});

	pserver->bind("get_atlas_trf_data", [this](int src, int dst) {
			bool success; 
			gatlas::TransformData trf {};	
			mx.lock();
			success = ga.getTransform(src, dst, trf);
			mx.unlock();

			RpcGAtlasTrsfData outdata {};
            outdata.good = false;
			
			if (success) {
                outdata.good = true;
				outdata.origin = src;
				outdata.dest = dst;
				outdata.pos = std::vector<double>(3);
				outdata.quat = std::vector<double>(4);
				outdata.quat[0] = trf.rot.w();
				for (int i = 0; i < 3; i++) {
					outdata.pos[i] = trf.t(i);
					outdata.quat[i + 1] = trf.rot.vec()(i);
				}
			}

			return success;
			});

    start();

    initialized_ = true;
    return true;
}


void GTrackServer::onNewData(RpcPointData data) {
}

void GTrackServer::onNewTrfData(RpcGAtlasTrsfData data) {
	gatlas::TransformData tf;
	tf.rot.w() = data.quat[0];
	for (int i = 0; i < 3; i++) {
		tf.t(i) = data.pos[i];
		tf.rot.vec()(i) = data.quat[i + 1];
	}
	ga.setTransform(data.origin, data.dest, tf);
}
