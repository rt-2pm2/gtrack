#ifndef _RPC_DATA_HPP_
#define _RPC_DATA_HPP_ 

#include "rpc/msgpack.hpp"

/**
 * These are the data structures exchanged with the RPC
 * protocol
 */

struct RpcPointData {
	// Timestamp
	uint64_t t;

	// Source Id
	int source_id;

	// Target Id
	int target_id;

	//Position
	double xx;
	double yy;
	double zz;

	MSGPACK_DEFINE_ARRAY(t, source_id, target_id, xx, yy, zz);
};

struct RpcSynchData {
	uint64_t sec;	
    uint64_t nsec;
	MSGPACK_DEFINE_ARRAY(sec, nsec);
};

struct RpcPointData_v {
	// Array of points
    std::vector<RpcPointData> data;
    MSGPACK_DEFINE_ARRAY(data);
};

struct RpcGAtlasTrsfData {
	// Is the data meaningfull?
    bool good;

	// Start frame 
	int origin;
	// End Frame
	int dest;
	
	// Translation
	std::vector<double> pos;
	// Rotation
	std::vector<double> quat;
	MSGPACK_DEFINE_ARRAY(good, origin, dest, pos, quat);
};

#endif
