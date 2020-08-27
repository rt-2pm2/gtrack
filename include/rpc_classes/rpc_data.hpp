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
    std::vector<RpcPointData> data;
    MSGPACK_DEFINE_ARRAY(data);
};

struct RpcGAtlasTrsfData {
    bool good;
	int origin;
	int dest;
	std::vector<double> pos;
	std::vector<double> quat;
	MSGPACK_DEFINE_ARRAY(good, origin, dest, pos, quat);
};

#endif
