#ifndef _RPC_DATA_HPP_
#define _RPC_DATA_HPP_ 

#include "rpc/msgpack.hpp"

struct RpcData {
	// Timestamp
	uint64_t t;
	// Id
	int id;

	//Position
	double xx;
	double yy;
	double zz;

	MSGPACK_DEFINE_ARRAY(t, id, xx, yy, zz);
};

struct RpcSynchData {
	uint64_t sec;
    uint64_t nsec;
	MSGPACK_DEFINE_ARRAY(sec, nsec);
};

struct RpcData_v {
    std::vector<RpcData> data;
    MSGPACK_DEFINE_ARRAY(data);
};

struct RpcAtlasTrsfData {
	int origin;
	int dest;
	std::vector<double> pos;
	std::vector<double> quat;
	MSGPACK_DEFINE_ARRAY(origin, dest, pos, quat);
};

#endif
