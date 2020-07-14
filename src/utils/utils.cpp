#include "utils/utils.hpp"

bool file_exist(std::string filename) {
	bool out = false;
	struct stat buf;

	if ((stat(filename.c_str(), &buf) == 0)) {
		out = true;
	}
	return out;
}
