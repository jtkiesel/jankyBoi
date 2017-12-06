#include "MotionProfile.hpp"

#include "JINX.h"

#include <string>

namespace bns {

void MotionProfile::graph(double x) const {
	Snapshot snap = getSnapshot();
	writeJINXData("actual position", std::to_string(x).c_str());
	writeJINXData("target position", std::to_string(snap.x).c_str());
	writeJINXData("target velocity", std::to_string(snap.v).c_str());
	writeJINXData("target acceleration", std::to_string(snap.a).c_str());
}

};  // namespace bns
