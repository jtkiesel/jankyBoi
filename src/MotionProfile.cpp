#include "MotionProfile.hpp"

#include <API.hpp>

namespace bns {

void MotionProfile::graph(double x) const {
	Snapshot snap = getSnapshot();
	printf("%lu,%f,%f,%f,%f\n", millis(), x, snap.x, snap.v, snap.a);
}

};  // namespace bns
