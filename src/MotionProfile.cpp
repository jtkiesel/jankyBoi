#include "MotionProfile.hpp"

#include "JINX.hpp"

namespace bns {

void MotionProfile::graph(double x) const {
	Snapshot snap = getSnapshot();
	/*int size = 1077;
	char arr[size];
	sprintf(arr, "%f", x);
	writeJINXData("actual position", arr);
	sprintf(arr, "%f", snap.x);
	writeJINXData("target position", arr);
	sprintf(arr, "%f", snap.v);
	writeJINXData("target velocity", arr);
	sprintf(arr, "%f", snap.a);
	writeJINXData("target acceleration", arr);*/
	printf("%lu,%f,%f,%f,%f\n", millis(), x, snap.x, snap.v, snap.a);
}

};  // namespace bns
