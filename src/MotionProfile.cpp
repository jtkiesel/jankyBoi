#include "MotionProfile.hpp"

#include "api.hpp"
#include "MotionSegment.hpp"
#include "MotionState.hpp"
#include "Optional.hpp"
#include "util.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace bns {

MotionProfile::MotionProfile() {}

MotionProfile::MotionProfile(std::vector<MotionSegment> segments) : mSegments(segments) {}

bool MotionProfile::isValid() const {
	for (auto it = mSegments.begin(); it != mSegments.end(); it++) {
		if (!it->isValid()) {
			return false;
		}
		if (it != mSegments.begin() && !it->start().coincident((it - 1)->end())) {
			// Adjacent segments are not continuous.
			return false;
		}
	}
	return true;
}

bool MotionProfile::isEmpty() const {
	return mSegments.empty();
}

Optional<MotionState> MotionProfile::stateByTime(double t) const {
	if (t < startTime() && t + kEpsilon >= startTime()) {
		return Optional<MotionState>::of(startState());
	}
	if (t > endTime() && t - kEpsilon <= endTime()) {
		return Optional<MotionState>::of(endState());
	}
	for (MotionSegment s : mSegments) {
		if (s.containsTime(t)) {
			return Optional<MotionState>::of(s.start().extrapolate(t));
		}
	}
	return Optional<MotionState>::empty();
}

MotionState MotionProfile::stateByTimeClamped(double t) const {
	if (t < startTime()) {
		return startState();
	}
	if (t > endTime()) {
		return endState();
	}
	for (MotionSegment s : mSegments) {
		if (s.containsTime(t)) {
			return s.start().extrapolate(t);
		}
	}
	// Should never get here.
	return MotionState::kInvalidState;
}

Optional<MotionState> MotionProfile::firstStateByPos(double pos) const {
	for (MotionSegment s : mSegments) {
		if (s.containsPos(pos)) {
			if (epsilonEquals(s.end().pos(), pos, kEpsilon)) {
				return Optional<MotionState>::of(s.end());
			}
			const double t = std::min(s.start().nextTimeAtPos(pos), s.end().t());
			if (std::isnan(t)) {
				pros::printf("Error! We should reach 'pos' but we don't.\n");
				return Optional<MotionState>::empty();
			}
			return Optional<MotionState>::of(s.start().extrapolate(t));
		}
	}
	// We never reach pos.
	return Optional<MotionState>::empty();
}

void MotionProfile::trimBeforeTime(double t) {
	std::vector<MotionSegment>::iterator it;
	for (it = mSegments.begin(); it != mSegments.end(); it++) {
		if (it->end().t() > t) {
			if (it->start().t() <= t) {
				// Segment begins before t; let's shorten the segment.
				it->setStart(it->start().extrapolate(t));
			}
			break;  // Remaining segments are fully after t.
		}
	}
	mSegments.erase(mSegments.begin(), it);
}

void MotionProfile::clear() {
	mSegments.clear();
}

void MotionProfile::reset(MotionState initialState) {
	clear();
	mSegments.push_back(MotionSegment(initialState, initialState));
}

void MotionProfile::consolidate() {
	mSegments.erase(std::remove_if(mSegments.begin(), mSegments.end(), [](const MotionSegment &s) {
		return s.start().coincident(s.end());
	}));
}

void MotionProfile::appendControl(double acc, double dt) {
	if (isEmpty()) {
		pros::printf("Error! Trying to append to empty profile.\n");
		return;
	}
	MotionState lastEndState = mSegments.back().end();
	MotionState newStartState = MotionState(lastEndState.t(), lastEndState.pos(), lastEndState.vel(), acc);
	appendSegment(MotionSegment(newStartState, newStartState.extrapolate(newStartState.t() + dt)));
}

void MotionProfile::appendSegment(MotionSegment segment) {
	mSegments.push_back(segment);
}

void MotionProfile::appendProfile(MotionProfile profile) {
	mSegments.insert(mSegments.end(), profile.mSegments.begin(), profile.mSegments.end());
}

int MotionProfile::size() const {
	return static_cast<int>(mSegments.size());
}

std::vector<MotionSegment> MotionProfile::segments() const {
	return mSegments;
}

MotionState MotionProfile::startState() const {
	if (isEmpty()) {
		return MotionState::kInvalidState;
	}
	return mSegments.front().start();
}

double MotionProfile::startTime() const {
	return startState().t();
}

double MotionProfile::startPos() const {
	return startState().pos();
}

MotionState MotionProfile::endState() const {
	if (isEmpty()) {
		return MotionState::kInvalidState;
	}
	return mSegments.back().end();
}

double MotionProfile::endTime() const {
	return endState().t();
}

double MotionProfile::endPos() const {
	return endState().pos();
}

double MotionProfile::duration() const {
	return endTime() - startTime();
}

double MotionProfile::length() const {
	double length = 0.0;
	for (MotionSegment s : mSegments) {
		length += std::abs(s.end().pos() - s.start().pos());
	}
	return length;
}

}  // namespace bns
