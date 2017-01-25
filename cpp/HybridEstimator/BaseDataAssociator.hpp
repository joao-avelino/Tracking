#ifndef BASEDATAASSOCIATOR_HPP
#define BASEDATAASSOCIATOR_HPP

#include "BaseTracker.hpp"
#include "Detection.hpp"
#include "Association.hpp"
#include <vector>

using namespace std;

template <class Tracker, class Detect> class DataAssociator
{
public:

	//Returns a list of associations between trackers and detections
	virtual vector<Association<Tracker, Detect> > associateData(vector<shared_ptr<Tracker> > trackPTRvec, vector<shared_ptr<Detect> > detecPTRvec) = 0;

protected:
	DataAssociator() {};

};



#endif // BASEDATAASSOCIATOR_HPP


