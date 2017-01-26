#ifndef BASEDATAASSOCIATOR_HPP
#define BASEDATAASSOCIATOR_HPP

#include "BaseTracker.hpp"
#include "Detection.hpp"
#include "Association.hpp"
#include <vector>

using namespace std;

template <class Obj> class BaseDataAssociator
{
public:

	//Returns a list of associations between trackers and detections
	virtual AssociationList<Obj> associateData(vector<shared_ptr<BaseTracker<Obj> > > &trackPTRvec, vector<shared_ptr<Detection<Obj> > > &detecPTRvec) = 0;

};



#endif // BASEDATAASSOCIATOR_HPP


