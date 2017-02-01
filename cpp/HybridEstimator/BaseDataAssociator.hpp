#ifndef BASEDATAASSOCIATOR_HPP
#define BASEDATAASSOCIATOR_HPP

#include "BaseTracker.hpp"
#include "Detection.hpp"
#include "Association.hpp"
#include <vector>

using namespace std;

template <class Obj, class Trk> class BaseDataAssociator
{
public:

	//Returns a list of associations between trackers and detections
    virtual AssociationList<Obj, Trk> associateData(vector<shared_ptr<Trk>> trackPTRvec, vector<shared_ptr<Detection<Obj> > > &detecPTRvec) = 0;

};



#endif // BASEDATAASSOCIATOR_HPP


