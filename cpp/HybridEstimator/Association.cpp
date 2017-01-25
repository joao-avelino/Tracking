#include "Association.hpp"

template<class Obj>
vector<Association<Obj>> AssociationList<Obj>::getSuccessfulAssociations()
{
	return vector<Association<Obj>>();
}

template<class Obj>
vector<shared_ptr<BaseTracker<Obj>>> AssociationList<Obj>::getUnassociatedTrackers()
{


	return vector<shared_ptr<BaseTracker<Obj>>>();
}

template<class Obj>
vector<shared_ptr<Detection<Obj>>> AssociationList<Obj>::getUnassociatedDetections()
{


	return vector<shared_ptr<Detection<Obj>>>();
}

template<class Obj>
void AssociationList<Obj>::addSuccessfulAssociation(Association<Obj> assoc)
{
	this->associationList.push_back(assoc);
}

template<class Obj>
void AssociationList<Obj>::addUnassociatedDetection(shared_ptr<Detection<Obj>> det)
{
	this->unassociatedDetections.push_back(det);
}


