#ifndef HUNGARIANASSOCIATOR_HPP
#define HUNGARIANASSOCIATOR_HPP
#include "BaseDataAssociator.hpp"
#include "HungarianFunctions.hpp"


template <class Obj> class HungarianAssociator :
	public BaseDataAssociator<Obj>
{
public:
	HungarianAssociator(int method, int metric) : method(method), metric(metric) {};
	~HungarianAssociator() {};


	AssociationList<Obj> associateData(vector<shared_ptr<BaseTracker<Obj> > > trackPTRvec, vector<shared_ptr<Detection<Obj> > > detecPTRvec);

protected:
	int method;
	int metric;

};



#endif // HUNGARIANASSOCIATOR_HPP

