#include "BayesianTracker.hpp"


template<class Obj, class Estimator>
inline BayesianTracker<Obj, Estimator>::BayesianTracker(std::shared_ptr<Estimator> estimatorPTR, std::shared_ptr<Obj> objectPTR)
{
	this->estimatorPTR = estimatorPTR;
	this->objectPTR = objectPTR;
}

template<class Obj, class Estimator>
std::shared_ptr<Obj> BayesianTracker<Obj, Estimator>::getObjPTR()
{

	//TODO
	return std::shared_ptr<Obj>();
}

template<class Obj, class Estimator>
inline double BayesianTracker<Obj, Estimator>::compareWith(const Obj &otherObject, const int mode, const int metric)
{
	return objectPTR->compareWith(otherObject, mode, metric);
}

template<class Obj, class Estimator>
void BayesianTracker<Obj, Estimator>::predict(const MatrixXd &controlVect)
{

	//TODO

}

template<class Obj, class Estimator>
void BayesianTracker<Obj, Estimator>::update(const Detection<Obj> &det)
{

	//TODO

};
