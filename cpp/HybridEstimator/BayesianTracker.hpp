#ifndef BAYESIANTRACKER_HPP
#define BAYESIANTRACKER_HPP

#include "Detection.hpp"
#include <eigen3/Eigen/Core>
#include <memory>

using namespace Eigen;

template <class Obj, class Estimator> class BayesianTracker
{

public:
	BayesianTracker<Obj, Estimator> (std::shared_ptr<Estimator> estimatorPTR, std::shared_ptr<Obj> objectPTR);

	std::shared_ptr<Obj> getObjPTR();
	
	double compareWith(const Obj &otherObject, const int mode, const int metric);

	void predict(const MatrixXd &controlVect);
	void update(const Detection<Obj> &det);

private:

	int trackerType;
	int trackerId;

	std::shared_ptr<Obj> objectPTR;
	std::shared_ptr<Estimator> estimatorPTR;

};



#endif // BAYESIANTRACKER_HPP


