#ifndef BASEOBJECTTRACKER_HPP
#define BASEOBJECTTRACKER_HPP

#include <eigen3/Eigen/Core>
#include <memory>

using namespace Eigen;

class BaseObjectTracker
{
public:

	virtual double compareWith(std::shared_ptr<BaseDetection> detection) = 0;

protected:

	int objectId;
	int objectType;


};


class BaseDetection
{
public:

	BaseDetection() {};
	~BaseDetection() {};

	virtual double compareWith(std::shared_ptr<BaseObjectTracker> tracker) = 0;

	VectorXd getPosition();
	int getObjectType();
	MatrixXd getMeasCovariance();

protected:

	VectorXd position;
	int objectType;
	MatrixXd measCovariance;

};

#endif // BASEOBJECTTRACKER_HPP