#include "Person3dBVT.hpp"

using namespace Comparator;

Person3dBVT::Person3dBVT(VectorXd position, VectorXd bvtHist, MatrixXd positionErrorCovariance)
{
	this->position = position;
	this->bvtHist = bvtHist;
	this->positionErrorCovariance = positionErrorCovariance;
	this->objectType = TYPE_PERSON;
}

Person3dBVT::~Person3dBVT()
{
}

void Person3dBVT::setPosition(VectorXd position)
{
	this->position = position;
}

void Person3dBVT::setBvtHist(VectorXd bvtHist)
{
	this->bvtHist = bvtHist;
}

void Person3dBVT::setPositionErrorCovariance(MatrixXd posErrorCov)
{
	this->positionErrorCovariance = posErrorCov;
}

double Person3dBVT::compareWith(Object & otherObject, const int mode, const int metric)
{

	assert(this->getObjectType() == otherObject.getObjectType() && "Objects should have the same type if you wish to compare them. Check the types of detections and tracks if you are using them.");

	Person3dBVT &otherPerson = dynamic_cast<Person3dBVT&>(otherObject);

	switch (mode)
	{
	case(COMP_POSITION):

		assert((metric == METRIC_MAHALANOBIS || metric == METRIC_EUCLIDEAN) && "Position distance can only use the Euclidean or the Mahalanobis metrics for now.");

		//Mahalanobis distance between the detected person's position and this ones
		//position distribution

		if (metric == METRIC_MAHALANOBIS)
		{
			return mahalanobis(otherPerson.position, this->position, this->positionErrorCovariance);
		}

		else if (metric == METRIC_EUCLIDEAN)
		{
			return euclidean(otherPerson.position, this->position);
		}

		break;
	case(COMP_COLORS):



		break;

	case(COMP_COLORSANDPOSITION):

		//IMPLEMENTAR

		break;
	case(COMP_FORMFEATURES):

		//IMPLEMENTAR

		break;
	case(COMP_ALL):

		//IMPLEMENTAR

		break;
	default:
		break;
	}

	return 0.0;
}

VectorXd Person3dBVT::getPosition()
{
	return position;
}

VectorXd Person3dBVT::getBvtHist()
{
	return bvtHist;
}

MatrixXd Person3dBVT::getPositionErrorCovariance()
{
	return positionErrorCovariance;
}
