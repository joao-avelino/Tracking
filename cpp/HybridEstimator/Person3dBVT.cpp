#include "Person3dBVT.hpp"

using namespace Comparator;

Person3dBVT::Person3dBVT(VectorXd observableStates, VectorXd bvtHist, MatrixXd observableCovariance, double colorPosWeight)
{
	this->observableStates = observableStates;
	this->bvtHist = bvtHist;
	this->observableCovariance = observableCovariance;
	this->colorPosWeight = colorPosWeight;
	this->objectType = TYPE_PERSON;
}

Person3dBVT::~Person3dBVT()
{
}

void Person3dBVT::setObservableStates(VectorXd observableStates)
{
	this->observableStates = observableStates;
}

void Person3dBVT::setBvtHist(VectorXd bvtHist)
{
	this->bvtHist = bvtHist;
}

void Person3dBVT::setObservableCovariance(MatrixXd posErrorCov)
{
	this->observableCovariance = posErrorCov;
}

void Person3dBVT::setNormalizedPosition(VectorXd normPos)
{
	normalizedPosition = normPos;
}

double Person3dBVT::compareWith(Object & otherObject, const int mode, const int metric)
{

	assert(this->getObjectType() == otherObject.getObjectType() && "Objects should have the same type if you wish to compare them. Check the types of detections and tracks if you are using them.");

	Person3dBVT &otherPerson = dynamic_cast<Person3dBVT&>(otherObject);

	double normalized_euclidean = 0;
	double colors = 0;

	switch (mode)
	{
	case(COMP_POSITION):

		assert((metric == METRIC_MAHALANOBIS || metric == METRIC_EUCLIDEAN) && "Position distance can only use the Euclidean or the Mahalanobis metrics for now.");

		//Mahalanobis distance between the detected person's observableStates and this ones
		//observableStates distribution

		if (metric == METRIC_MAHALANOBIS)
		{
			return mahalanobis(otherPerson.observableStates, this->observableStates, this->observableCovariance);
		}

		else if (metric == METRIC_EUCLIDEAN)
		{
			return euclidean(otherPerson.observableStates, this->observableStates);
		}

		break;
	case(COMP_COLORS):

		return hellinger(otherPerson.bvtHist, this->bvtHist);

		break;

	case(COMP_COLORSANDPOSITION):
		normalized_euclidean = euclidean(otherPerson.normalizedPosition, this->normalizedPosition);
		colors = hellinger(otherPerson.bvtHist, this->bvtHist);
		return (1- colorPosWeight)*normalized_euclidean + colorPosWeight*colors;

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

VectorXd Person3dBVT::getObservableStates()
{
	return observableStates;
}

VectorXd Person3dBVT::getBvtHist()
{
	return bvtHist;
}

MatrixXd Person3dBVT::getObervableCovariance()
{
	return observableCovariance;
}

VectorXd Person3dBVT::getNormalizedPosition()
{
	return normalizedPosition;
}
