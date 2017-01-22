#include "Person3dBVT.hpp"



Person3dBVT::Person3dBVT(VectorXd position, VectorXd bvtHist, MatrixXd positionErrorCovariance)
{
	this->position = position;
	this->bvtHist = bvtHist;
	this->positionErrorCovariance = positionErrorCovariance;
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
