#include "MMAEItem.hpp"



MMAEItem::MMAEItem(std::shared_ptr<BaseBayesianFilter> model, std::string modelName)
{
	this->filter = model;
}

MMAEItem::~MMAEItem()
{
}

void MMAEItem::computeProbabilityDensity(VectorXd & measurement)
{

	//Deal with the case where we have no measurement
	if (measurement.size() < 1)
	{


	}
	else
	{


	}

}

double MMAEItem::getProbDensity()
{
	return probDensity;
}
