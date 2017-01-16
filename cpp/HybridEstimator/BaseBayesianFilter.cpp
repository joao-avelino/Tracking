#include "BaseBayesianFilter.hpp"



BaseBayesianFilter::BaseBayesianFilter()
{

}


BaseBayesianFilter::~BaseBayesianFilter()
{
}

std::string BaseBayesianFilter::getModelName()
{
	return modelName;
}

void BaseBayesianFilter::setModelName(std::string newName)
{
	modelName = newName;
	return;
}
