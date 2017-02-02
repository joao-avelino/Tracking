#include "MMBankItem.hpp"



MMBankItem::MMBankItem()
{
}


MMBankItem::~MMBankItem()
{
}

VectorXd MMBankItem::getStatePred()
{
	return filter->getStatePred();
}

VectorXd MMBankItem::getStatePost()
{
	return filter->getStatePost();
}

MatrixXd MMBankItem::getCovPred()
{
	return this->filter->getCovPred();
}

MatrixXd MMBankItem::getCovPost()
{
	return this->filter->getCovPost();
}

void MMBankItem::predict(VectorXd &control)
{
	this->filter->predict(control);
}

void MMBankItem::update(VectorXd & measure)
{
	this->filter->update(measure);
}

void MMBankItem::update(VectorXd & measure, MatrixXd & measurementCov)
{
	this->filter->update(measure, measurementCov);
}

std::string MMBankItem::getModelName()
{
	return this->filter->getModelName();
}
