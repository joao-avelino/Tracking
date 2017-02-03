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

void MMBankItem::setStatePred(VectorXd statePred)
{
	this->filter->setStatePred(statePred);
}
void MMBankItem::setCovPred(MatrixXd covPred)
{
	this->filter->setCovPred(covPred);
}

void MMBankItem::setStatePost(VectorXd statePost)
{
	this->filter->setStatePost(statePost);
}

void MMBankItem::setCovPost(MatrixXd covPost)
{
	this->filter->setCovPost(covPost);
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
