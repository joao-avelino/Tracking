#ifndef PERSON3DBVT_HPP
#define PERSON3DBVT_HPP

#include "Object.hpp"
#include "Comparator.hpp"
#include <eigen3/Eigen/Core>

using namespace Eigen;
using namespace std;

class Person3dBVT :
	public Object
{
public:
	Person3dBVT(VectorXd observableStates, VectorXd bvtHist, MatrixXd observableCovariance);
	~Person3dBVT();

	void setObservableStates(VectorXd observableStates);
	void setBvtHist(VectorXd bvtHist);
	void setObservableCovariance(MatrixXd posErrorCov);

	double compareWith(Object &otherObject, int mode, int metric);

	VectorXd getObservableStates();
	VectorXd getBvtHist();
	MatrixXd getObervableCovariance();

	shared_ptr<Object> clone()
	{
		return shared_ptr<Object>(new Person3dBVT(this->observableStates, this->bvtHist, this->observableCovariance));
	}

protected:
	VectorXd observableStates;
	VectorXd bvtHist;
	MatrixXd observableCovariance;
};


#endif //PERSON3DBVT
