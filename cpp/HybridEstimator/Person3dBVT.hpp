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
	Person3dBVT(VectorXd position, VectorXd bvtHist, MatrixXd positionErrorCovariance);
	~Person3dBVT();

	void setPosition(VectorXd position);
	void setBvtHist(VectorXd bvtHist);
	void setPositionErrorCovariance(MatrixXd posErrorCov);

	double compareWith(Object &otherObject, int mode, int metric);

	VectorXd getPosition();
	VectorXd getBvtHist();
	MatrixXd getPositionErrorCovariance();

	shared_ptr<Object> clone()
	{
		return shared_ptr<Object>(new Person3dBVT(this->position, this->bvtHist, this->positionErrorCovariance));
	}

protected:
	VectorXd position;
	VectorXd bvtHist;
	MatrixXd positionErrorCovariance;
};


#endif //PERSON3DBVT
