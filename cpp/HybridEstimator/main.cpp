#include <iostream>
#include "KalmanFilter.hpp"
#include <eigen3/Eigen/Eigenvalues>

#include "Kfutils.hpp"

#include "Comparator.hpp"

#include "HungarianAssociator.hpp"
#include "Person3dBVT.hpp"
#include "PeopleTrackerWithMMAE.hpp"
#include "Association.hpp"

using namespace std;

int main()
{

/*
    MatrixXd P;
    MatrixXd U;
    VectorXd D;

    P = MatrixXd(3,3);

    P << 100, 30, 30,
            30, 115, 20,
            30, 20, 123;

    KFUtils::uduFactorization(P, U, D);

    cout << "U: " << U <<endl;
    MatrixXd D_mat = D.asDiagonal();
    cout << "Dif: " << P-U*D_mat*U.transpose() << endl;

    VectorXd correlatedData(2);
    correlatedData << 4.1864, 1.9174;
    VectorXd uncorrelatedData;
    MatrixXd H_normal(2,4);
    H_normal << 1, 0, 0, 0,
            0, 1, 0, 0;

    MatrixXd H_uncorr;
    MatrixXd R = MatrixXd(2,2);
    R << 1.5083, 0.8623,
            0.8623, 0.4965;

    MatrixXd uncorrR;

    KFUtils::decorrelateData(correlatedData,uncorrelatedData,
                                  H_normal,H_uncorr,
                                  R,uncorrR);

*/
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    MatrixXd stateTransitionModel(4,4);
    stateTransitionModel << 1, 0, 0.5, 0,
            0, 1, 0, 0.5,
            0, 0, 1, 0,
            0, 0, 0, 1;

    MatrixXd observationModel(2,4);
    observationModel << 1, 0, 0, 0,
            0, 1, 0, 0;

    double T = 0.5;

    MatrixXd processNoiseCovariance(4,4);
    processNoiseCovariance << pow(T, 4)/4, 0, pow(T,3)/2, 0, 0, pow(T, 4)/4, 0, pow(T,3)/2, pow(T,3)/2, 0, pow(T,2), 0, 0, pow(T,3)/2, 0, pow(T,2);

    MatrixXd observationNoiseCov(2,2);
    observationNoiseCov << 1.5083, 0.4  ,
            0.4, 0.4965;

    VectorXd initial_state(4);
    initial_state << 10,10,10,10;

    MatrixXd initial_cov(4,4);
    initial_cov << 2000, 10, 10, 10,
            10, 2000, 10, 10,
            10, 10, 2000, 10,
            10, 10, 10, 2000;

    KalmanFilter kf(stateTransitionModel, observationModel, processNoiseCovariance, observationNoiseCov,
                    initial_state, initial_cov);

    std::cout << "Gonna predict" << std::endl;

    kf.predict();

    std::cout << "x_pred: " << kf.getStatePred() << std::endl;
	
    std::cout << "P_pred =  " << kf.getCovPred() << std::endl;

    std::cout << "Check" << std::endl;

	getchar();

    VectorXd meas(2);
    meas << 10, 5;

    kf.update(meas);

	std::cout << "x_post: " << kf.getStatePost() << std::endl;
	std::cout << "P_post: " << kf.getCovPost() << std::endl;

	getchar();


	MatrixXd S(2, 2);
	S << 2, 1,
		1, 2;



	VectorXd x(2);
	x << 3, 1;


	VectorXd u(2);
	u << 4, 2;


	std::cout << Comparator::mahalanobis(x, u, S);

	std::cout << "BHATACHARYYA!" << std::endl;

	VectorXd h1(4);
	h1 << 2, 1, 3, 4;
	VectorXd h2(4);
	h2 << 3, 2, 3, 4;


	std::cout << Comparator::hellinger(h1, h2);



	getchar();

	std::cout << "Testing out the Hungarian Associator" << std::endl;


	//Build a list of people trackers
	vector<shared_ptr<BaseTracker<Person3dBVT>>> trackerList;

	//Person 1
	VectorXd position_p1(2);
	position_p1 << 1, 1;
	shared_ptr<Person3dBVT> person1(new Person3dBVT(position_p1, VectorXd(), MatrixXd()));
	shared_ptr<PeopleTrackerWithMMAE<Person3dBVT>> personTracker1(new PeopleTrackerWithMMAE<Person3dBVT>(person1));
	trackerList.push_back(personTracker1);

	//Person 2
	VectorXd position_p2(2);
	position_p2 << 3, 1;
	shared_ptr<Person3dBVT> person2(new Person3dBVT(position_p2, VectorXd(), MatrixXd()));
	shared_ptr<PeopleTrackerWithMMAE<Person3dBVT>> personTracker2(new PeopleTrackerWithMMAE<Person3dBVT>(person2));
	trackerList.push_back(personTracker2);

	//Person 3
	VectorXd position_p3(2);
	position_p3 << 4, 5;
	shared_ptr<Person3dBVT> person3(new Person3dBVT(position_p3, VectorXd(), MatrixXd()));
	shared_ptr<PeopleTrackerWithMMAE<Person3dBVT>> personTracker3(new PeopleTrackerWithMMAE<Person3dBVT>(person3));
	trackerList.push_back(personTracker3);

	//Person 4
	VectorXd position_p4(2);
	position_p4 << 1, 5;
	shared_ptr<Person3dBVT> person4(new Person3dBVT(position_p4, VectorXd(), MatrixXd()));
	shared_ptr<PeopleTrackerWithMMAE<Person3dBVT>> personTracker4(new PeopleTrackerWithMMAE<Person3dBVT>(person4));
	trackerList.push_back(personTracker4);


	//Build a list of detections
	vector<shared_ptr<Detection<Person3dBVT>>> detectionList;


	//Detect 1
	VectorXd pdet1(2);
	pdet1 << 1.5, 1.5;
	shared_ptr<Person3dBVT> det1(new Person3dBVT(pdet1, VectorXd(), MatrixXd()));
	shared_ptr<Detection<Person3dBVT>> detection1(new Detection<Person3dBVT>(det1, "Camera"));
	detectionList.push_back(detection1);

	//Detect 2
	VectorXd pdet2(2);
	pdet2 << 3.5, 1.5;
	shared_ptr<Person3dBVT> det2(new Person3dBVT(pdet2, VectorXd(), MatrixXd()));
	shared_ptr<Detection<Person3dBVT>> detection2(new Detection<Person3dBVT>(det2, "Camera"));
	detectionList.push_back(detection2);

	//Detec 3
	VectorXd pdet3(2);
	pdet3 << 4.5, 5.5;
	shared_ptr<Person3dBVT> det3(new Person3dBVT(pdet3, VectorXd(), MatrixXd()));
	shared_ptr<Detection<Person3dBVT>> detection3(new Detection<Person3dBVT>(det3, "Camera"));
	detectionList.push_back(detection3);

	//Detec 4
	VectorXd pdet4(2);
	pdet4 << 1.5, 5.5;
	shared_ptr<Person3dBVT> det4(new Person3dBVT(pdet4, VectorXd(), MatrixXd()));
	shared_ptr<Detection<Person3dBVT>> detection4(new Detection<Person3dBVT>(det4, "Camera"));
	detectionList.push_back(detection4);



	//Build an Hungarian associator of people's positions with the euclidean metric

	HungarianAssociator<Person3dBVT> associator(Person3dBVT::COMP_POSITION, Comparator::METRIC_EUCLIDEAN);
	AssociationList<Person3dBVT> assocList = associator.associateData(trackerList, detectionList);
	
	vector<Association<Person3dBVT>> success = assocList.getSuccessfulAssociations();
	vector<shared_ptr<Detection<Person3dBVT>>> unDetections = assocList.getUnassociatedDetections();


	cout << "------------------------------ Sucessful associations ------------------------------" << endl;

	for (Association<Person3dBVT> &ass : success)
	{
		cout << "Associated: " << endl;
		cout << ass.getDetectionPTR()->getObjPTR()->getPosition() << endl;
		cout << "with: " << endl;
		cout << ass.getTrackerPTR()->getObjPTR()->getPosition() << endl;
	}

	cout << "------------------------------ Unsucessful detection ------------------------------" << endl;
	
	for (shared_ptr<Detection<Person3dBVT>> dt : unDetections)
	{

		cout << dt->getObjPTR()->getPosition();

	}

	getchar();

    return 0;
}

