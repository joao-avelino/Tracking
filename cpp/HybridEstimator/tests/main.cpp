#include <iostream>
#include "KalmanFilter.hpp"
#include <eigen3/Eigen/Eigenvalues>

#include "Kfutils.hpp"

#include "Comparator.hpp"
#include "MMAE.hpp"
#include "HungarianAssociator.hpp"
#include "Person3dBVT.hpp"
#include "TrackerWithBVT.hpp"
#include "Association.hpp"
#include <iterator>
#include <algorithm>
#include <utility>
#include <map>

#include "SimpleTrackerManager.hpp"
#include "MultiObjectTracker.hpp"

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


	/*Test out if we are doing shallow or deep copies on kalman filters*/

    KalmanFilter kf(stateTransitionModel, observationModel, processNoiseCovariance, observationNoiseCov,
                    initial_state, initial_cov);

	shared_ptr<BaseBayesianFilter> kf2PTR = kf.clone();

    std::cout << "Gonna predict: KF1" << std::endl;

    VectorXd empty;
    empty = VectorXd();
    kf.predict(empty);

    std::cout << "x_pred: " << kf.getStatePred() << std::endl;
	
    std::cout << "P_pred =  " << kf.getCovPred() << std::endl;

    std::cout << "Check gonna update KF1" << std::endl;

	getchar();

    VectorXd meas(2);
    meas << 10, 5;

    kf.update(meas);

	std::cout << "x_post: " << kf.getStatePost() << std::endl;
	std::cout << "P_post: " << kf.getCovPost() << std::endl;

	getchar();

	cout << "Checking contents of kf2" << endl;
	/*
	std::cout << "x_pred: " << kf2PTR->getStatePred() << std::endl;
	std::cout << "P_pred =  " << kf2PTR->getCovPred() << std::endl;
	std::cout << "x_post: " << kf2PTR->getStatePost() << std::endl;
	std::cout << "P_post: " << kf2PTR->getCovPost() << std::endl;*/

	/*It does a deep copy :D*/

	getchar();

	/*Test out if we are doing deep copies or shallow copies of persons*/

	cout << "Deep or shallow copies of people" << endl;

	VectorXd testePerson(2);
	testePerson << 1, 5;
	shared_ptr<Person3dBVT> testeP1(new Person3dBVT(testePerson, VectorXd(), MatrixXd()));

	shared_ptr<Person3dBVT> testeP2 = static_pointer_cast<Person3dBVT>(testeP1->clone());

	VectorXd newPost = testeP1->getObservableStates();
	newPost(0) = 9;
	newPost(1) = 3;
	testeP1->setObservableStates(newPost);

	cout << "P1: " << testeP1->getObservableStates() << endl;
	cout << "P2: " << testeP2->getObservableStates() << endl;

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



	//getchar();

	//std::cout << "Testing out the Hungarian Associator" << std::endl;


	//Build a list of people trackers
//	vector<shared_ptr<TrackerWithBVT<Person3dBVT, MMAE>>> trackerList;
/*
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
	trackerList.push_back(personTracker3);*/

	//Person 4
//	VectorXd position_p4(2);
//	position_p4 << 1, 5;
//	shared_ptr<Person3dBVT> person4(new Person3dBVT(position_p4, VectorXd(), MatrixXd()));

	//Let's create an  MMAEItem for each received model and add it
	//to the filterBank

	//std::vector<std::shared_ptr<MMAEItem> > filterBank;

	//shared_ptr<MMAE> estim(new MMAE(filterBank));
	
//	shared_ptr<TrackerWithBVT<Person3dBVT, MMAE>> personTracker4(new TrackerWithBVT<Person3dBVT, MMAE>(person4, estim, 0.5));
//	trackerList.push_back(personTracker4);
	

	//Build a list of detections
/*	vector<shared_ptr<Detection<Person3dBVT>>> detectionList;


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
	*/


	//Build an Hungarian associator of people's positions with the euclidean metric
	/*
	HungarianAssociator<Person3dBVT, TrackerWithBVT<Person3dBVT, MMAE>> associator(Person3dBVT::COMP_POSITION, Comparator::METRIC_EUCLIDEAN);
	AssociationList<Person3dBVT, TrackerWithBVT<Person3dBVT, MMAE>> assocList = associator.associateData(trackerList, detectionList);
	
	vector<Association<Person3dBVT, TrackerWithBVT<Person3dBVT, MMAE>>> success = assocList.getSuccessfulAssociations();
	vector<shared_ptr<Detection<Person3dBVT>>> unDetections = assocList.getUnassociatedDetections();


	cout << "------------------------------ Sucessful associations ------------------------------" << endl;

	for (Association<Person3dBVT, TrackerWithBVT<Person3dBVT, MMAE>> &ass : success)
	{
		cout << "Associated: " << endl;
		cout << ass.getDetectionPTR()->getObjPTR()->getObservableStates() << endl;
		cout << "with: " << endl;
		cout << ass.getTrackerPTR()->getObjPTR()->getObservableStates() << endl;
	}

	cout << "------------------------------ Unsucessful detections ------------------------------" << endl;
	
	for (shared_ptr<Detection<Person3dBVT>> dt : unDetections)
	{

		cout << dt->getObjPTR()->getObservableStates() << endl;

	}*/


	getchar();

	cout << "------------------------------ Testing tracker manager and MoT (the managing part) ------------------------------" << endl;

	//Create a nickname for the difficult templates
	typedef TrackerWithBVT<Person3dBVT, MMAE> MMAEpersonTracker;
	typedef SimpleTrackerManager<Person3dBVT, MMAEpersonTracker> TrackerManager;
	typedef HungarianAssociator<Person3dBVT, TrackerWithBVT<Person3dBVT, MMAE>> PeopleHungarianAssociator;
	typedef MultiObjectTracker<MMAEpersonTracker, TrackerManager, PeopleHungarianAssociator, Person3dBVT> MoT;
	typedef AssociationList<Person3dBVT, MMAEpersonTracker> PeopleAssociations;
    typedef Association<Person3dBVT, MMAEpersonTracker> PersonTrackerAssociation;

	//Create an object
	shared_ptr<Person3dBVT> dummy_person(new Person3dBVT(VectorXd::Zero(2), VectorXd::Zero(10), MatrixXd::Zero(2, 2)));

	//Create a observableStates estimator
	std::vector<std::shared_ptr<MMAEItem> > filterBank;
	std::shared_ptr<MMAEItem> mmaeitem_ptr(new MMAEItem(kf2PTR, "Just one filter lol"));


	filterBank.push_back(mmaeitem_ptr);

	std::shared_ptr<MMAE> posEstimator(new MMAE(filterBank));

	//Create a Tracker
    MMAEpersonTracker *trackerPTR = new MMAEpersonTracker(dummy_person, posEstimator, 0.5);


	//Create an associator
	PeopleHungarianAssociator hung(Person3dBVT::COMP_POSITION, Comparator::METRIC_EUCLIDEAN);

	//Create a tracker manager
	TrackerManager trkMgr(5);

	//Create a MoT
	MoT multipersontracker(trkMgr, hung, trackerPTR);


	//Make up 5 associations and give them to the tracker manager

	PeopleAssociations assocList;

	//Build a list of detections
	vector<shared_ptr<Detection<Person3dBVT>>> detectionList;
	vector<shared_ptr<Detection<Person3dBVT>>> detectionList2;

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



	//One detection
	detectionList2.push_back(detection1);

	//No detection
	vector<shared_ptr<Detection<Person3dBVT>>> detectionListEmpty;

	assocList.addUnassociatedDetection(detectionList);

	//Check the trackers vector in the beginning
	cout << "---- Before the existance of any tracker ----" << endl;

	auto trkVect = multipersontracker.getTrackersVector();
	
	for (auto& trk : trkVect)
	{
		cout << trk->getObjPTR()->getObservableStates() << endl;
	}

	getchar();

	//Give the detections
	

	multipersontracker.processDetections(detectionList);


	//Check if it created the new tracks
	cout << "---- Check if trackers were created ----" << endl;
	trkVect = multipersontracker.getTrackersVector();

	for (auto& trk : trkVect)
	{
		cout << trk->getObjPTR()->getObservableStates() << endl;
	}
	
	getchar();


	//Now give only the first detection

	multipersontracker.processDetections(detectionList2);
	

	//Give no detections 4 times in a row
	
	multipersontracker.processDetections(detectionListEmpty);
	multipersontracker.processDetections(detectionListEmpty);
	multipersontracker.processDetections(detectionListEmpty);
	multipersontracker.processDetections(detectionListEmpty);


	//Check if only the first tracker is alive
	cout << "---- Check if only the first tracker is alive ----" << endl;
	trkVect = multipersontracker.getTrackersVector();

	for (auto& trk : trkVect)
	{
		cout << trk->getObjPTR()->getObservableStates() << endl;
	}

	getchar();

	//Give no detections again

	multipersontracker.processDetections(detectionListEmpty);



	//Check if only the first tracker is alive
	cout << "---- Check that no tracker is alive ----" << endl;
	trkVect = multipersontracker.getTrackersVector();

	for (auto& trk : trkVect)
	{
		cout << trk->getObjPTR()->getObservableStates() << endl;
	}

	getchar();

	//Give 1 detection
	
	multipersontracker.processDetections(detectionList2);


	//Give no detections 4 more times

	multipersontracker.processDetections(detectionListEmpty);
	multipersontracker.processDetections(detectionListEmpty);
	multipersontracker.processDetections(detectionListEmpty);
	multipersontracker.processDetections(detectionListEmpty);


	cout << "---- Check that only one tracker is alive----" << endl;

	trkVect = multipersontracker.getTrackersVector();

	for (auto& trk : trkVect)
	{
		cout << trk->getObjPTR()->getObservableStates() << endl;
	}

	getchar();

	//Give no detections again and check that no trackers are alive

	multipersontracker.processDetections(detectionListEmpty);

	cout << "---- Check that no trackers are alive ----" << endl;


	trkVect = multipersontracker.getTrackersVector();

	for (auto& trk : trkVect)
	{
		cout << trk->getObjPTR()->getObservableStates() << endl;
	}

	


	getchar();

}

