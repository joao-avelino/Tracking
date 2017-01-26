#ifndef HUNGARIANASSOCIATOR_HPP
#define HUNGARIANASSOCIATOR_HPP
#include "BaseDataAssociator.hpp"
#include "HungarianFunctions.hpp"


template <class Obj> class HungarianAssociator :
	public BaseDataAssociator<Obj>
{
public:
	HungarianAssociator<Obj>(int mode, int metric) : mode(mode), metric(metric) {};
	~HungarianAssociator() {};
	 

	AssociationList<Obj> associateData(vector<shared_ptr<BaseTracker<Obj> > > &trackPTRvec, vector<shared_ptr<Detection<Obj> > > &detecPTRvec);

protected:
	int mode;
	int metric;

};



#endif // HUNGARIANASSOCIATOR_HPP

template<class Obj>
AssociationList<Obj> HungarianAssociator<Obj>::associateData(vector<shared_ptr<BaseTracker<Obj>>>& trackPTRvec, vector<shared_ptr<Detection<Obj>>>& detecPTRvec)
{
	int nTrackers = trackPTRvec.size();
	int nDetections = detecPTRvec.size();
	AssociationList<Obj> associationList;

	//If the number of trackers and the number of detections is greater than zero, perform the Hungarian Algorithm
	if (nTrackers > 0 && nDetections > 0)
	{

		//Build the matrix
		double *distMatrixIn = (double*)malloc(sizeof(double)*nTrackers*nDetections);
		double *assignment = (double*)malloc(sizeof(double)*nDetections);
		double *cost = (double*)malloc(sizeof(double)*nDetections);


		//Fill the matrix
		/*  _           _
		*  |  l1c1 l1c2  |
		*  |  l2c1 l2c2  |
		*  |_ l3c1 l3c2 _|
		*
		*  [l1c1, l2c1, l3c1, l1c2,l2c2,l3c2]
		*/

		//Each detection -> a row
		int row = 0;
		for (shared_ptr<Detection<Obj>> ptrDetect : detecPTRvec)
		{

			//Each tracker -> a column
			int col = 0;
			for (shared_ptr<BaseTracker<Obj>> ptrTrack : trackPTRvec)
			{
				distMatrixIn[row + col*nDetections] = ptrTrack->compareWith((*ptrDetect), this->mode, this->metric);
				col++;
			}

			row++;

		}

		assignmentoptimal(assignment, cost, distMatrixIn, nDetections, nTrackers);

		//assignment vector positions represents the detections and the value in each position represents the assigned tracker
		//if there is no possible association, then the value is -1
		//cost vector doesn't mean anything at this point...

		//Let's make a list of successful associations, one for unassociated trackers and one for unassociated detections



		for (int i = 0; i < nDetections; i++)
		{
			//Check if the detecion is assigned
			if (assignment[i] != -1)
			{
				Association<Obj> assoc(trackPTRvec.at(assignment[i]), detecPTRvec.at(i));
				associationList.addSuccessfulAssociation(assoc);
			}
			else
			{
				associationList.addUnassociatedDetection(detecPTRvec.at(i));
			}
		}


		free(distMatrixIn);
		free(assignment);
		free(cost);
	}

	return associationList;
}
