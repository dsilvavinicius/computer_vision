#include "AffineFromProjRectificator.h"
#include <iostream>

using namespace std;

namespace math
{
	AffineFromProjRectificator::AffineFromProjRectificator(vector<pair<VectorXd, VectorXd>> parallelPairs)
	{
		buildTransformation(parallelPairs);
	}

	shared_ptr<MatrixXd> AffineFromProjRectificator::buildTransformation
		(const vector<pair<VectorXd, VectorXd>>& parallelPairs)
	{
		// Conversion to Vector3d because of cross products.
		vector<pair<Vector3d, Vector3d>> parallelPairsVec3 = IPointMapper::toVector3d(parallelPairs);
		Vector3d l0 = parallelPairsVec3[0].first;
		Vector3d l1 = parallelPairsVec3[0].second;
		Vector3d l2 = parallelPairsVec3[1].first;
		Vector3d l3 = parallelPairsVec3[1].second;
		
		Vector3d intersection0 = l0.cross(l1);
		intersection0 /= intersection0[2];
		Vector3d intersection1 = l2.cross(l3);
		intersection1 /= intersection1[2];
		
		Vector3d lineAtInf = intersection0.cross(intersection1);
		lineAtInf /= lineAtInf[2];
		
		cout << "Intersection0: " << endl << intersection0 << endl << "Intersection1: " << endl << intersection1 << endl
		<< "LineAtInf: " << lineAtInf << endl << endl; 
		
		m_transformation = make_shared<MatrixXd>(3, 3);
		(*m_transformation) <<
			1.f			, 0.f			, 0.f,
			0.f			, 1.f			, 0.f,
			lineAtInf[0], lineAtInf[1]	, lineAtInf[2];
		
		cout << "Built transformation: " << endl << (*m_transformation) << endl << endl;
			
		//sanityCheck(parallelPairs, lineAtInf, 1.0e-1);	
		
		return m_transformation;
	}
	
	void AffineFromProjRectificator::sanityCheck(vector<pair<VectorXd, VectorXd>> parallelLines, Vector3d lineAtInf,
										 double error)
	{
		cout << endl << endl << "STARTING TRANSFORMATION SANITY CHECK" << endl << endl;
		for (int i = 0; i < parallelLines.size(); ++i)
		{
			VectorXd l0 = parallelLines[i].first;
			VectorXd l1 = parallelLines[i].second;
			
			MatrixXd inverseTransposed = (*m_transformation).inverse().transpose();
			
			VectorXd transformedL0 = inverseTransposed * l0;
			transformedL0 /= transformedL0[2];
			
			VectorXd transformedL1 = inverseTransposed * l1;
			transformedL1 /= transformedL1[2];
			
			double dot = transformedL0.dot(transformedL1) / (transformedL0.norm() * transformedL1.norm());
			
			stringstream ss;
			ss << "Line " << endl << l0 << endl << "Mapped to " << endl << transformedL0 << endl <<
			"Should be parallel to " << endl << l1 << endl << "Mapped to " << endl << transformedL1 << endl <<
			transformedL0 << endl << "Dot = " << dot << endl << endl;
			
			cout << ss.str();
			
			if (dot > 1. + error || dot < 1. - error)
			{
				throw logic_error(ss.str());
			}
		}
		
		MatrixXd inverseTransposed = m_transformation->inverse().transpose();
		Vector3d affineLineAtInf(3);
		affineLineAtInf << 0., 0., 1.f;
		
		Vector3d transformedLineAtinf = inverseTransposed * lineAtInf;
		transformedLineAtinf /= transformedLineAtinf[2];
		
		if (abs(affineLineAtInf[0] - transformedLineAtinf[0]) > error ||
			abs(affineLineAtInf[1] - transformedLineAtinf[1]) > error ||
			abs(affineLineAtInf[2] - transformedLineAtinf[2]) > error
		)
		{
			stringstream ss;
			ss << "Line at infinity " << endl << lineAtInf << endl << "was not mapped correctly to " << endl
			<< affineLineAtInf << endl << endl;
			throw logic_error(ss.str());
		}
		
		cout << "ENDING TRANSFORMATION SANITY CHECK" << endl << endl;
	}
}
