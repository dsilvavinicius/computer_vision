#include "SimilarityFromAffineRectificator.h"
#include <iostream>

using namespace std;

namespace math
{
	SimilarityFromAffineRectificator::SimilarityFromAffineRectificator(vector<pair<VectorXd, VectorXd>> orthoLines)
	{
		buildTransformation(orthoLines);
	}
	
	shared_ptr<MatrixXd> SimilarityFromAffineRectificator::buildTransformation
		(vector<pair<VectorXd, VectorXd>> orthoLines)
	{
		Vector3d l0 = orthoLines[0].first;
		Vector3d l1 = orthoLines[0].second;
		Vector3d l2 = orthoLines[1].first;
		Vector3d l3 = orthoLines[1].second;
		
		MatrixXd A(2, 3);
		A <<
			l0[0] * l1[0], l0[0] * l1[1], l0[1] * l1[1],
			l2[0] * l3[0], l2[0] * l3[1], l2[1] * l3[1];
		
		VectorXd b(2);
		b << 0. , 0.;
		
		cout 	<< "Linear system A matrix: " << endl << A << endl << endl
				<< "Linear system b vector: " << endl << b << endl << endl;
		
		VectorXd x = A.colPivHouseholderQr().solve(b);		
		cout << "Linear system x vector: " << endl << x << endl << endl;
		
		m_transformation = make_shared<MatrixXd>(3, 3);
		(*m_transformation) << 	x[0], x[1]	, 0.,
								x[1], 1.	, 0.,
								0.	, 0.	, 0.;
		cout << "Rectification transformation: " << endl << (*m_transformation) << endl << endl;
		
		sanityCheck(orthoLines, 1.0e-1);
		
		return m_transformation;
	}
	
	void SimilarityFromAffineRectificator::sanityCheck(vector<pair<VectorXd, VectorXd>> orthoLines, double error)
	{
		
	}
}
