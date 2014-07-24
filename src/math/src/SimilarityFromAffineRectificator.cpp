#include "SimilarityFromAffineRectificator.h"
#include <iostream>

using namespace std;

namespace math
{
	SimilarityFromAffineRectificator::SimilarityFromAffineRectificator(const vector<pair<VectorXd, VectorXd>>& orthoLines)
	{
		buildTransformation(orthoLines);
	}
	
	shared_ptr<MatrixXd> SimilarityFromAffineRectificator::buildTransformation
		(const vector<pair<VectorXd, VectorXd>>& orthoLines)
	{
		Vector3d l0 = orthoLines[0].first;
		Vector3d l1 = orthoLines[0].second;
		Vector3d l2 = orthoLines[1].first;
		Vector3d l3 = orthoLines[1].second;
		
		MatrixXd A(2, 2);
		A <<
			l0[0] * l1[0], l0[0] * l1[1] + 	l0[1] * l1[0],
			l2[0] * l3[0], l2[0] * l3[1] + l2[1] * l3[0];
		
		VectorXd b(2);
		b << -l0[1] * l1[1], -l2[1] * l3[1];
		
		cout << "Linear system A matrix: " << endl << A << endl << endl << "b vector: " << b << endl << endl;
		
		VectorXd x = A.colPivHouseholderQr().solve(b);
		cout << "Linear system x vector: " << endl << x << endl << endl;
		
		MatrixXd KKt(2, 2);
		KKt <<
			x[0], x[1],
			x[1], 1.;
		cout << "KKt matrix: " << endl << KKt << endl << endl;
		
		MatrixXd K = LLT<MatrixXd>(KKt).matrixU();
		cout << "K matrix: " << endl << K << endl << endl;
		
		MatrixXd affineTransf(3, 3);
		affineTransf << K(0, 0)	, K(0, 1)	, 0.,
						0.		, K(1, 1)	, 0.,
						0.		, 0.		, 1.;
		cout << "Affine transformation: " << endl << affineTransf << endl << endl;
		
		m_transformation = make_shared<MatrixXd>(affineTransf.inverse());
		cout << "Rectification transformation: " << endl << (*m_transformation) << endl << endl;
		
		sanityCheck(orthoLines, 2.0e-1);
		
		return m_transformation;
	}
	
	void SimilarityFromAffineRectificator::sanityCheck(const vector<pair<VectorXd, VectorXd>>& orthoLines, double error)
	{
		cout << endl << endl << "STARTING TRANSFORMATION SANITY CHECK" << endl << endl;
		for (pair<VectorXd, VectorXd> orthoLine : orthoLines)
		{
			MatrixXd inverseTransposed = m_transformation->inverse().transpose();
			
			VectorXd l0 = orthoLine.first;
			VectorXd l1 = orthoLine.second;
			
			VectorXd transformedL0 = inverseTransposed * l0;
			transformedL0 /= transformedL0[2];
			VectorXd transformedL1 = inverseTransposed * l1;
			transformedL1 /= transformedL1[2];
			
			VectorXd l0Vector(2);
			l0Vector << transformedL0[0], transformedL0[1];
			VectorXd l1Vector(2);
			l1Vector << transformedL1[0], transformedL1[1];
			
			double dot = l0Vector.dot(l1Vector) / (l0Vector.norm() * l1Vector.norm());
			
			stringstream ss;
			ss << "Line " << endl << l0 << endl << "Mapped to " << endl << transformedL0 << endl <<
			"Should be orthogonal to " << endl << l1 << endl << "Mapped to " << endl << transformedL1 << endl <<
			transformedL0 << endl << "Dot = " << dot << endl << endl;
			cout << ss.str();
			
			if (abs(dot) > error)
			{
				throw logic_error(ss.str());
			}
		}
		cout << "ENDING TRANSFORMATION SANITY CHECK" << endl << endl;
	}
}
