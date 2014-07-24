#include "SimilarityFromProjRectificator.h"
#include <iostream>

namespace math
{
	SimilarityFromProjRectificator::SimilarityFromProjRectificator(const vector<pair<VectorXd, VectorXd>>& orthoLines)
	{
		buildTransformation(orthoLines);
	}

	shared_ptr<MatrixXd> SimilarityFromProjRectificator::buildTransformation(const vector<pair<VectorXd, VectorXd>>& orthoLines)
	{
		if (orthoLines.size() != 5)
		{
			throw logic_error("Number of selected pairs is not 5." + orthoLines.size());
		}
		
		MatrixXd A(5, 5);
		VectorXd b(5);
		
		for (int i = 0; i < 5; ++i)
		{
			VectorXd l = orthoLines[i].first;
			VectorXd m = orthoLines[i].second;
			
			Block<MatrixXd> block = A.block(i, 0, 1, 5);
			
			// The two matrix lines associates with the current point correlation.
			block <<
			l[0] * m[0],
			(l[0] * m[1] + l[1] * m[0]) * 0.5,
			l[1] * m[1],
			(l[0] * m[2] + l[2] * m[0]) * 0.5,
			(l[1] * m[2] + l[2] * m[1]) * 0.5;
			
			b[i] = -l[2] * m[2];
		}
		cout << "Linear system A matrix: " << endl << A << endl << endl << "b vector: " << b << endl << endl;
		
		VectorXd x = A.colPivHouseholderQr().solve(b);
		cout << "Linear system x vector: " << endl << x << endl << endl;
		
		MatrixXd KKt(2, 2);
		KKt <<
			x[0]		, x[1] * 0.5,
			x[1] * 0.5	, x[2];
		cout << "KKt matrix: " << endl << KKt << endl << endl;
		
		MatrixXd K = LLT<MatrixXd>(KKt).matrixU();
		cout << "K matrix: " << endl << K << endl << endl;
		
		VectorXd KKtv(2);
		KKtv << x[3] * 0.5, x[4] * 0.5;
		VectorXd v = KKt.inverse() * KKtv;
		cout << "v: " << endl << v << endl << endl;
		
		MatrixXd projection(3, 3);
		projection <<
			K(0, 0)	, K(0, 1)	, 0. ,
			0		, K(1, 1)	, 0. ,
			v[0]	, v[1]		, 1.;
		
		m_transformation = make_shared<MatrixXd>(projection.inverse());
		cout << "Rectification transformation: " << endl << (*m_transformation) << endl << endl;
		
		sanityCheck(orthoLines, 1.0e-1);
		
		return m_transformation;
	}

	void SimilarityFromProjRectificator::sanityCheck(const vector<pair<VectorXd, VectorXd>>& orthoLines, double error)
	{	
	}
}