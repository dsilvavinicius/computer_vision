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
		cout << "Linear system A matrix: " << endl << A << endl << endl << "b vector: " << b << endl << endl;
		
		for (int i = 0; i < 5; ++i)
		{
			VectorXd l = orthoLines[i].first;
			VectorXd m = orthoLines[i].second;
			
			Block<MatrixXd> block = A.block(i, 0, 1, 5);
			
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
		
		MatrixXd CInfImage(3, 3);
		CInfImage <<
			x[0]		, x[1] * 0.5, x[3] * 0.5,
			x[1] * 0.5	, x[2]		, x[4] * 0.5,
			x[3] * 0.5	, x[4] * 0.5, 1.;
		cout << "CInfImage matrix: " << endl << CInfImage << endl << endl;
		
		JacobiSVD<MatrixXd> svd(CInfImage, ComputeFullU);
		MatrixXd H = svd.matrixU();
		cout << "H matrix: " << endl << H << endl << endl << "Singular values: " << svd.singularValues()
		<< endl << endl;
		
		m_transformation = make_shared<MatrixXd>(H.inverse());
		cout << "Rectification transformation: " << endl << (*m_transformation) << endl << endl;
		
		//sanityCheck(orthoLines, 3.0e-1);
		
		return m_transformation;
	}

	void SimilarityFromProjRectificator::sanityCheck(const vector<pair<VectorXd, VectorXd>>& orthoLines, double error)
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