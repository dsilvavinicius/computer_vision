#include <iostream>
#include "math/header/AssistedSimilarityFromProjRectificator.h"

namespace math
{
	AssistedSimilarityFromProjRectificator::AssistedSimilarityFromProjRectificator(const vector<pair<VectorXd, VectorXd>>& correlations)
	{
		buildTransformation(correlations);
	}

	shared_ptr<MatrixXd> AssistedSimilarityFromProjRectificator::buildTransformation
		(const vector<pair<VectorXd, VectorXd>>& correlations)
	{
		MatrixXd A(8, 8);
		VectorXd b(8);
		for (int i = 0; i < 4; ++i)
		{
			VectorXd from = correlations[i].first;
			VectorXd to = correlations[i].second;
			
			cout << "Correlation pair: " << endl << "(" << from << ")" << endl << "(" << to << ")" << endl;
			
			Block<MatrixXd> block = A.block(i * 2, 0, 2, 8);
			
			// The two matrix lines associates with the current point correlation.
			block << 	from[0], from[1],	1, 0.,		0,		 0, -from[0] * to[0], -from[1] * to[0],
						0,		 0,			0, from[0], from[1], 1, -from[0] * to[1], -from[1] * to[1];
			
			b[i * 2]	= to[0];
			b[i*2 + 1]	= to[1];
		}
		cout 	<< "Linear system A matrix: " << endl << A << endl << endl
				<< "Linear system b vector: " << endl << b << endl << endl;
		
		VectorXd x = A.colPivHouseholderQr().solve(b);		
		cout << "Linear system x vector: " << endl << x << endl << endl;
		
		m_transformation = make_shared<MatrixXd>(3, 3);
		(*m_transformation) << 	x[0], x[1], x[2],
								x[3], x[4], x[5],
								x[6], x[7], 1.;
		cout << "Rectification transformation: " << endl << (*m_transformation) << endl << endl;
		
		//sanityCheck(correlations, 1.0e-1);
		
		return m_transformation;
	}
	
	void AssistedSimilarityFromProjRectificator::sanityCheck(const vector<pair<VectorXd, VectorXd>> &correlations, double error)
	{
		cout << endl << endl << "STARTING TRANSFORMATION SANITY CHECK" << endl << endl;
		for (int i = 0; i < correlations.size(); ++i)
		{
			VectorXd from = correlations[i].first;
			VectorXd to = correlations[i].second;
			
			VectorXd transformedPoint = (*m_transformation) * from;
			transformedPoint /= transformedPoint[2];
			
			stringstream ss;
			ss << "Point " << endl << from << endl << "Mapped to " << endl << transformedPoint << endl <<
			"Should be mapped to " << endl << to << endl << endl;
			cout << ss.str();
			
			if (transformedPoint[0] > to[0] + error || transformedPoint[0] < to[0] - error ||
				transformedPoint[1] > to[1] + error || transformedPoint[1] < to[1] - error ||
				transformedPoint[2] > to[2] + error || transformedPoint[2] < to[2] - error)
			{
				throw logic_error(ss.str());
			}
		}
		
		cout << "ENDING TRANSFORMATION SANITY CHECK" << endl << endl;
	}
}