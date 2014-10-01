#include "CameraMatrixDlt.h"
#include "TriangulationDlt.h"

#include <iostream>

using namespace std;

namespace math
{
	CameraMatrixDlt::CameraMatrixDlt( vector< Correspondence >& correspondences, shared_ptr< MatrixXd > K0,
											shared_ptr< MatrixXd > K1 )
	: DltBase( correspondences ),
	m_K0( K0 ),
	m_K1( K1 )
	{}
	
	void CameraMatrixDlt::normalize(){} ;
	
	MatrixXd CameraMatrixDlt::createLinearSystem() const
	{
		int sampleSize = m_sample->size();
		MatrixXd A( sampleSize, 9 );
		
		for( int i = 0; i < sampleSize; ++i)
		{
			Block< MatrixXd > block = A.block(i, 0, 1, 9);
			VectorXd v0 = (*m_sample)[i].first;
			VectorXd v1 = (*m_sample)[i].second;
			
			block 	<< v1[ 0 ] * v0[ 0 ], v1[ 0 ] * v0[ 1 ], v1[ 0 ], v1[ 1 ] * v0[ 0 ], v1[ 1 ] * v0[ 1 ], v1[ 1 ],
					   v0[ 0 ], v0[ 1 ], 1.;
		}
		
		cout << "Linear System Matrix: " << endl << A << endl << endl;
		
		return A;
	}
	
	void CameraMatrixDlt::applyRestrictions()
	{
		JacobiSVD< MatrixXd, FullPivHouseholderQRPreconditioner > svd( *m_resultH, ComputeFullU | ComputeFullV );
		VectorXd singularValues = svd.singularValues();
		double singularValue = ( singularValues[ 0 ] + singularValues[ 1 ] ) * 0.5;
		DiagonalMatrix< double, 3, 3 > D( singularValue, singularValue, 0. );
		MatrixXd DMat = D.toDenseMatrix();
		MatrixXd restricted = svd.matrixU() * DMat * svd.matrixV().transpose();
		
		cout << "========== E Restricion Appliance =============: " << endl
			 << "Singular diagonal: " << endl << DMat << endl << endl
			 << "U:" << endl << svd.matrixU() << endl << endl
			 << "V:" << endl << svd.matrixV() << endl << endl
			 << "E: " << endl << restricted << endl << endl
			 << "Determinant: " << restricted.determinant() << endl << endl
			 << "========== Restriction Appliance End =============" << endl << endl;
			 
		*m_resultH = restricted;
	}
	
	void CameraMatrixDlt::denormalize(){} ;
	
	bool CameraMatrixDlt::checkP1( const MatrixXd& P0, const MatrixXd& P1 ) const
	{
		VectorXd p0 = ( *m_sample )[ 0 ].first;
		VectorXd p1 = ( *m_sample )[ 0 ].second;
		
		vector< Correspondence > pair( 1 );
		pair[ 0 ] = Correspondence( p0, p1 );
		
		TriangulationDlt dlt( pair, P0, P1 );
		dlt.solve();
		VectorXd point3D = dlt.getPoint3D();
		
		VectorXd reprojectedImg0 = P0 * point3D;
		VectorXd reprojectedImg1 = P1 * point3D;
		
		cout << "========== P1 Check =============: " << endl
			 << "P0:" << endl << P0 << endl << endl
			 << "P1:" << endl << P1 << endl << endl
			 << "x: " << endl << p0 << endl << endl 
			 << "x': " << endl << p1 << endl << endl
			 << "3d point: " << endl << point3D << endl << endl
			 << "Reprojected x: " << endl << reprojectedImg0 << endl << endl
			 << "Reprojected x': " << endl << reprojectedImg1 << endl << endl
			 << "========== P1 Check End =============: " << endl << endl;
		
		return reprojectedImg0[2] > 0. && reprojectedImg1[2] > 0.;
	}
	
	MatrixXd CameraMatrixDlt::computeP( MatrixXd& E )
	{
		JacobiSVD< MatrixXd, FullPivHouseholderQRPreconditioner > svd( E, ComputeFullU | ComputeFullV );
		MatrixXd U = svd.matrixU();
		MatrixXd Vt = svd.matrixV().transpose();
			 
		
		MatrixXd W( 3 , 3);
		W << 0., -1., 0.,
			 1., 0., 0.,
			 0., 0., 1.;
		
		VectorXd u3 = U.col(2);
		
		cout << "========== P Computation =============: " << endl
			 << "E: " << endl << E << endl << endl
			 << "U: " << endl << U << endl << endl
			 << "D: " << endl << svd.singularValues() << endl << endl
			 << "Vt: " << endl << Vt << endl << endl
			 << "W: " << endl << W << endl << endl
			 << "u3 :" << u3 << endl << endl;
		
		m_P0 = make_shared< MatrixXd >( MatrixXd::Identity( 3, 4 ) ); // Just K0: no translation or rotation.
		
		MatrixXd P1noT = U * W * Vt; // P without translation part.
		
		MatrixXd P1( 3, 4 );
		P1.block( 0, 0, 3, 3 ) = P1noT;
		P1.block( 0, 3, 3, 1 ) = u3;
		
		cout << "1st sol: " << endl;
		
		MatrixXd bestP1;
		int numCorrectSolutions = 0;
		if( checkP1( *m_P0, P1 ) )
		{
			++numCorrectSolutions;
			bestP1 = P1;
			//return P1;
		}
		
		P1.block( 0, 3, 3, 1 ) = -u3;
		
		cout << "2nd sol: " << endl;
		
		if( checkP1( *m_P0, P1 ) )
		{
			++numCorrectSolutions;
			bestP1 = P1;
			//return P1;
		}
		
		P1noT = U * W.transpose() * Vt;
		P1.block( 0, 0, 3, 3 ) = P1noT;
		P1.block( 0, 3, 3, 1 ) = u3;
		
		cout << "3rd sol: " << endl;
		
		if( checkP1( *m_P0, P1 ) )
		{
			++numCorrectSolutions;
			bestP1 = P1;
			//return P1;
		}
		
		P1.block( 0, 3, 3, 1 ) = -u3;
		
		cout << "4th sol: " << endl;
		
		if( checkP1( *m_P0, P1 ) )
		{
			++numCorrectSolutions;
			bestP1 = P1;
			//return P1;
		}
		
		cout << "========== P Computation End ============= " << endl << endl
			 << "Number of correct solutions: " << numCorrectSolutions << endl << endl;
		
		if( numCorrectSolutions == 0 ) throw runtime_error( "None of the results for P1 was accepted!" );
		if( numCorrectSolutions > 1 ) throw runtime_error( "More than one solution found." );
		
		return bestP1;
	}
	
	void CameraMatrixDlt::onDenormalizationEnd()
	{
		cout << "========== After denormalizing E =============: " << endl
			 << "E: " << endl << *m_resultH << endl << endl;
			 
		for( Correspondence correspondence : *m_sample )
		{
			VectorXd p0 = correspondence.first;
			VectorXd p1 = correspondence.second;
			cout << correspondence
				 << "Condition x'^T * E * x ( should be 0. ):" << endl << ( p1.transpose() * (*m_resultH) ) * p0 << endl << endl;
		}
			 
		cout << "========== Denormalization End =============" << endl << endl;
		*m_resultH = computeP( *m_resultH );
	}
	
	int CameraMatrixDlt::scoreSolution( shared_ptr< vector< Correspondence > > allCorrespondences, const double& threshold )
	{
		m_points3D = make_shared< vector< VectorXd > >();
		
		vector< double > distances;
		for( Correspondence correspondence : *allCorrespondences )
		{
			vector< Correspondence > pair;
			pair.push_back( correspondence );
			
			//cout << "Score sol: P0: " << endl << *m_P0 << endl << endl << "P1: " << endl << *m_resultH << endl << endl;
			
			TriangulationDlt dlt( pair, *m_P0, *m_resultH );
			dlt.solve();
			VectorXd point3D = dlt.getPoint3D();
			m_points3D->push_back( point3D );
			
			VectorXd v0Expected = ( *m_P0 ) * point3D;
			v0Expected = v0Expected / v0Expected[ 2 ];
			VectorXd v1Expected = ( *m_resultH ) * point3D;
			v1Expected = v1Expected / v1Expected[ 2 ];
			
			VectorXd v0Diff = v0Expected - correspondence.first;
			VectorXd v1Diff = v1Expected - correspondence.second;
			
			double distance = v0Diff.norm() + v1Diff.norm();
			
			distances.push_back( distance );
		}
		
		// Third, returns the percentage of outliers in the correspondence set.
		int inliers = 0;
		for( int i = 0; i < distances.size(); ++i)
		{
			double distance = distances[ i ]; 
			
			if( distance <= threshold )
			{
				++inliers;
			}
		}
		
		//cout << "Homography: " << endl << *m_resultH << endl << endl
		//	 << "Inliers: " << inliers << " of " << numPoints << endl << endl;
		
		return inliers;
	}
	
	MatrixXd CameraMatrixDlt::getP0() const { return *m_P0; }
	
	shared_ptr< vector< VectorXd > > CameraMatrixDlt::getPoints3D() const { return m_points3D; }
}