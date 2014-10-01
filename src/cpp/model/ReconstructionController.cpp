#include "ReconstructionController.h"
#include <Ransac.h>
#include <TriangulationDlt.h>
#include <ReconstructionRansac.h>
#include <qtextcodec.h>
#include <fstream>

namespace model
{
	ReconstructionController::ReconstructionController( vector< Correspondence >& correspondences, const MatrixXd& K0,
														const MatrixXd& K1 )
	: m_correspondences( make_shared< vector< Correspondence > >( correspondences ) )
	{
		if( correspondences.size() < 8 ) { throw logic_error("8 correspondences needed to compute essential matrix."); }
		m_ransac = make_shared< ReconstructionRansac >( make_shared< vector< Correspondence > >( correspondences ), K0, K1 );
	}
	
	vector< MatrixXd > ReconstructionController::readCalibrationMatrices( vector< string >& camFileNames )
	{
		vector< MatrixXd > camMatrices;
		
		for( string fileName : camFileNames )
		{
			ifstream ifs( fileName );
			if( ifs.fail() ) { throw runtime_error( "Cannot open file" + fileName ); }
			
			MatrixXd P(3, 4);
			ifs >> P(0, 0) >> P(0, 1) >> P(0, 2) >> P(0, 3)
				>> P(1, 0) >> P(1, 1) >> P(1, 2) >> P(1, 3)
				>> P(2, 0) >> P(2, 1) >> P(2, 2) >> P(2, 3);
			
			MatrixXd KR1 = P.block( 0, 0, 3, 3 );
			FullPivHouseholderQR< MatrixXd > qr( KR1 );
			MatrixXd K = qr.matrixQR().triangularView<Upper>();
			
			//cout << "=========== Read calibration matrix ===============" << endl
			//	 << "QR: " << endl << qr.matrixQR() << endl << endl << "K: " << endl << K << endl << endl
			//	 << "=========== Read calibration matrix end ===============" << endl << endl;
			
			camMatrices.push_back( K );
		}
		
		return camMatrices;
	}
	
	vector< map< int, VectorXd > > ReconstructionController::readPointCorrespondence( const vector< string >& pointFileNames,
																					  const string& correspondenceFileName )
	{
		int numImgs = pointFileNames.size();
		// [ 0 ] is the vector of points in img 0, [ 1 ] is the vector for points in img 1, and so on.
		vector< vector< VectorXd > > pointsPerImg( numImgs ); 
		
		for( int i = 0; i < numImgs; ++i )
		{
			vector< VectorXd > imgPoints;
			
			ifstream pointIFS( pointFileNames[i] );
			if( pointIFS.fail() ) { throw runtime_error( "Cannot open file " + pointFileNames[ i ] ); }
			
			VectorXd p( 3 );
			while( pointIFS >> p[ 0 ] >> p[ 1 ] )
			{
				pointIFS.ignore( 200, '\n' );
				
				p[ 2 ] = 1.;
				imgPoints.push_back( p );
			}
			pointIFS.close();
			
			pointsPerImg[ i ] = imgPoints;
		}
		
		vector< map< int, VectorXd > > correspondencesPerPoint;
		ifstream correspondenceIFS( correspondenceFileName );
		if( correspondenceIFS.fail() ) { throw runtime_error( "Cannot open file " + correspondenceFileName ); }
		
		string line;
		while( getline( correspondenceIFS, line ) )
		{
			map< int, VectorXd > correspondentPoints;
			
			stringstream ss( line );
			
			string token;
			int imgIdx = 0;
			while( getline( ss, token, ' ') )
			{
				while( ss.peek() == ' ' )
				{
					ss.get();
				}
				
				if ( token.compare( "*" ) )
				{
					stringstream ss( token );
					int pointIdx;
					ss >> pointIdx;
					
					//cout << "Point " << correspondencesPerPoint.size() << endl << "imgidx: " << imgIdx << endl
					//	 << "pointIdx:" << pointIdx << endl << endl;
					
					correspondentPoints[ imgIdx ] = pointsPerImg[ imgIdx ][ pointIdx ];
				}
				++imgIdx;
			}
			
			correspondencesPerPoint.push_back( correspondentPoints );
		}

		correspondenceIFS.close();
		
		return correspondencesPerPoint;
	}
	
	vector< Correspondence > ReconstructionController::restrictPointCorrespondencesToImgs(
		const vector< map< int, VectorXd > >& pointMap, const int& imgIdx0, const int& imgIdx1 )
	{
		vector< Correspondence > correspondences;
		for( map< int, VectorXd > correspondence : pointMap )
		{
			auto itP0 = correspondence.find( imgIdx0 );
			auto itP1 = correspondence.find( imgIdx1 );
			
			if( itP0 != correspondence.end() && itP1 != correspondence.end() )
			{
				VectorXd p0 = itP0->second;
				VectorXd p1 = itP1->second;
				correspondences.push_back( Correspondence( p0, p1 ) );
			}
		}
		
		return correspondences;
	}
	
	vector< map< int, Line > > ReconstructionController::readLineCorrespondence( const vector< string >& lineFileNames,
																				 const string& correspondenceFileName )
	{
		int numImgs = lineFileNames.size();
		// [ 0 ] is the vector of lines in img 0, [ 1 ] is the vector for lines in img 1, and so on.
		vector< vector< Line > > linesPerImg( numImgs ); 
		
		for( int i = 0; i < numImgs; ++i )
		{
			vector< Line > imgLines;
			
			ifstream lineIFS( lineFileNames[i] );
			if( lineIFS.fail() ) { throw runtime_error( "Cannot open file " + lineFileNames[ i ] ); }
			
			Line line;
			VectorXd p0( 3 );
			VectorXd p1( 3 );
			while( lineIFS >> p0[ 0 ] >> p0[ 1 ] >> p1[ 0 ] >> p1[ 1 ] )
			{
				lineIFS.ignore( 200, '\n' );
				
				p0[ 2 ] = 1.;
				p1[ 2 ] = 1.;
				line.first = p0;
				line.second = p1;
				imgLines.push_back( line );
			}
			lineIFS.close();
			
			linesPerImg[ i ] = imgLines;
		}
		
		vector< map< int, Line > > correspondencesPerLine;
		ifstream correspondenceIFS( correspondenceFileName );
		if( correspondenceIFS.fail() ) { throw runtime_error( "Cannot open file " + correspondenceFileName ); }
		
		string line;
		while( getline( correspondenceIFS, line ) )
		{
			map< int, Line > correspondentLines;
			
			stringstream ss( line );
			
			int lineIdx;
			string token;
			int imgIdx = 0;
			while( getline( ss, token, ' ') )
			{
				while( ss.peek() == ' ' )
				{
					ss.get();
				}
				
				if ( token.compare( "*" ) )
				{
					stringstream ss( token );
					ss >> lineIdx;
					
					correspondentLines[ imgIdx ] = linesPerImg[ imgIdx ][ lineIdx ];
				}
				
				++imgIdx;
			}
			
			correspondencesPerLine.push_back( correspondentLines );
		}

		correspondenceIFS.close();
		
		return correspondencesPerLine;
	}
	
	vector< Correspondence > ReconstructionController::lineCorrespToPointCorresp( const vector< map< int, Line > >& lineCorrespondences,
																				  const int& imgIdx0, const int& imgIdx1)
	{
		vector< Correspondence > correspondences;
		for( map< int, Line > lineCorrespondence : lineCorrespondences )
		{
			auto itL0 = lineCorrespondence.find( imgIdx0 );
			auto itL1 = lineCorrespondence.find( imgIdx1 );
			
			if( itL0 != lineCorrespondence.end() && itL1 != lineCorrespondence.end() )
			{
				Line l0 = itL0->second;
				Line l1 = itL1->second;
				correspondences.push_back( Correspondence( l0.first, l1.first ) );
				correspondences.push_back( Correspondence( l0.second, l1.second ) );
			}
		}
		
		return correspondences;
	}
	
	vector< Correspondence > ReconstructionController::readLineCorrespConvertPointCorresp( const vector< string >& lineFileNames,
																						   const string& correspondenceFileName,
																						const int& imgIdx0, const int& imgIdx1 )
	{
		vector< map< int, Line > > lineCorrespondences = readLineCorrespondence( lineFileNames, correspondenceFileName );
		return lineCorrespToPointCorresp( lineCorrespondences, imgIdx0, imgIdx1 );
	}

	shared_ptr< vector< VectorXd > > ReconstructionController::reconstruct()
	{
		MatrixXd E = m_ransac->compute( 10. );
		shared_ptr< vector< VectorXd > > points3D = m_ransac->getSolver()->getPoints3D();
		return points3D;
	}
	
	vector< Correspondence > ReconstructionController::normalizeWithCamCalibration( const vector< Correspondence >& correspondences,
																					const MatrixXd& K0, const MatrixXd& K1)
	{
		vector< Correspondence > normalizedCorrespondences;
		
		for( int i = 0; i < correspondences.size(); ++i )
		{
			Correspondence correspondence = correspondences[ i ];
			VectorXd p0 = K0.inverse() * correspondence.first; p0 = p0 / p0[ 2 ];
			VectorXd p1 = K1.inverse() * correspondence.second; p1 = p1 / p1[ 2 ];
			
			Correspondence normalized( p0, p1 );
			
			normalizedCorrespondences.push_back( normalized );
		}
		
		return normalizedCorrespondences;
	}
	
	shared_ptr< ReconstructionRansac > ReconstructionController::getRansac() { return m_ransac; }
}