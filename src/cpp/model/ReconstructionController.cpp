#include "ReconstructionController.h"
#include <Ransac.h>
#include <TriangulationDlt.h>
#include <ReconstructionRansac.h>
#include <qtextcodec.h>
#include <fstream>

namespace model
{
	ReconstructionController::ReconstructionController( vector< Correspondence >& correspondences, MatrixXd& K0, MatrixXd& K1 )
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
			HouseholderQR< MatrixXd > qr( KR1 );
			MatrixXd K = qr.matrixQR().triangularView<Upper>();
			
			camMatrices.push_back( K );
		}
		
		return camMatrices;
	}
	
	vector< map< int, Line > > ReconstructionController::readLineCorrespondence( vector< string >& lineFileNames,
																			   string& correspondenceFileName )
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
					++imgIdx;
				}
			}
			
			correspondencesPerLine.push_back( correspondentLines );
		}

		correspondenceIFS.close();
		
		return correspondencesPerLine;
	}

	shared_ptr< vector< VectorXd > > ReconstructionController::reconstruct()
	{
		MatrixXd E = m_ransac->compute();
		shared_ptr< vector< VectorXd > > points3D = m_ransac->getSolver()->getPoints3D();
		return points3D;
	}
}