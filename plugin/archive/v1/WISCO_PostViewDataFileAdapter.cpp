#include "WISCO_PostViewDataFileAdapter.h"
#include <fstream>
namespace OpenSim {

WISCO_PostViewDataFileAdapter::WISCO_PostViewDataFileAdapter() :
    DelimFileAdapter(",", // delimiter for read
                     ","  // delimiter for write
                     ) {}

WISCO_PostViewDataFileAdapter*
WISCO_PostViewDataFileAdapter::clone() const {
    return new WISCO_PostViewDataFileAdapter{*this};
}
/*
TimeSeriesTable
WISCO_PostViewDataFileAdapter::read(const std::string& fileName) {
    auto abs_table = WISCO_PostViewDataFileAdapter{}.extendRead(fileName).at(_table);
    return static_cast<TimeSeriesTable&>(*abs_table);
}
*/
void 
WISCO_PostViewDataFileAdapter::write(const TimeSeriesTable& intable,
                        const std::string& fileName, const SimTK::PolygonalMesh mesh, const int ver_start_ind) {

	InputTables absTables{};
    absTables.emplace(_table, &intable);
	//WISCO_PostViewDataFileAdapter{}.extendWrite(tables, fileName);

	//Added to write in PostView Data file format
	const std::string _table{ "table" };
	const std::string _endHeaderString{ "endheader" };
	const std::string _timeColumnLabel{ "time" };
	const std::string _delimiterWrite{ "," };

	// Check for errors in table
	//==========================
	OPENSIM_THROW_IF(absTables.empty(), NoTableFound);

	const TimeSeriesTable* table{};

	try {
		auto abs_table = absTables.at(_table);
		table = dynamic_cast<const TimeSeriesTable*>(abs_table);
	}
	catch (std::out_of_range&) {
		OPENSIM_THROW(KeyMissing,
			_table);
	}
	catch (std::bad_cast&) {
		OPENSIM_THROW(IncorrectTableType);
	}

	OPENSIM_THROW_IF(fileName.empty(),
		EmptyFileName);

	std::ofstream out_stream{ fileName };


	//Interpolate Face Values to Node Values
	//(done by solving an overdetermined system of linear equations using least squares)
	//======================================
	SimTK::Matrix nodeMatrix;

	nodeMatrix.resize(mesh.getNumVertices(), table->getNumRows());
	nodeMatrix.setToZero();

	auto time = table->getIndependentColumn();
	auto matrix = table->getMatrix();

	int nNode = mesh.getNumVertices();
	int nTri = mesh.getNumFaces();
	int nTimeStep = table->getNumRows();



	//Loop over each timestep
	for (int i = 0; i < nTimeStep; ++i) {
		
		//Assign nonzero face values to right hand side
		SimTK::Vector activeTriNum;
		activeTriNum.resize(nTri); //size only nActiveTri
		int ind = 0;
		
		SimTK::Vector activeNode;
		SimTK::Vector activeTri;
		activeNode.resize(nNode);
		activeNode.setToZero();
		activeTri.resize(nTri);
		activeTri.setToZero();

		//Find active triangles and corresponding nodes
		
		for (int k = 0; k < nTri; ++k) {
			if (matrix(i, k) != 0) {
				activeTri(k) = 1;
				activeTriNum(ind) = k;
				ind++;

				for (int j = 0; j < mesh.getNumVerticesForFace(k); j++) {
					int iFV = mesh.getFaceVertex(k, j);
					activeNode(iFV) = 1;
				}
			}
		}
		
		int nActiveTri = activeTri.sum();
		int nActiveNode = activeNode.sum();
		SimTK::Vector activeNodeNum(nNode);
		SimTK::Vector nodeNumActive(nNode);
		SimTK::Vector b(nActiveTri);
		SimTK::Matrix A(nActiveTri, nActiveNode);
		SimTK::Vector x(nActiveNode);
		b.setToZero();
		A.setToZero();
		
		//Check that tri are active
		if (nActiveTri == 0) {
			continue;
		}

		//Renumber the active nodes (to keep track in A matrix)
		ind = 0;
		activeNodeNum = 0;
		
		for (int p = 0; p < nNode; ++p) {
			if (activeNode(p) == 1) {
				activeNodeNum(ind) = p;
				nodeNumActive(p) = ind;
				ind++;
			}
		}

		
		for (int j = 0; j < nActiveTri; ++j) {

			//Assign active face values to right hand side
			int tri_ind = activeTriNum(j);
			b(j) = matrix(i, tri_ind);
					
			//Build triangle-vertices connectivity matrix
			for (int k = 0; k < mesh.getNumVerticesForFace(tri_ind); k++) {
					int iFV = mesh.getFaceVertex(tri_ind, k);
					A(j, nodeNumActive(iFV)) = 1.0 / mesh.getNumVerticesForFace(tri_ind);
					//std::cout << A.ncol() << " " << A.nrow() << std::endl;
					//A(1, 0) = 1.0;
				
			}

		}
		//Solve least squares for node values
		auto LSmatrix = SimTK::FactorQTZ(A);
		LSmatrix.solve(b, x);
		
        //Ensure all interpolated values are greater than zero
        for (int j = 0; j < nActiveNode; ++j) {
            if (x(j) < 0.0){
                x(j) = 0.0;
            }
        }

		//Assign active node values to node matrix
		for (int j = 0; j < nActiveNode; ++j) {
			nodeMatrix(activeNodeNum(j),i) = x(j);
		}
		

        
        //Reset A matrix
		A.setToZero();
		
	    
	}

	// Write PostView Data file
	if (true) {
		constexpr auto prec = std::numeric_limits<double>::digits10 + 1;

		for (unsigned row = 0; row < nodeMatrix.nrow(); ++row) {

			out_stream << std::setprecision(1) << ver_start_ind + row + 1;

			for (unsigned col = 0; col < nodeMatrix.ncol(); ++col) {
				auto elt = nodeMatrix(row, col);
				out_stream << _delimiterWrite << std::setprecision(prec) << elt;
			}

			out_stream << "\n";
		}
	}

	// Write .prs file body
	if (false) {
		constexpr auto prec = std::numeric_limits<double>::digits10 + 1;

		for (unsigned row = 0; row < matrix.nrow(); ++row) {

			out_stream << std::setprecision(1);
			
			//Mesh1 data
            out_stream << time[row] << " 1 0 \n";

			//Mesh2 data 
			
            out_stream << time[row] << " 2 " << matrix.ncol();

            for (unsigned col = 0; col < matrix.ncol(); ++col) {
                auto elt = matrix(row, col);
                out_stream << " " << col << " " << std::setprecision(prec) << elt;
            }

            out_stream << "\n";
		}
	}
}

}
