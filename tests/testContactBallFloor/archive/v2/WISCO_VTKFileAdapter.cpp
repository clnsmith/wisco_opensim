#include "WISCO_VTKFileAdapter.h"
#include <fstream>

namespace OpenSim {

    WISCO_VTKFileAdapter::WISCO_VTKFileAdapter() 
    {

    }

    WISCO_VTKFileAdapter* WISCO_VTKFileAdapter::clone() const 
    {
        return new WISCO_VTKFileAdapter{ *this };
    }


    WISCO_VTKFileAdapter::OutputTables WISCO_VTKFileAdapter::extendRead(const std::string& fileName) const 
    {
        OutputTables output_tables{};
        return output_tables;
    };

    void WISCO_VTKFileAdapter::extendWrite(const InputTables& tables, const std::string& fileName) const
    {
    };


	//void WISCO_VTKFileAdapter::write(const std::string& fileName, const std::string& filePath,
	//	const SimTK::PolygonalMesh& mesh, const TimeSeriesTable& intable1) const {
	void WISCO_VTKFileAdapter::write(const std::string& fileName, const std::string& filePath, const SimTK::PolygonalMesh& mesh,
		std::vector<SimTK::Matrix>& vecOutputMatrix, std::vector<std::string>& OutputNames) const {

		std::cout << "Writing .vtk files to: " + filePath + fileName << std::endl;

		//Write File for each time step
		int nTimeSteps = vecOutputMatrix[0].nrow();

		for (int iFile = 0; iFile < nTimeSteps; ++iFile)
		{
			const std::string fullPath = filePath + fileName + "_" + std::to_string(iFile) + ".vtk";
			std::cout << fullPath << std::endl;
			//ERROR Checking
			OPENSIM_THROW_IF(fullPath.empty(), EmptyFileName);

			//Open File
			std::ofstream out_stream{ fullPath };

			//Write header
			out_stream << "# vtk DataFile Version 2.0" << std::endl;
			out_stream << fileName << std::endl;
			out_stream << "ASCII" << std::endl;
			out_stream << "DATASET POLYDATA" << std::endl;

			//Write Node Locations
			out_stream << "POINTS " << mesh.getNumVertices() << " float" << std::endl;

			write_vertex_positions(out_stream, mesh);

			//Write Face Connectivity
			int cellListSize = mesh.getNumFaces()*(3 + 1); // 3 for triangles
			out_stream << "POLYGONS" << " " << mesh.getNumFaces() << " " << cellListSize << std::endl;
			write_face_connectivity(out_stream, mesh);

			//Write Data
			out_stream << "CELL_DATA " << mesh.getNumFaces() << std::endl;
			int nDataField = vecOutputMatrix.size();
			
			for (int j = 0; j < nDataField; ++j)
			{
				
				out_stream << "SCALARS " << OutputNames[j] << " float" << std::endl;
				out_stream << "LOOKUP_TABLE default" << std::endl;

				for (int i = 0; i < mesh.getNumFaces(); ++i)
				{
					out_stream << vecOutputMatrix[j](iFile, i) << std::endl;
				}
			}
			out_stream.close();
		}
	}

	void WISCO_VTKFileAdapter::write_vertex_positions(std::ofstream& out_stream, const SimTK::PolygonalMesh& mesh) const
	{
		const std::string& delim = " ";

		for (int i = 0; i < mesh.getNumVertices(); ++i)
		{
			//Vertex Locations
			SimTK::Vec3 position = mesh.getVertexPosition(i);
			out_stream << position(0);
			out_stream << delim << position(1);
			out_stream << delim << position(2);
			out_stream << std::endl;
		}
		
    }

    void WISCO_VTKFileAdapter::write_face_connectivity(std::ofstream& out_stream, const SimTK::PolygonalMesh& mesh) const
    {
        for (int j = 0; j < mesh.getNumFaces(); ++j)
        {
            //Number of vertices
            out_stream << mesh.getNumVerticesForFace(j);

            //Member vertices
            int num_ver = mesh.getNumVerticesForFace(j);
                for (int k = 0; k < 3; ++k)
                {
                    out_stream << " " << mesh.getFaceVertex(j, k);
                }
                out_stream << std::endl;   
        }
    }
}