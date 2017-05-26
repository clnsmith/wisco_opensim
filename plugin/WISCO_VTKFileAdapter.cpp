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

	//Write Using direct input of points and vertices
	void WISCO_VTKFileAdapter::write(const std::string& fileName, const std::string& filePath,
		const SimTK::Matrix_<SimTK::Vec3>& vertices, const SimTK::Matrix& faces,
		const int nTimeSteps) const {

		std::cout << "Writing .vtk files to: " + filePath + fileName << std::endl;

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
			int nVertices = vertices.nrow();
			out_stream << "POINTS " << nVertices << " float" << std::endl;

			

			write_vertex_positions(out_stream, vertices, iFile);

			//Write Face Connectivity
			int nFaces = faces.nrow();
			int nFacePerVert = faces.ncol();
			int cellListSize = nFaces*(nFacePerVert + 1); // 3 for triangles
			out_stream << "POLYGONS" << " " << nFaces << " " << cellListSize << std::endl;
			write_face_connectivity(out_stream, faces);

			//Write Face Data
			if (faceData.size() > 0) {
				out_stream << "CELL_DATA " << nFaces << std::endl;
				int nDataField = faceData.size();

				for (int j = 0; j < nDataField; ++j)
				{

					out_stream << "SCALARS " << faceDataNames[j] << " float" << std::endl;
					out_stream << "LOOKUP_TABLE default" << std::endl;

					for (int i = 0; i < nFaces; ++i)
					{
						out_stream << faceData[j](iFile, i) << std::endl;
					}
				}
			}

			//Write Point Data
			if (pointData.size() > 0) {
				out_stream << "POINT_DATA " << nVertices << std::endl;
				int nDataField = pointData.size();

				for (int j = 0; j < nDataField; ++j)
				{

					out_stream << "SCALARS " << pointDataNames[j] << " float" << std::endl;
					out_stream << "LOOKUP_TABLE default" << std::endl;

					for (int i = 0; i < nVertices; ++i)
					{
						out_stream << pointData[j](iFile, i) << std::endl;
					}
				}
			}

			out_stream.close();
		}
	}

	//Write Helper Using SimTK::PolygonalMesh
	void WISCO_VTKFileAdapter::write(const std::string& fileName, 
		const std::string& filePath, const SimTK::PolygonalMesh& mesh,
		const int nTimeSteps) const {

		//Find Vertex locations
		SimTK::Matrix_<SimTK::Vec3> vertices(mesh.getNumVertices(),nTimeSteps);

		for (int j = 0; j  <nTimeSteps; ++j){
			for (int i = 0; i < mesh.getNumVertices(); ++i)
			{
				//Vertex Locations
				SimTK::Vec3 position = mesh.getVertexPosition(i);
				vertices(i, j)(0) = position(0);
				vertices(i, j)(1) = position(1);
				vertices(i, j)(2) = position(2);
			}
		}

		//Construct Face connectivity
		SimTK::Matrix faces(mesh.getNumFaces(), mesh.getNumVerticesForFace(0));

		for (int i = 0; i < mesh.getNumFaces(); ++i) {
			for (int j = 0; j < mesh.getNumVerticesForFace(i); ++j)
			{
				 faces(i,j) = mesh.getFaceVertex(i, j);
			}
		}

		//Call write function
		write(fileName, filePath, vertices, faces, nTimeSteps);
		
	}

	void WISCO_VTKFileAdapter::write_vertex_positions(std::ofstream& out_stream, const SimTK::Matrix_<SimTK::Vec3>& vertices, int step) const
	{
		const std::string& delim = " ";

		for (int i = 0; i < vertices.nrow(); ++i)
		{
			//Vertex Locations
			out_stream << vertices(i,step)(0);
			out_stream << delim << vertices(i, step)(1);
			out_stream << delim << vertices(i, step)(2);
			out_stream << std::endl;
		}

	}

	void WISCO_VTKFileAdapter::write_face_connectivity(std::ofstream& out_stream, const SimTK::Matrix& faces) const
	{
		
		int nVertPerFace = faces.ncol();

		for (int i = 0; i < faces.nrow(); ++i)
		{
			//Number of vertices
			out_stream << nVertPerFace;

			//Member vertices
			for (int j = 0; j < nVertPerFace; ++j)
			{
				out_stream << " " << faces(i,j);
			}
			out_stream << std::endl;
		}
	}
}