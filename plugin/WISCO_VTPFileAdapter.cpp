#include "WISCO_VTPFileAdapter.h"
#include <fstream>
#include "base64.h"
#include <bitset>

using namespace SimTK;
namespace OpenSim {

	WISCO_VTPFileAdapter::WISCO_VTPFileAdapter()
	{

	}

	WISCO_VTPFileAdapter* WISCO_VTPFileAdapter::clone() const
	{
		return new WISCO_VTPFileAdapter{ *this };
	}


	WISCO_VTPFileAdapter::OutputTables WISCO_VTPFileAdapter::extendRead(const std::string& fileName) const
	{
		OutputTables output_tables{};
		return output_tables;
	};

	void WISCO_VTPFileAdapter::extendWrite(const InputTables& tables, const std::string& fileName) const
	{
	};

	//Write Using direct input of points and vertices
	void WISCO_VTPFileAdapter::write(const std::string& fileName, const std::string& filePath,
		const SimTK::Matrix_<SimTK::Vec3>& vertices, const SimTK::Matrix& faces,
		const int nTimeSteps) const {

		std::cout << "Writing .vtp files to: " + filePath + fileName << std::endl;

		int nPoints = vertices.nrow();
		int nStrips = 0;
		int nVerts = 0;
		int nLines = 0;
		int nPolys = faces.nrow();

		for (int iFrame = 0; iFrame < nTimeSteps; ++iFrame)
		{
			Xml::Document doc;

			doc.setRootTag("VTKFile");

			Xml::Element root = doc.getRootElement();
			root.setAttributeValue("type", "PolyData");
			root.setAttributeValue("version", "1.0");
			root.setAttributeValue("header_type", "UInt32");
			if (isLittleEndian()) root.setAttributeValue("byte_order", "LittleEndian");
			else root.setAttributeValue("byte_order", "BigEndian");
			SimTK::Xml::Element PolyData("PolyData");
			root.appendNode(PolyData);

			Xml::Element Piece("Piece");
			Piece.setAttributeValue("NumberOfPoints", std::to_string(nPoints));
			Piece.setAttributeValue("NumberOfVerts", std::to_string(nVerts));
			Piece.setAttributeValue("NumberOfLines", std::to_string(nLines));
			Piece.setAttributeValue("NumberOfStrips", std::to_string(nStrips));
			Piece.setAttributeValue("NumberOfPolys", std::to_string(nPolys));

			PolyData.appendNode(Piece);

			//Point Data
			Xml::Element PointData("PointData");
			std::string pnt_names;
			for (int i = 0; i < _pointDataNames.size(); ++i) {
				pnt_names.append(" " + _pointDataNames[i]);
			}
			PointData.setAttributeValue("Scalars", pnt_names);

			for (int i = 0; i < _pointDataNames.size(); ++i) {
				Xml::Element DataArray("DataArray");
				DataArray.setAttributeValue("type","Float32");
				DataArray.setAttributeValue("Name", _pointDataNames[i]);

				if (_data_format == "ascii") {
					DataArray.setAttributeValue("format", "ascii");

					std::string pdata;
					for (int j = 0; j < _pointData[i].ncol(); ++j) {
						pdata.append(std::to_string(_pointData[i](iFrame, j)) + " ");
					}
					DataArray.setValue(pdata);
				}
				
				else if (_data_format == "binary") {
					DataArray.setAttributeValue("format", "binary");

					std::vector<float> data;
					for (int j = 0; j < _pointData[i].ncol(); ++j) {
						data.push_back(_pointData[i](iFrame, j));
					}
					std::string encoded_data = encodeFloatDataVTPBase64(data);
					DataArray.setValue(encoded_data);

				}
				PointData.appendNode(DataArray);
			}
			Piece.appendNode(PointData);


			//Cell Data
			Xml::Element CellData("CellData");
			std::string cell_names;
			for (int i = 0; i < _faceDataNames.size(); ++i) {
				cell_names.append(" " + _faceDataNames[i]);
			}
			CellData.setAttributeValue("Scalars", cell_names);

			for (int i = 0; i < _faceDataNames.size(); ++i) {
				Xml::Element DataArray("DataArray");
				DataArray.setAttributeValue("type", "Float32");
				DataArray.setAttributeValue("Name", _faceDataNames[i]);

				std::string cdata;
				if (_data_format == "ascii") {
					DataArray.setAttributeValue("format", "ascii");
					for (int j = 0; j < _faceData[i].ncol(); ++j) {
						cdata.append(std::to_string(_faceData[i](iFrame, j)) + " ");
						
					}
					DataArray.setValue(cdata);
				}

				else if (_data_format == "binary") {
					DataArray.setAttributeValue("format", "binary");

					std::vector<float> data;
					for (int j = 0; j < _faceData[i].ncol(); ++j) {
						data.push_back(_faceData[i](iFrame, j));
					}

					std::string encode_data = encodeFloatDataVTPBase64(data);
					DataArray.setValue(encode_data);
				}
				CellData.appendNode(DataArray);
			}
			Piece.appendNode(CellData);

			//Points
			Xml::Element Points("Points");
			Xml::Element PointArray("DataArray");
			PointArray.setAttributeValue("Name", "Points");
			PointArray.setAttributeValue("type", "Float32");
			PointArray.setAttributeValue("NumberOfComponents", "3");

			
			if (_data_format == "ascii") {
				PointArray.setAttributeValue("format", "ascii");
				std::string pnt_coords;
				for (int i = 0; i < vertices.nrow(); ++i) {
					pnt_coords.append(std::to_string(vertices(i,iFrame)(0)) + " ");
					pnt_coords.append(std::to_string(vertices(i,iFrame)(1)) + " ");
					pnt_coords.append(std::to_string(vertices(i,iFrame)(2)) + " ");
				}
				PointArray.setValue(pnt_coords);
			}
			else if (_data_format == "binary") {
				PointArray.setAttributeValue("format", "binary");
				std::vector<float> pnt_coords;
				for (int i = 0; i < vertices.nrow(); ++i) {
					pnt_coords.push_back(vertices(i, iFrame)(0));
					pnt_coords.push_back(vertices(i, iFrame)(1));
					pnt_coords.push_back(vertices(i, iFrame)(2));
				}
				std::string encode_data = encodeFloatDataVTPBase64(pnt_coords);
				PointArray.setValue(encode_data);
			}
			Points.appendNode(PointArray);
			Piece.appendNode(Points);

			//Verts
			Xml::Element Verts("Verts");
			Xml::Element VertsData1("DataArray");
			Xml::Element VertsData2("DataArray");
			VertsData1.setAttributeValue("type", "Int64");
			VertsData1.setAttributeValue("Name", "connectivity");
			VertsData2.setAttributeValue("type", "Int64");
			VertsData2.setAttributeValue("Name", "offsets");
			if (_data_format == "binary") {
				VertsData1.setAttributeValue("format", "binary");
				VertsData2.setAttributeValue("format", "binary");
			}
			else if (_data_format == "ascii") {
				VertsData1.setAttributeValue("format", "ascii");
				VertsData2.setAttributeValue("format", "ascii");
			}
			Verts.appendNode(VertsData1);
			Verts.appendNode(VertsData2);
			Piece.appendNode(Verts);

			//Lines
			Xml::Element Lines("Lines");
			Xml::Element LinesData1("DataArray");
			Xml::Element LinesData2("DataArray");
			LinesData1.setAttributeValue("type", "Int64");
			LinesData1.setAttributeValue("Name", "connectivity");
			LinesData2.setAttributeValue("type", "Int64");
			LinesData2.setAttributeValue("Name", "offsets");
			if (_data_format == "binary") {
				LinesData1.setAttributeValue("format", "binary");
				LinesData2.setAttributeValue("format", "binary");
			}
			else if (_data_format == "ascii") {
				LinesData1.setAttributeValue("format", "ascii");
				LinesData2.setAttributeValue("format", "ascii");
			}
			Lines.appendNode(LinesData1);
			Lines.appendNode(LinesData2);
			Piece.appendNode(Lines);

			//Strips
			Xml::Element Strips("Strips");
			Xml::Element StripsData1("DataArray");
			Xml::Element StripsData2("DataArray");
			StripsData1.setAttributeValue("type", "Int64");
			StripsData1.setAttributeValue("Name", "connectivity");
			StripsData2.setAttributeValue("type", "Int64");
			StripsData2.setAttributeValue("Name", "offsets");
			if (_data_format == "binary") {
				StripsData1.setAttributeValue("format", "binary");
				StripsData2.setAttributeValue("format", "binary");
			}
			else if (_data_format == "ascii") {
				StripsData1.setAttributeValue("format", "ascii");
				StripsData2.setAttributeValue("format", "ascii");
			}
			Strips.appendNode(StripsData1);
			Strips.appendNode(StripsData2);
			Piece.appendNode(Strips);

			//Polygons
			Xml::Element Polys("Polys");
			Xml::Element PolysData1("DataArray");
			Xml::Element PolysData2("DataArray");
			PolysData1.setAttributeValue("type", "UInt32");
			PolysData1.setAttributeValue("Name", "connectivity");
			PolysData2.setAttributeValue("type", "UInt32");
			PolysData2.setAttributeValue("Name", "offsets");

			if (_data_format == "ascii") {
				PolysData1.setAttributeValue("format", "ascii");
				PolysData2.setAttributeValue("format", "ascii");

				std::string poly_data;
				std::string poly_offset;
				int poly_off=0;
				for (int i = 0; i < faces.nrow(); ++i) {
					for (int j = 0; j < 3; ++j) {
						poly_data.append(std::to_string(int(faces(i, j))) + " ");
						poly_off++;
					}
					poly_offset.append(std::to_string(poly_off)+" ");
				}
				PolysData1.setValue(poly_data);
				PolysData2.setValue(poly_offset);
			}

			else if (_data_format == "binary") {
				PolysData1.setAttributeValue("format", "binary");
				PolysData2.setAttributeValue("format", "binary");

				std::vector<uint32_t> poly_data;
				std::vector<uint32_t> poly_offset;
				int poly_off = 0;
				for (int i = 0; i < faces.nrow(); ++i) {
					for (int j = 0; j < 3; ++j) {
						poly_data.push_back(uint32_t(faces(i, j)));
						poly_off++;
					}
					poly_offset.push_back(poly_off);
				}
				std::string encode_poly_data = encodeIntDataVTPBase64(poly_data);
				std::string encode_poly_offset = encodeIntDataVTPBase64(poly_offset);

				PolysData1.setValue(encode_poly_data);
				PolysData2.setValue(encode_poly_offset);

			}

			Polys.appendNode(PolysData1);
			Polys.appendNode(PolysData2);

			Piece.appendNode(Polys);

			//Write File
			doc.writeToFile(filePath + fileName + "_" + std::to_string(iFrame) + ".vtp");
		}

	}

	//Write Helper Using SimTK::PolygonalMesh
	void WISCO_VTPFileAdapter::write(const std::string& fileName,
		const std::string& filePath, const SimTK::PolygonalMesh& mesh,
		const int nTimeSteps) const {

		//Find Vertex locations
		SimTK::Matrix_<SimTK::Vec3> vertices(mesh.getNumVertices(), nTimeSteps);

		for (int j = 0; j < nTimeSteps; ++j) {
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
				faces(i, j) = mesh.getFaceVertex(i, j);
			}
		}

		//Call write function
		write(fileName, filePath, vertices, faces, nTimeSteps);

	}

	bool WISCO_VTPFileAdapter::isLittleEndian() const 
	{
		int num = 1;

		if (*(char *)&num == 1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	std::string WISCO_VTPFileAdapter::encodeFloatDataVTPBase64(std::vector<float>& data) const {

		int nData = data.size();

		const float *p_floats = &(data[0]);
		const unsigned char* p_bytes = reinterpret_cast<const unsigned char *>(p_floats);

		uint32_t len = nData * sizeof(float);


		std::string cdata = base64_encode(p_bytes, nData * sizeof(float));
		std::string encoded_data = base64_encode(reinterpret_cast<unsigned char *>(&len), sizeof(uint32_t));
		encoded_data.append(cdata);
		return encoded_data;
	}

	std::string WISCO_VTPFileAdapter::encodeIntDataVTPBase64(std::vector<uint32_t>& data) const {

		int nData = data.size();

		const uint32_t *p_int = &(data[0]);
		const unsigned char* p_bytes = reinterpret_cast<const unsigned char *>(p_int);
		uint32_t len = nData * sizeof(uint32_t);

		std::string cdata = base64_encode(p_bytes, nData * sizeof(uint32_t));
		std::string encoded_data = base64_encode(reinterpret_cast<unsigned char *>(&len), sizeof(uint32_t));
		encoded_data.append(cdata);

		return encoded_data;
	}
}


