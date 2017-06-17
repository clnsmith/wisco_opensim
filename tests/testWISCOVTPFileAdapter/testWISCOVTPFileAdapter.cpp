#include <OpenSim/OpenSim.h>
#include <vector>
#include "testWISCOVTPFileAdapter.h"
#include "WISCO_VTPFileAdapter.h"

//using namespace SimTK;
using namespace OpenSim;
using namespace SimTK;

int main()
{
    try {

		const std::string file_name{ "test" };
		const std::string file_path{ "C:/github/wisco_opensim/build/sandbox/wipVTPFileAdapter/results/" };
		const std::string mesh_file{ "C:/github/wisco_opensim/build/sandbox/wipVTPFileAdapter/results/test_paraview.vtp" };

		SimTK::PolygonalMesh mesh;
		mesh.loadFile(mesh_file);

		SimTK::Matrix cell_data(1,20,4.0);
		std::vector<SimTK::Matrix> cellData;
		cellData.push_back(cell_data);

		std::vector<std::string> cellDataNames = { "CellDataSet1" };
		

		SimTK::Matrix point_data(1, 15, 5.5);
		std::vector<SimTK::Matrix> pointData;
		pointData.push_back(point_data);

		std::vector<std::string> pointDataNames = { "PointDataSet1" };

		WISCO_VTPFileAdapter vtp = WISCO_VTPFileAdapter();

		vtp.setDataFormat("binary");
		vtp.setFaceData(cellDataNames, cellData);
		vtp.setPointData(pointDataNames, pointData);
		vtp.write(file_name, file_path, mesh, 1);

	// **********  END CODE  **********
	}
	catch (OpenSim::Exception ex)
	{
		std::cout << ex.getMessage() << std::endl;
		std::cin.get();
		return 1;
	}
	catch (SimTK::Exception::Base ex)
	{
		std::cout << ex.getMessage() << std::endl;
		std::cin.get();
		return 1;
	}
	catch (std::exception ex)
	{
		std::cout << ex.what() << std::endl;
		std::cin.get();
		return 1;
	}
	catch (...)
	{
		std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
		std::cin.get();
		return 1;
	}
	std::cout << "OpenSim example completed successfully" << std::endl;
	std::cout << "Press return to continue" << std::endl;
	std::cin.get();
	return 0;
}

