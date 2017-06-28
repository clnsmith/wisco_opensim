#include "WISCO_H5FileAdapter.h"
#include "WISCO_HelperFunctions.h"
#include <fstream>

namespace OpenSim {

	WISCO_H5FileAdapter::WISCO_H5FileAdapter()
	{
		_time_is_empty = true;
	}

	WISCO_H5FileAdapter* WISCO_H5FileAdapter::clone() const
	{
		return new WISCO_H5FileAdapter{ *this };
	}

	void WISCO_H5FileAdapter::open(const std::string& file_name) 
    {
		_file = H5::H5File(file_name, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    }

	void WISCO_H5FileAdapter::close() {
		_file.close();
	}



	void WISCO_H5FileAdapter::createGroup(const std::string& group_name) {
		_file.createGroup(group_name);
	}
	
	void WISCO_H5FileAdapter::writeDataSet(const TimeSeriesTable& table, const std::string dataset_path) 
	{
		writeTimeDataSet(table);

		SimTK::Matrix data_matrix = table.getMatrix();

		hsize_t dim_data[1];
		dim_data[0] = data_matrix.nrow();
		
		for (int i = 0; i < data_matrix.ncol(); ++i) {
			H5::DataSpace dataspace(1, dim_data, dim_data);
			H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
			H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

			//Allocate space for data
			double* data = (double*)malloc(dim_data[0] * sizeof(double));

			//Set Data Array
			for (int r = 0; r < dim_data[0]; ++r) {
					data[r] = data_matrix(r,i);
			}
		
			dataset.write(&data[0], datatype);

			//Free dynamically allocated memory
			free(data);
		}
	}

	void WISCO_H5FileAdapter::writeDataSetVec3(const TimeSeriesTableVec3& table, const std::string dataset_path)
	{
		
		SimTK::Matrix_<SimTK::Vec3> data_matrix = table.getMatrix();

		hsize_t dim_data[2];
		dim_data[0] = data_matrix.nrow();
		dim_data[1] = 3;

		for (int i = 0; i < data_matrix.ncol(); ++i) {

			H5::DataSpace dataspace(2, dim_data, dim_data);
			H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
			H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

			//Allocate space for data
			double** data = (double**)malloc(dim_data[0] * sizeof(double*));
			data[0] = (double*)malloc(dim_data[1] * dim_data[0] * sizeof(double));
			for (int i = 1; i < dim_data[0]; i++) data[i] = data[0] + i*dim_data[1];

			//Set Data Array
			for (int r = 0; r < dim_data[0]; ++r) {
				for (int c = 0; c < 3; ++c) {
					data[r][c] = data_matrix(r, i)(c);
				}
			}

			dataset.write(&data[0][0], datatype);

			//Free dynamically allocated memory
			free(data[0]);
			free(data);
		}
	}

	void WISCO_H5FileAdapter::writeDataSetVector(const TimeSeriesTable& table, const std::string dataset_path)
	{
		writeTimeDataSet(table);
		
		SimTK::Matrix data_matrix = table.getMatrix();

		hsize_t dim_data[2];
		dim_data[0] = data_matrix.nrow();
		dim_data[1] = data_matrix.ncol();
		
		H5::DataSpace dataspace(2, dim_data, dim_data);
		H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
		H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

		//Allocate space for data
		double** data = (double**)malloc(dim_data[0] * sizeof(double*));
		data[0] = (double*)malloc(dim_data[1] * dim_data[0] * sizeof(double));
		for (int i = 1; i < dim_data[0]; i++) data[i] = data[0] + i*dim_data[1];

		//Set Data Array
		for (int r = 0; r < dim_data[0]; ++r) {
			for (int c = 0; c < dim_data[1]; ++c) {
				data[r][c] = data_matrix(r, c);
			}
		}

		dataset.write(&data[0][0], datatype);

		//Free dynamically allocated memory
		free(data[0]);
		free(data);
		
	}

	void WISCO_H5FileAdapter::writeStatesDataSet(const TimeSeriesTable& table) {
		
		writeTimeDataSet(table);

		std::string states_group_name = "/States";
		_file.createGroup(states_group_name);

		std::vector<std::string> labels = table.getColumnLabels();
	
		SimTK::Matrix data_matrix = table.getMatrix();

		hsize_t dim_data[1];
		dim_data[0] = data_matrix.nrow();

		for (int i = 0; i < data_matrix.ncol(); ++i) {
			SimTK::String label(labels[i]);

			label.replaceAllChar('/','_');
						
			std::string dataset_path = states_group_name + "/" + label;
			
			H5::DataSpace dataspace(1, dim_data, dim_data);
			H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
			H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

			//Allocate space for data
			double* data = (double*)malloc(dim_data[0] * sizeof(double));

			//Set Data Array
			for (int r = 0; r < dim_data[0]; ++r) {
				data[r] = data_matrix(r, i);
			}

			dataset.write(&data[0], datatype);

			//Free dynamically allocated memory
			free(data);
		}
	}

	void WISCO_H5FileAdapter::writeCoordinatesDataSet(const TimeSeriesTable& table) {

		std::string group_name = "/Coordinates";
		_file.createGroup(group_name);

		const SimTK::Matrix& data_matrix = table.getMatrix().getAsMatrix();

		std::vector<std::string> labels = table.getColumnLabels();
		std::string coord_name = split_string(labels[0], ".")[0];

		std::string coord_group = group_name + "/" + coord_name;
		_file.createGroup(coord_group);

		int i = 0;
		for (std::string label : labels) {
			std::vector<std::string> decomp_label = split_string(label, ".");
			std::string label_coord = decomp_label[0];
			std::string param = decomp_label[1];

			if (label_coord == coord_name) {
				SimTK::Vector data(data_matrix.nrow());
				for (int j = 0; j < data_matrix.nrow(); ++j) {
					data(j) = data_matrix(j, i);
				}

				writeDataSetSimTKVector(data, coord_group + "/" + param);
			}
			else {
				coord_name = label_coord;
				coord_group = group_name + "/" + coord_name;
				_file.createGroup(coord_group);

				SimTK::Vector data(data_matrix.nrow());
				for (int j = 0; j < data_matrix.nrow(); ++j) {
					data(j) = data_matrix(j, i);
				}

				writeDataSetSimTKVector(data, coord_group + "/" + param);
			}
			++i;
		}
	}

	void WISCO_H5FileAdapter::writeMuscleDataSet(const TimeSeriesTable& table) {
		writeTimeDataSet(table);

		std::string group_name = "/WISCO_IdealMuscle";
		_file.createGroup(group_name);

		const SimTK::Matrix& data_matrix = table.getMatrix().getAsMatrix();

		std::vector<std::string> labels = table.getColumnLabels();
		std::string msl_name = split_string(labels[0], ".")[0];

		std::string msl_group = group_name + "/" + msl_name;
		_file.createGroup(msl_group);

		int i = 0;
		for (std::string label : labels) {
			std::vector<std::string> decomp_label = split_string(label, ".");
			std::string label_msl = decomp_label[0];
			std::string param = decomp_label[1];

			if (label_msl == msl_name) {
				SimTK::Vector data(data_matrix.nrow());
				for (int j = 0; j < data_matrix.nrow(); ++j) {
					data(j) = data_matrix(j, i);
				}

				writeDataSetSimTKVector(data, msl_group + "/" + param);
			}
			else {
				msl_name = label_msl;
				msl_group = group_name + "/" + msl_name;
				_file.createGroup(msl_group);

				SimTK::Vector data(data_matrix.nrow());
				for (int j = 0; j < data_matrix.nrow(); ++j) {
					data(j) = data_matrix(j, i);
				}

				writeDataSetSimTKVector(data, msl_group + "/" + param);
			}
			++i;
		}
		
		
	}

	void WISCO_H5FileAdapter::writeLigamentDataSet(const TimeSeriesTable& table) {
		writeTimeDataSet(table);

		std::string group_name = "/WISCO_Ligament";
		_file.createGroup(group_name);

		const SimTK::Matrix& data_matrix = table.getMatrix().getAsMatrix();
		
		std::vector<std::string> labels = table.getColumnLabels();
		std::string lig_name = split_string(labels[0], ".")[0];
		
		std::string lig_group = group_name + "/" + lig_name;
		_file.createGroup(lig_group);

		int i = 0;
		for (std::string label : labels) {
			std::vector<std::string> decomp_label = split_string(label, ".");
			std::string label_lig = decomp_label[0];
			std::string param = decomp_label[1];

			if (label_lig == lig_name) {
				SimTK::Vector data(data_matrix.nrow());
				for (int j = 0; j < data_matrix.nrow(); ++j) {
					data(j) = data_matrix(j, i);
				}

				writeDataSetSimTKVector(data, lig_group + "/" + param);
			}
			else {
				lig_name = label_lig;
				lig_group = group_name + "/" + lig_name;
				_file.createGroup(lig_group);

				SimTK::Vector data(data_matrix.nrow());
				for (int j = 0; j < data_matrix.nrow(); ++j) {
					data(j) = data_matrix(j, i);
				}

				writeDataSetSimTKVector(data, lig_group + "/" + param);
			}
			++i;
		}
	}

	void WISCO_H5FileAdapter::writeDataSetSimTKVector(const SimTK::Vector& data_vector, const std::string dataset_path) {
		hsize_t dim_data[1];
		dim_data[0] = data_vector.size();

		H5::DataSpace dataspace(1, dim_data, dim_data);
		H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
		H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

		//Allocate space for data
		double* data = (double*)malloc(dim_data[0] * sizeof(double));

		//Set Data Array
		for (int r = 0; r < dim_data[0]; ++r) {
			data[r] = data_vector(r);
		}

		dataset.write(&data[0], datatype);

		//Free dynamically allocated memory
		free(data);
	}

	void WISCO_H5FileAdapter::writeDataSetSimTKVectorVec3(const SimTK::Vector_<SimTK::Vec3>& data_vector, const std::string dataset_path) {
		hsize_t dim_data[2];
		dim_data[0] = data_vector.nrow();
		dim_data[1] = 3;
		
		H5::DataSpace dataspace(2, dim_data, dim_data);
		H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
		H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

		//Allocate space for data
		double** data = (double**)malloc(dim_data[0] * sizeof(double*));
		data[0] = (double*)malloc(dim_data[1] * dim_data[0] * sizeof(double));
		for (int i = 1; i < dim_data[0]; i++) data[i] = data[0] + i*dim_data[1];

		//Set Data Array
		for (int r = 0; r < dim_data[0]; ++r) {
			for (int c = 0; c < 3; ++c) {
				data[r][c] = data_vector(r)(c);
			}
		}

		dataset.write(&data[0][0], datatype);

		//Free dynamically allocated memory
		free(data[0]);
		free(data);
	}

	void WISCO_H5FileAdapter::writeDataSetSimTKMatrixColumns(const SimTK::Matrix& data, std::vector<std::string> column_dataset_paths) {

		for (int i = 0; i < data.ncol(); ++i) {
			std::string path = column_dataset_paths[i];

			SimTK::Vector data_vec(data.nrow());
			for (int j = 0; j < data.nrow(); ++j) {
				data_vec(j) = data(j, i);
			}
			writeDataSetSimTKVector(data_vec, path);
		}	
	}

	void WISCO_H5FileAdapter::writeDataSetSimTKMatrixVec3Columns(const SimTK::Matrix_<SimTK::Vec3>& data, std::vector<std::string> column_dataset_paths) {

		for (int i = 0; i < data.ncol(); ++i) {
			std::string path = column_dataset_paths[i];

			SimTK::Vector_<SimTK::Vec3> data_vec(data.nrow());
			for (int j = 0; j < data.nrow(); ++j) {
				data_vec(j) = data(j, i);
			}
			writeDataSetSimTKVectorVec3(data_vec, path);
		}
	}

	void WISCO_H5FileAdapter::writeTimeDataSet(const TimeSeriesTable& table) {
		if (_time_is_empty) {
			std::vector<double> time = table.getIndependentColumn();

			hsize_t dim_time[1];
			dim_time[0] = time.size();

			H5::DataSpace time_dataspace(1, dim_time, dim_time);
			H5::PredType time_datatype(H5::PredType::NATIVE_DOUBLE);
			H5::DataSet dataset = _file.createDataSet("/time", time_datatype, time_dataspace);

			dataset.write(&time[0], time_datatype);

			_time_is_empty = false;
		}
	}
	
    WISCO_H5FileAdapter::OutputTables WISCO_H5FileAdapter::extendRead(const std::string& fileName) const 
    {
        OutputTables output_tables{};
        return output_tables;
    };

    void WISCO_H5FileAdapter::extendWrite(const InputTables& tables, const std::string& fileName) const
    {
    };
}