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

	void WISCO_H5FileAdapter::writeKinematicsDataSet(
		const TimeSeriesTable& pos_table, const TimeSeriesTable& vel_table, const TimeSeriesTable& acc_table) {

		std::string group_name = "/Coordinates";
		_file.createGroup(group_name);

		const SimTK::Matrix& pos_data = pos_table.getMatrix().getAsMatrix();
		const SimTK::Matrix& vel_data = vel_table.getMatrix().getAsMatrix();
		const SimTK::Matrix& acc_data = acc_table.getMatrix().getAsMatrix();

		std::vector<std::string> coords = pos_table.getColumnLabels();

		SimTK::Vector coord_pos(pos_data.nrow());
		SimTK::Vector coord_vel(vel_data.nrow());
		SimTK::Vector coord_acc(acc_data.nrow());

		for (int i = 0; i < pos_data.ncol(); ++i) {
			for (int j = 0; j < pos_data.nrow(); ++j) {
				coord_pos(j) = pos_data(j, i);
				coord_vel(j) = vel_data(j, i);
				coord_acc(j) = acc_data(j, i);
			}

			std::string coord_group = group_name + "/" + coords[i];
			_file.createGroup(coord_group);

			writeDataSetSimTKVector(coord_pos, coord_group + "/" + "position");
			writeDataSetSimTKVector(coord_vel, coord_group + "/" + "velocity");
			writeDataSetSimTKVector(coord_acc, coord_group + "/" + "acceleration");
		}
	}

	void WISCO_H5FileAdapter::writeMuscleDataSet(
		std::vector<SimTK::Matrix>& msl_vec, std::vector<std::string> msl_names, 
		std::vector<std::string> param_names, const TimeSeriesTable& states_table) {
		writeTimeDataSet(states_table);

		std::string group_name = "/Muscles";
		_file.createGroup(group_name);

		int nParams = param_names.size();
		int nFrames = msl_vec[0].nrow();
		int nMsl = msl_vec[0].ncol();

		for (int i = 0; i < nMsl; ++i) {
			std::string msl_group = group_name + "/" + msl_names[i];
			_file.createGroup(msl_group);
			
			for (int j = 0; j < nParams; ++j) {
				SimTK::Vector data(nFrames);
				for (int k = 0; k < nFrames; ++k) {
					data(k) = msl_vec[j](k,i);
				}
				std::string msl_param_path = msl_group + "/" + param_names[j];
				writeDataSetSimTKVector(data, msl_param_path);

			}

		}

		std::vector<std::string> states_labels = states_table.getColumnLabels();
		int nStates = states_labels.size();

		const SimTK::Matrix& states_matrix = states_table.getMatrix().getAsMatrix();

		for (int s = 0; s < nStates; ++s) {
			std::vector<std::string> decomp_label = split_string(states_labels[s], "/");
			int index;
			if (contains_string(msl_names, decomp_label[0], index)) {
				std::string data_path = group_name + "/" + decomp_label[0] + "/" + decomp_label[1];
				SimTK::Vector ms_data(nFrames);
				for (int k = 0; k < nFrames; ++k) {
					ms_data(k) = states_matrix(s, k);
				}
				writeDataSetSimTKVector(ms_data, data_path);

			}
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