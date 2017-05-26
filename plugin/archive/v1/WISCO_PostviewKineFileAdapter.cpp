#include "WISCO_PostViewKineFileAdapter.h"
#include <fstream>

namespace OpenSim {

    WISCO_PostViewKineFileAdapter::WISCO_PostViewKineFileAdapter()
    {

    }

    WISCO_PostViewKineFileAdapter* WISCO_PostViewKineFileAdapter::clone() const
    {
        return new WISCO_PostViewKineFileAdapter{ *this };
    }


    WISCO_PostViewKineFileAdapter::OutputTables WISCO_PostViewKineFileAdapter::extendRead(const std::string& fileName) const
    {
        OutputTables output_tables{};
        return output_tables;
    };

    void WISCO_PostViewKineFileAdapter::extendWrite(const InputTables& tables, const std::string& fileName) const
    {
    };


    void WISCO_PostViewKineFileAdapter::write(const TimeSeriesTable& intable1, const TimeSeriesTable& intable2, const std::string& fileName)
    {
        const std::string _table{ "table" };
        
        InputTables absTables1{};
        absTables1.emplace(_table, &intable1);

        InputTables absTables2{};
        absTables2.emplace(_table, &intable2);

        //ERROR Checking
        OPENSIM_THROW_IF(absTables1.empty(), NoTableFound);
        OPENSIM_THROW_IF(absTables2.empty(), NoTableFound);

        const TimeSeriesTable* table1{};
        const TimeSeriesTable* table2{};

        try {
            auto abs_table1 = absTables1.at(_table);
            table1 = dynamic_cast<const TimeSeriesTable*>(abs_table1);

            auto abs_table2 = absTables2.at(_table);
            table2 = dynamic_cast<const TimeSeriesTable*>(abs_table2);
        }
        catch (std::out_of_range&) {
            OPENSIM_THROW(KeyMissing,
                _table);
        }
        catch (std::bad_cast&) {
            OPENSIM_THROW(IncorrectTableType);
        }

        OPENSIM_THROW_IF(fileName.empty(), EmptyFileName);

        //Open File
        std::ofstream out_stream{ fileName };

        //Write Transformations

        
        int nRow = table1->getNumRows();
        
        auto& data_matrix1 = intable1.getMatrix();
        auto& data_matrix2 = intable2.getMatrix();
        //intable.getDependentColumn()
        //auto col = table->getDependentColumn("ground_to_mesh2[0]");
        
        for (int i = 0; i < nRow; ++i) {
            for (int j = 0; j < data_matrix1.ncol(); ++j) {
               
                out_stream << data_matrix1(i, j) << ","; 
            }
            for (int j = 0; j < data_matrix1.ncol(); ++j) {

                out_stream << data_matrix1(i, j) << ",";
            }
            for (int j = 0; j < data_matrix2.ncol(); ++j) {

                out_stream << data_matrix2(i, j) << ",";
            }
            for (int j = 0; j < data_matrix2.ncol(); ++j) {

                out_stream << data_matrix2(i, j) << ",";
            }
            out_stream << std::endl;
        }
        
    }
}