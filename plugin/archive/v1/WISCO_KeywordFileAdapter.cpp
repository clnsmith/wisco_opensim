#include "WISCO_KeywordFileAdapter.h"
#include <fstream>

namespace OpenSim {

    WISCO_KeywordFileAdapter::WISCO_KeywordFileAdapter() 
    {

    }

    WISCO_KeywordFileAdapter* WISCO_KeywordFileAdapter::clone() const 
    {
        return new WISCO_KeywordFileAdapter{ *this };
    }


    WISCO_KeywordFileAdapter::OutputTables WISCO_KeywordFileAdapter::extendRead(const std::string& fileName) const 
    {
        OutputTables output_tables{};
        return output_tables;
    };

    void WISCO_KeywordFileAdapter::extendWrite(const InputTables& tables, const std::string& fileName) const
    {
    };


    void WISCO_KeywordFileAdapter::write(const std::string& fileName, 
        const SimTK::PolygonalMesh& mesh1b, const SimTK::PolygonalMesh& mesh1c,
        const SimTK::PolygonalMesh& mesh2b, const SimTK::PolygonalMesh& mesh2c
        ) const {

        const std::string& delim = " ";

        //ERROR Checking
        OPENSIM_THROW_IF(fileName.empty(), EmptyFileName);

        //Open File
        std::ofstream out_stream{ fileName };

        //Write Node Locations
        out_stream << "*KEYWORD" << std::endl;
        out_stream << "*NODE" << std::endl;

        write_vertex_positions(out_stream, mesh1b,0);
        write_vertex_positions(out_stream, mesh1c,mesh1b.getNumVertices());
        
        int nVer = mesh1b.getNumVertices() + mesh1c.getNumVertices();
        write_vertex_positions(out_stream, mesh2b, nVer);
        
        nVer = mesh1b.getNumVertices() + mesh1c.getNumVertices()+mesh2b.getNumVertices();
        write_vertex_positions(out_stream, mesh2c, nVer);
       
        //Write Face Connectivity
        out_stream << "*ELEMENT_SHELL" << std::endl;

        write_face_connectivity(out_stream,mesh1b,1,0,0);
        
        write_face_connectivity(out_stream, mesh1c, 2, mesh1b.getNumVertices(), mesh1b.getNumFaces());
        
        nVer = mesh1b.getNumVertices()+ mesh1c.getNumVertices();
        int nTri = mesh1b.getNumFaces() + mesh1c.getNumFaces();
        write_face_connectivity(out_stream, mesh2b, 3, nVer, nTri);
        
        nVer = mesh1b.getNumVertices() + mesh1c.getNumVertices()+ mesh2b.getNumVertices();
        nTri = mesh1b.getNumFaces() + mesh1c.getNumFaces() + mesh2b.getNumFaces();
        write_face_connectivity(out_stream, mesh2c, 4, nVer, nTri);
        

        //End File
        out_stream << "*END" << std::endl;
    }

    void WISCO_KeywordFileAdapter::write_vertex_positions(std::ofstream& out_stream, const SimTK::PolygonalMesh mesh, const int ver_start_ind) const 
    {
        const std::string& delim = " ";

        int vert_ind = ver_start_ind+1;
        for (int i = 0; i < mesh.getNumVertices(); ++i)
        {
            //Node Number
            out_stream << delim << vert_ind;

            //Vertex Locations
            SimTK::Vec3 position = mesh.getVertexPosition(i);
            out_stream << delim << position(0);
            out_stream << delim << position(1);
            out_stream << delim << position(2);
            out_stream << std::endl;

            vert_ind++;
        }
    }

    void WISCO_KeywordFileAdapter::write_face_connectivity(std::ofstream& out_stream, const SimTK::PolygonalMesh mesh, 
        const int part_ind, const int ver_start_ind, const int tri_start_ind) const
    {
        const std::string& delim = " ";

        int tri_ind = tri_start_ind + 1;

        for (int j = 0; j < mesh.getNumFaces(); ++j)
        {
            //Triangle Number
            out_stream << delim << tri_ind;

            //Part Identifier
            out_stream << delim << part_ind;

            //Member vertices
            int num_ver = mesh.getNumVerticesForFace(j);

            if (num_ver == 3)
            {
                for (int k = 0; k < 3; ++k)
                {
                    out_stream << delim << mesh.getFaceVertex(j, k) + ver_start_ind + 1;
                }

                out_stream << delim << mesh.getFaceVertex(j, 2) + ver_start_ind + 1; //repeat last vertex
                out_stream << std::endl;
                
            }
            else {
                for (int k = 0; k < 4; ++k)
                {
                    out_stream << delim << mesh.getFaceVertex(j, k) + ver_start_ind + 1;
                }
                out_stream << std::endl;
            }

            tri_ind++;
        }
    }
}