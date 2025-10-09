#ifndef HEIGHTMAP_PARSER_H
#define HEIGHTMAP_PARSER_H

#include <vicar_data.h>
#include <mod_data.h>
#include <composite_data.h>
#include <translated_data.h>
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "perseverance_utils.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../thirdparty/stb_image_write.h"

using namespace chrono;

#if INCL_VSG==1
using namespace chrono::vsg3d;
#endif

using namespace chrono::vehicle;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

class HeightmapParser {


public:

    struct SoilParameters {
        double spacing = 0.08;
        double density = 1200;
        double cohesion = 4e3;
        double friction = 0.8;
        double youngs_modulus = 1e6;
        double poisson_ratio = 0.3;
    };

    static std::shared_ptr<rsvp::ImageData> ParseHeightmap(
                const std::string& filename,
                bool verbose=true) {
        std::shared_ptr<rsvp::ImageData> res =
            rsvp::ModData::read_bare_vicarfile(filename);
        return res;
    }

    static std::shared_ptr<rsvp::ImageData> ParseModFile(
            const std::string& filename
    ) 
    {
        std::shared_ptr<rsvp::ImageData> result = rsvp::ModData::read_modfile(filename);
        return result;
    }
    
    static void 
        as16BitPNG(std::string filename, const std::shared_ptr<rsvp::ImageData>& heightmap, float spacing, double width, double height, double x_offset, double y_offset) {

        int pixel_width = width/spacing;
        int pixel_height = height/spacing;
        
        float heights[pixel_width][pixel_height];
        uint8_t image[pixel_width][pixel_height];
       
        float max = -1000;
        float min = 1000;

        for (int j = -pixel_height*0.5f; j < pixel_height*0.5f; ++j) {
            for (int i = -pixel_width*0.5f; i < pixel_width*0.5f; ++i) {
                float x = i * spacing + x_offset;
                float y = j * spacing + y_offset;
                double z = 0;
                heightmap->get_interpolated_pixel_double(z, x, y, 1);
                heights[i][j] = z;
            }
        }
        for(int i = 0; i <  pixel_width; i++) {
            for(int j = 0; j < pixel_height; j++) {
                max = std::fmax(heights[i][j],max);
                min = std::fmin(heights[i][j],min);
            }
        }

        for(int i = 0; i < pixel_width; i++) {
            for(int j = 0; j < pixel_height; j++) {
                uint8_t h = (heights[i][j]-min)/(max-min);
                image[i][j] = h;
            }
        }

        stbi_write_png(filename.c_str(), pixel_width, pixel_height, 1, image, pixel_width);
    }



    static std::shared_ptr<ChTriangleMeshConnected> 
        asChronoMesh(const std::shared_ptr<rsvp::ImageData>& heightmap, float spacing, double width, double height, double x_offset, double y_offset) {

        auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        
        int pixel_width = width/spacing;
        int pixel_height = height/spacing;
        
        std::vector<ChVector3d> vertices;
        for (int j = -pixel_height*0.5f; j < pixel_height*0.5f; ++j) {
            for (int i = -pixel_width*0.5f; i < pixel_width*0.5f; ++i) {
                float x = i * spacing + x_offset;
                float y = j * spacing + y_offset;
                double z = 0;
                heightmap->get_interpolated_pixel_double(z, x, y, 1);
                vertices.push_back({x,y,z});
            }
        }

        for (int j = 0; j <  pixel_height - 1; ++j) {
            for (int i = 0; i < pixel_width - 1; ++i) {
                int v0 = j * pixel_width + i;
                int v1 = j * pixel_height + (i + 1);
                int v2 = (j + 1) * pixel_width + (i + 1);
                int v3 = (j + 1) * pixel_width + i;

                mesh->AddTriangle(ChTriangle(vertices[v0], vertices[v1], vertices[v2]));
                mesh->AddTriangle(ChTriangle(vertices[v0], vertices[v2], vertices[v3]));
            }
        }
        return mesh;
    }


    /*
        Construct SPH box
    */
    static void 
    Construct(CRMTerrain& terr, const double& m_spacing, const ChVector3d& box_size, const ChVector3d& pos, int side_flags) {

        // Number of points in each direction
        int Nx = std::round(box_size.x() / m_spacing) + 1;
        int Ny = std::round(box_size.y() / m_spacing) + 1;
        int Nz = std::round(box_size.z() / m_spacing) + 1;

        // Reserve space for containers
        int num_sph = Nx * Ny * Nz;

        std::vector<ChVector3i> sph;
        sph.reserve(num_sph);

        // Generate SPH points
        for (int Ix = 0; Ix < Nx; Ix++) {
            for (int Iy = 0; Iy < Ny; Iy++) {
                for (int Iz = 0; Iz < Nz; Iz++) {
                    sph.push_back(ChVector3i(Ix, Iy, Iz));  // SPH particles above 0
                }
            }
        }

        ChFsiProblemSPH::GridPoints m_sph;

        // // Insert in cached sets
        for (auto& p : sph) {
            m_sph.insert(p);
        }

        // if (m_verbose) {
        //     cout << "  Particle grid size:      " << Nx << " " << Ny << " " << Nz << endl;
        //     cout << "  Num. SPH particles:      " << m_sph.size() << " (" << sph.size() << ")" << endl;
        // }


        ChVector3d m_offset_sph = pos - ChVector3d(box_size.x() / 2, box_size.y() / 2, 0);

        terr.SetSPHPoints(m_sph, m_offset_sph);

        if (side_flags != BoxSide::NONE)
            terr.AddBoxContainer(box_size, pos, side_flags);

    }

    /*
        Construct CRMTerrain using RSVP image-data height ifc
    */
    static void 
    Construct(CRMTerrain& terr, std::shared_ptr<rsvp::ImageData> image, const double& m_spacing, const ChVector2d& box_size, const ChVector3f& rover_pos_o, int side_flags) {

        // Find heights to offset against
        double z_min = 1000;
        double z_max = -1000;

        double y_off = 0.0;
        // double z_off = 0.11;
        double z_off = 0.0;

        ChVector3f rover_pos = ChVector3f(rover_pos_o.x(),rover_pos_o.y() + y_off, rover_pos_o.z());

        for(float x = -box_size.x()*0.5f; x < box_size.x()*0.5f; x+=m_spacing) {
            for(float y = -box_size.y()*0.5f; y < box_size.y()*0.5f; y+=m_spacing) {
                double z = -1;
                image->get_interpolated_pixel_double(z, x + rover_pos.x(),y + rover_pos.y(), 1);
                // z = -z; // flip to site frame
                z_min = std::min(z,z_min);
                z_max = std::max(z,z_max);
            }
        }

        double height =  (z_max - z_min);

        ChVector3d box = { box_size.x(), box_size.y(), height };

        std::cout << "Minimum height: " << z_min << std::endl;
        std::cout << "Maximum height: " << z_max << std::endl;
        std::cout << "Height Range: " << height << std::endl;

        // Number of points in each direction
        int Nx = std::round(box.x() / m_spacing) + 1;
        int Ny = std::round(box.y() / m_spacing) + 1;
        int Nz = std::round(box.z() / m_spacing) + 1;

        std::vector<ChVector3i> sph;
        std::vector<ChVector3i> bce;
        // Generate SPH points
        for (int Ix = 0; Ix < Nx; Ix++) {
            for (int Iy = 0; Iy < Ny; Iy++) {
                float x = (Ix*m_spacing + rover_pos.x()) - (box.x()*0.5);
                float y = (Iy*m_spacing + rover_pos.y()) - (box.y()*0.5);

                double z = -1;
                image->get_interpolated_pixel_double(z, x, y, 1);
                // z = -z;
                
                int zLim = std::round((z - z_min)/m_spacing);
                double terr_depth = 0.3 / m_spacing; // meters / spacing
                double bce_thickness = 0.1 / m_spacing; // meters / spacing
                
                for (int Iz = zLim; Iz < zLim + terr_depth; Iz++) {
                    sph.push_back(ChVector3i(Ix, Iy, Iz));  // SPH particles above 0
                }
                
                for (int Iz = zLim + terr_depth; Iz < zLim + terr_depth + bce_thickness; Iz++) {
                    bce.push_back(ChVector3i(Ix, Iy, Iz));  // SPH particles above 0
                }
            }
        }

        ChFsiProblemSPH::GridPoints m_sph;
        ChFsiProblemSPH::GridPoints m_bce;

        //  Insert in cached sets
        for (auto& p : sph) {
            m_sph.insert(p);
        }

        //  Insert in cached sets
        for (auto& p : bce) {
            m_bce.insert(p);
        }

        double box_z = z_min - z_off;

        ChVector3d m_offset_sph = ChVector3d(rover_pos.x(),rover_pos.y(), 0.0) - ChVector3d(box.x() / 2, box.y() / 2, -box_z);
        ChVector3d m_offset_bce = ChVector3d(rover_pos.x(),rover_pos.y(), 0.0) - ChVector3d(box.x() / 2, box.y() / 2, -box_z);

        terr.SetSPHPoints(m_sph, m_offset_sph);
        terr.SetBCEPoints(m_bce, m_offset_bce);

        // terr.AddBoxContainer(box, rover_pos, BoxSide::);

    }


    static void InitializeCRMTerrain(PerseveranceUtils::RoverDefinition def, CRMTerrain& terrain, ChVector2d size, SoilParameters params, std::vector<std::string> mod_files, std::vector<std::string> ht_files, ChVector3d rover_pos, double step_size) 
    {
        // /*/////////////////////
        //  *  Initialize Terrain 
        // *//////////////////////
        
        double x_offset = 0.0;
        double y_offset = 0.0;
        double width = 0.0;
        double height = 0.0;

        auto compo_img = std::make_shared<rsvp::AverageCompositeData>();
        // auto compo_img = std::make_shared<rsvp::AlphaBlendingCompositeData>();


        for(const auto& mod_file : mod_files) {
            auto img = HeightmapParser::ParseModFile((mod_file));
            compo_img->add_image(img);
        }

        for(const auto& ht_file : ht_files) {
            auto img = HeightmapParser::ParseHeightmap((ht_file));
            compo_img->add_image(img);
        }
        
        
        // auto mesh = HeightmapParser::asChronoMesh(compo_img, 0.05f, 20, 20, rover_x, rover_y);
        // std::vector<chrono::ChTriangleMeshConnected> meshes { *mesh };
        // ChTriangleMeshConnected::WriteWavefront("test.obj", meshes);

        terrain.SetStepSizeCFD(step_size);
        terrain.SetGravitationalAcceleration(ChVector3d(0, 0, 3.7));

        HeightmapParser::Construct(terrain, compo_img, params.spacing, size, rover_pos, BoxSide::ALL & ~BoxSide::Z_POS);
        
        ChFsiSystemSPH& sysFSI = terrain.GetSystemFSI();

        auto aabb = terrain.GetSPHBoundingBox();


        // Set SPH parameters and soil material properties
        ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
        mat_props.density = params.density;
        mat_props.Young_modulus = params.youngs_modulus;
        mat_props.Poisson_ratio = params.poisson_ratio;
        mat_props.mu_I0 = 0.04;
        mat_props.mu_fric_s = params.friction;
        mat_props.mu_fric_2 = params.friction;
        mat_props.average_diam = 0.005;
        mat_props.cohesion_coeff = params.cohesion;
        terrain.SetElasticSPH(mat_props);

        // Set SPH solver parameters
        ChFsiFluidSystemSPH::SPHParameters sph_params;
        sph_params.integration_scheme = IntegrationScheme::RK2;
        sph_params.initial_spacing = params.spacing;
        sph_params.d0_multiplier = 1;
        sph_params.kernel_threshold = 0.8;
        sph_params.artificial_viscosity = 0.5;
        sph_params.consistent_gradient_discretization = false;
        sph_params.consistent_laplacian_discretization = false;
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
        sph_params.boundary_method = BoundaryMethod::ADAMI;
        terrain.SetSPHParameters(sph_params);


        /*
        * Initialize FSI wheels
        */

        // Add rover wheels as FSI bodies
        std::cout << "Create wheel BCE markers..." << std::endl;
        std::string mesh_filename_r = GetChronoDataFile("M2020/meshes/Wheel_Col.obj");
        std::shared_ptr<utils::ChBodyGeometry> geometry_r = std::make_shared<utils::ChBodyGeometry>();
        geometry_r->materials.push_back(ChContactMaterialData());
        geometry_r->coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, mesh_filename_r, VNULL));
        
        std::string mesh_filename_l = GetChronoDataFile("M2020/meshes/Wheel_Col_L.obj");
        std::shared_ptr<utils::ChBodyGeometry> geometry_l = std::make_shared<utils::ChBodyGeometry>();
        geometry_l->materials.push_back(ChContactMaterialData());
        geometry_l->coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, mesh_filename_l, VNULL));

        ChVector3d active_box_dim(0.6, 0.6, 0.6);
        terrain.SetActiveDomain(active_box_dim);

        for(int i = 0; i < 6; i++) {
            if(i < 3) {
                terrain.AddRigidBody(def.wheels[i],geometry_r,false);
            } else {
                terrain.AddRigidBody(def.wheels[i],geometry_l,false);
            }
        }
        terrain.Initialize();

    }


    static void InitializeCRMTerrain(PerseveranceUtils::RoverDefinition def, CRMTerrain& terrain, ChVector3d size, SoilParameters params, ChVector3d rover_pos, double step_size) 
    {

        // /*/////////////////////
        //  *  Initialize Terrain 
        // *//////////////////////
        
        double x_offset = 0.0;
        double y_offset = 0.0;
        double width = 0.0;
        double height = 0.0;
        
        // auto mesh = HeightmapParser::asChronoMesh(compo_img, 0.05f, 20, 20, rover_x, rover_y);
        // std::vector<chrono::ChTriangleMeshConnected> meshes { *mesh };
        // ChTriangleMeshConnected::WriteWavefront("test.obj", meshes);

        terrain.SetStepSizeCFD(step_size);
        terrain.SetGravitationalAcceleration(ChVector3d(0, 0, 3.7));
        
        HeightmapParser::Construct(terrain, params.spacing, size, rover_pos, BoxSide::ALL & ~BoxSide::Z_NEG);
        
        ChFsiSystemSPH& sysFSI = terrain.GetSystemFSI();

        auto aabb = terrain.GetSPHBoundingBox();


        // Set SPH parameters and soil material properties
        ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
        mat_props.density = params.density;
        mat_props.Young_modulus = params.youngs_modulus;
        mat_props.Poisson_ratio = params.poisson_ratio;
        mat_props.mu_I0 = 0.04;
        mat_props.mu_fric_s = params.friction;
        mat_props.mu_fric_2 = params.friction;
        mat_props.average_diam = 0.005;
        mat_props.cohesion_coeff = params.cohesion;
        terrain.SetElasticSPH(mat_props);

        // Set SPH solver parameters
        ChFsiFluidSystemSPH::SPHParameters sph_params;
        sph_params.integration_scheme = IntegrationScheme::RK2;
        sph_params.initial_spacing = params.spacing;
        sph_params.d0_multiplier = 1;
        sph_params.kernel_threshold = 0.8;
        sph_params.artificial_viscosity = 0.5;
        sph_params.consistent_gradient_discretization = false;
        sph_params.consistent_laplacian_discretization = false;
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
        sph_params.boundary_method = BoundaryMethod::ADAMI;
        terrain.SetSPHParameters(sph_params);


        /*
        * Initialize FSI wheels
        */

        // Add rover wheels as FSI bodies
        std::cout << "Create wheel BCE markers..." << std::endl;
        std::string mesh_filename_r = GetChronoDataFile("M2020/meshes/Wheel_Col.obj");
        std::shared_ptr<utils::ChBodyGeometry> geometry_r = std::make_shared<utils::ChBodyGeometry>();
        geometry_r->materials.push_back(ChContactMaterialData());
        geometry_r->coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, mesh_filename_r, VNULL));
        
        std::string mesh_filename_l = GetChronoDataFile("M2020/meshes/Wheel_Col_L.obj");
        std::shared_ptr<utils::ChBodyGeometry> geometry_l = std::make_shared<utils::ChBodyGeometry>();
        geometry_l->materials.push_back(ChContactMaterialData());
        geometry_l->coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, mesh_filename_l, VNULL));

        ChVector3d active_box_dim(0.6, 0.6, 0.6);
        terrain.SetActiveDomain(active_box_dim);

        for(int i = 0; i < 6; i++) {
            if(i < 3) {
                terrain.AddRigidBody(def.wheels[i],geometry_r,false);
            } else {
                terrain.AddRigidBody(def.wheels[i],geometry_l,false);
            }
        }
        terrain.Initialize();

    }
};

#endif  