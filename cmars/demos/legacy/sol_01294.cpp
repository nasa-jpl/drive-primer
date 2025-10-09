#define INCL_VSG 1

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkTSDA.h"

#if INCL_VSG == 1

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
#include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"

#endif

#include "perseverance_utils.h"
#include "heightmap_parser.h"
// #include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "perseverance_goto_controller.h"
#include "perseverance_openloop_controller.h"
#include "perseverance_logger.h"


using namespace chrono;

#if INCL_VSG == 1
using namespace chrono::vsg3d;
#endif

using namespace chrono::vehicle;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

ChVector2d box;
//double t_init = 1500.0;
//double t_fin = 1800.0 - t_init; // + 200;
double t_settle = 10.0;
double t_fix = 3.0;
double offset_z = 0.1;
//double spacing = 0.07;

int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath("../../resources/");

    double bulk_density = std::stod(argv[1]);
    double cohesion = std::stod(argv[2]);
    double friction = std::stod(argv[3]);
    double youngs_modulus = std::stod(argv[4]);
    double poisson_ratio = std::stod(argv[5]);
    double spacing = std::stod(argv[6]);
    double t_init = std::stod(argv[7]);
    double t_fin = std::stod(argv[8]);
    double z_off = std::stod(argv[9]);
    std::string output_dir = argv[10];
    double step_size = std::stod(argv[11]);
    double terr_size = std::stod(argv[12]);
    std::string integrator = argv[13];

    box = ChVector2d{terr_size,terr_size};

    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 3.7));

    if(integrator == "NEWMARK") {
        sys.SetTimestepperType(ChTimestepper::Type::NEWMARK);
    }
    if(integrator == "EULER_IMPLICIT_LINEARIZED") {
        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    }
    if(integrator == "EULER_IMPLICIT_PROJECTED") {
        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
    }
   if(integrator == "HHT") {
        sys.SetTimestepperType(ChTimestepper::Type::HHT);
    }
     
    
    /*/////////////////////
     *  Initialize Rover 
    *//////////////////////
    std::string filename = "M2020/m2020.urdf";
    auto def = PerseveranceUtils::InitializeRover(t_init, filename, GetChronoDataFile("downlink/open_loop.csv"), sys, z_off, true, false);

    double rover_x = def.init_pose.GetPos().x();
    double rover_y = def.init_pose.GetPos().y();
    double rover_z = def.init_pose.GetPos().z();

    // /*/////////////////////
    //  *  Initialize Terrain 
    // *//////////////////////
    
    double x_offset = 0.0;
    double y_offset = 0.0;
    double width = 0.0;
    double height = 0.0;

    std::vector<std::string> mod_files = {
        "downlink/01294/N_LT_1294_XYM_SM2_RAS060_3632_AUTOGENJ06.mod",
        "downlink/01293/N_LT_1293_XYM_SM2_RAS060_3478_AUTOGENJ06.mod"
    };
    std::vector<std::string> ht_files = {
        "downlink/01294/NLM_1294_0781815035M991RAS_N0603619VCE_15950_0A02LLJ01.ht"
    };

    HeightmapParser::SoilParameters params {
        spacing, // Spacing
        bulk_density, // 1500, // Bulk density
        cohesion,  //5e3, // Cohesion
        friction,// //0.5, // Friction coefficient -or- tan(friction_angle)
        youngs_modulus, // Youngs modulus
        poisson_ratio  // Poisson Ratio
    };

    CRMTerrain terrain(sys,params.spacing);
    

    HeightmapParser::InitializeCRMTerrain(def,terrain,box,params,mod_files,ht_files,ChVector3d(rover_x,rover_y,rover_z),3e-4);

    std::cout << "Finished Initializing Terrain" << std::endl;

    double render_fps = 30;               // rendering FPS
    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = true;  // render boundary BCE markers
    bool visualization_rigid_bce = true;  // render wheel BCE markers

    ChFsiSystemSPH& sysFSI = terrain.GetSystemFSI();

#if INCL_VSG == 1
    auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
    visFSI->EnableFluidMarkers(visualization_sph);
    visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
    visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
    auto col_callback = chrono_types::make_shared<ParticleHeightColorCallback>(-80.676,-90.733);
    visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::BROWN);
    
    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    visVSG->AttachPlugin(visFSI);
    visVSG->AttachSystem(&sys);
    visVSG->SetWindowTitle("M2020 - Case: Sol 01294");
    visVSG->SetWindowSize(1280, 800);
    visVSG->SetWindowPosition(100, 100);
    visVSG->AddCamera(ChVector3d(rover_x + 10, rover_y + 10, rover_z), ChVector3d(rover_x, rover_y, rover_z));
    visVSG->SetLightIntensity(0.9f);
    visVSG->Initialize();
#endif
    // 5 - Simulation loop
    // double step_size = 3e-4;


    // vis.EnableLinkDrawing(LinkDrawMode::LINK_REACT_FORCE);
    // vis.EnableLinkFrameDrawing(true);
    // visVSG->EnableBodyFrameDrawing(true);
    // vis.EnableCollisionShapeDrawing(true);
    double time = 0;

    int sim_frame = 0;
    int render_frame = 0;
    bool started = false;

    // PerseveranceGotoController controller;
    // controller.Initialize(&def.parser);
    // controller.SetWaypoints({
    //     {ChVector3d(-199.74929491310294, -254.04715089684146, -84.4994751452405), 6.696 - 1.0, 1.0, 1.75},
    //     {ChVector3d(-195.742296, -260.101252, -86.012213), 7.260, 1.0, 1.75},
    //     {ChVector3d(-197.03672348781652,-265.17789779343,-87.68009881230944), 5.239, 1.0, 1.75}
    // });

    PerseveranceOpenLoopController controller;
    controller.Initialize(t_init, &def.parser, GetChronoDataFile("downlink/open_loop.csv"));

    PerseveranceLogger logger;
    logger.SetClock(t_init);
    logger.Initialize(def.chassis, output_dir, 1.0);

    bool fixed = true;
#if INCL_VSG == 1
    while (visVSG->Run()) {
#else 
    while (1) {
#endif
        // Render scene
#if INCL_VSG == 1
        if(time >= render_frame / render_fps) {
            visVSG->BeginScene();
            visVSG->Render();
            visVSG->EndScene();
            render_frame++;
        }
#endif

        // Perform the integration step
        // sys.DoStepDynamics(step_size);
        terrain.Advance(step_size);

        time += step_size;
        sim_frame++;
        
        if(fixed && time < t_fix) {
            def.chassis->SetFixed(true);
        } 
        if(fixed && time > t_fix) {
            fixed = false;
            def.chassis->SetFixed(false);
        }

        if(time > t_settle) {
            logger.Advance(0.0,step_size);
            controller.Advance({ def.chassis->GetFrameRefToAbs().GetPos(), def.chassis->GetFrameRefToAbs().GetRot() }, step_size);
        }        
        if(controller.IsComplete() || time - t_settle > t_fin) {
            return 0;
        }

    }

    return 0;
}
