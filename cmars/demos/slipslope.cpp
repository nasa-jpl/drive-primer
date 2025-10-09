#define INCL_VSG 0

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkTSDA.h"
#include <../thirdparty/nlohmann/json.hpp>


using json = nlohmann::json;

#if INCL_VSG == 1

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
#include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"

#endif

#include "perseverance_slip.h"
#include "perseverance_utils.h"
#include "heightmap_parser.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "perseverance_straight_drive_controller.h"
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
double t_fin = 20;
double t_settle = 5.0;
double offset_z = 0.1;

int main(int argc, char* argv[]) {

    json jsonData;
    double gravity_angle_deg;
    if(argc < 2) {
        std::cout << "Must supply input JSON string" << std::endl;
    }
    if(argc < 3) {
        std::cout << "Must supply gravity angle (deg)" << std::endl;
    }
    try {
        jsonData = json::parse(argv[1]);
        gravity_angle_deg = std::stof(argv[2]);

    } catch (const json::parse_error& e) {
        std::cerr << "Parse error: " << e.what() << std::endl;
    }
    

    // Set path to Chrono data directory
    SetChronoDataPath(std::getenv("CHRONO_DATA_PATH"));
    
    double bulk_density = jsonData["soil"]["bulk_density"];
    double cohesion = jsonData["soil"]["cohesion"];
    double friction = jsonData["soil"]["friction"];
    double youngs_modulus = jsonData["soil"]["youngs_modulus"];
    double poisson_ratio = jsonData["soil"]["poisson_ratio"];

    std::cout << "Loaded BD: " << bulk_density << " C:" << cohesion << " F: " 
                    << friction << " YM:" << youngs_modulus << " PR: " << poisson_ratio << std::endl;

    double spacing = jsonData["soil"]["spacing"];

    std::string output_dir = jsonData["results"]["trial_output_file"];
    // output_dir = output_dir +"/output.csv";

    double step_size = jsonData["integrator"]["step_size_mbd"];
    double step_size_cfd = jsonData["integrator"]["step_size_cfd"];
    double terr_size = jsonData["soil"]["size"];
    std::string integrator = jsonData["integrator"]["integrator"];
    
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

    ChVector3d pos = ChVector3d(0,0,0);

    auto def = PerseveranceUtils::InitializeRover(filename, ChFrame<>(pos, ChQuaterniond(1,0,0,0)), sys, 0.0, false, true, false);


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


    HeightmapParser::SoilParameters params {
        spacing, // Spacing
        bulk_density, // 1500, // Bulk density
        cohesion,  //5e3, // Cohesion
        friction,// //0.5, // Friction coefficient -or- tan(friction_angle)
        youngs_modulus, // Youngs modulus
        poisson_ratio  // Poisson Ratio
    };

    CRMTerrain terrain(sys,params.spacing);
    
    HeightmapParser::InitializeCRMTerrain(def, terrain, ChVector3d(6,6,0.2), params, ChVector3d(pos.x(), pos.y(), pos.z() + 0.1), step_size);

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
    auto col_callback = chrono_types::make_shared<ParticleHeightColorCallback>(-35,-30);
    visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::BROWN);
    
    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    visVSG->AttachPlugin(visFSI);
    visVSG->AttachSystem(&sys);
    visVSG->SetWindowTitle("M2020 - Slip Slope");
    visVSG->SetWindowSize(1280, 800);
    visVSG->SetWindowPosition(100, 100);
    visVSG->AddCamera(ChVector3d(rover_x + 10, rover_y + 10, rover_z), ChVector3d(rover_x, rover_y, rover_z));
    visVSG->SetLightIntensity(0.9f);
    visVSG->Initialize();
#endif
    // 5 - Simulation loop

    double time = 0;

    int sim_frame = 0;
    int render_frame = 0;
    bool started = false;

    PerseveranceLogger logger;
    logger.SetClock(0);
    logger.Initialize(def.chassis, &def.parser, output_dir, 1.0);

    PerseveranceSlip slip_monitor;
    slip_monitor.SetClock(0);
    slip_monitor.Initialize(&def.parser);

    PerseveranceStraightDriveController controller;
    controller.Initialize(&def.parser);

    double g_angle = gravity_angle_deg*(3.14/180.0);
    double g = 3.71;
    double g_x = -g * std::sin(g_angle);
    double g_y =  0.0; // -3.7 * std::sin(g_angle);
    double g_z =  g * std::cos(g_angle);

    sys.SetGravitationalAcceleration(ChVector3d(g_x,g_y,g_z));
    sysFSI.SetGravitationalAcceleration(ChVector3d(g_x,g_y,g_z));
    terrain.SetGravitationalAcceleration(ChVector3d(g_x,g_y,g_z));

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
        terrain.Advance(step_size);

        time += step_size;
        sim_frame++;
        

        if(time > t_settle) {
            controller.Advance({ def.chassis->GetFrameRefToAbs().GetPos(), def.chassis->GetFrameRefToAbs().GetRot() }, step_size);
        }        

        if(time > t_settle + 1.0) {
            slip_monitor.Advance(step_size);
            logger.Advance(slip_monitor.GetLastSlip(), step_size);

        }
        if(controller.IsComplete() || time - t_settle > t_fin) {
            return 0;
        }

    }

    return 0;
}
