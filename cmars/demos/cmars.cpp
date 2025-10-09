#define INCL_VSG 1

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
double t_settle = 8.0;
double t_fix = 3.0;
// double t_drop = 3.0;
double offset_z = 0.1;

int main(int argc, char* argv[]) {

    bool render = false;
    json jsonData;
    bool loaded = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if(arg == "--render" || arg == "-r") {
            render = true;
        } else {
            try{
                jsonData = json::parse(argv[i]);
                loaded = true;
            } catch(const json::parse_error& e) {
                std::cerr << "Parse error: " << e.what() << std::endl;
            }
        }
    }

    if (!loaded) {
        std::cout << "Must provide json" << std::endl;
        return 1;
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

    double t_init = jsonData["incon"]["t_init"];
    double t_fin = jsonData["incon"]["t_fin"];
    double z_off = jsonData["incon"]["z_off"];
    std::string output_dir = jsonData["results"]["trial_output_file"];
    // output_dir = output_dir +"/output.csv";

    double step_size = jsonData["integrator"]["step_size_mbd"];
    double step_size_cfd = jsonData["integrator"]["step_size_cfd"];
    double terr_size = jsonData["soil"]["size"];
    std::string integrator = jsonData["integrator"]["integrator"];


    std::vector<std::string> mod_files;
    for (const auto& f : jsonData["downlink"]["mod"]) {
        mod_files.push_back(f);
    }
    

    std::vector<std::string> ht_files;
    for (const auto& f : jsonData["downlink"]["ht"]) {
        ht_files.push_back(f);
    }
    
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
    
    std::string traj_input_dir = jsonData["downlink"]["sim_input_dir"];
    

    std::string filename = "M2020/m2020.urdf";
    auto def = PerseveranceUtils::InitializeRover(t_init, filename, traj_input_dir, sys, z_off, true, true, false);


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
        spacing, 
        bulk_density,
        cohesion,
        friction,
        youngs_modulus, 
        poisson_ratio  
    };

    CRMTerrain terrain(sys,params.spacing);

    terrain.GetFluidSystemSPH().EnableCudaErrorCheck(false);
    
    HeightmapParser::InitializeCRMTerrain(def,terrain,box,params,mod_files,ht_files,
                                            ChVector3d(rover_x,rover_y,rover_z),step_size_cfd);

    
    std::cout << "Finished Initializing Terrain" << std::endl;

    double render_fps = 15;               // rendering FPS
    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = true;  // render boundary BCE markers
    bool visualization_rigid_bce = true;  // render wheel BCE markers

    ChFsiSystemSPH& sysFSI = terrain.GetSystemFSI();

#if INCL_VSG == 1
    
    auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();

    if(render) {
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
        visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
        auto col_callback = chrono_types::make_shared<ParticleHeightColorCallback>(-35,-30);
        visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::BROWN);
        
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sys);
        visVSG->SetWindowTitle("M2020");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(rover_x + 10, rover_y + 10, rover_z), ChVector3d(rover_x, rover_y, rover_z));
        visVSG->SetLightIntensity(0.9f);
        visVSG->Initialize();
    }
#endif
    // 5 - Simulation loop

    double time = 0;

    int sim_frame = 0;
    int render_frame = 0;
    bool started = false;

    std::string control_input_dir = jsonData["downlink"]["control_input_dir"];

    PerseveranceOpenLoopController controller;
    controller.Initialize(t_init, &def.parser, control_input_dir);

    PerseveranceLogger logger;
    logger.SetClock(t_init);
    logger.Initialize(def.chassis, &def.parser, output_dir, 1.0);

    PerseveranceSlip slip_monitor;
    slip_monitor.SetClock(t_init);
    slip_monitor.Initialize(&def.parser);
   
    auto rocker_right = def.parser.GetChBody("Body_RockerRight"); 
    auto rocker_left = def.parser.GetChBody("Body_RockerLeft");
    auto bogie_right = def.parser.GetChBody("Body_BogieRight"); 
    auto bogie_left = def.parser.GetChBody("Body_BogieLeft");

    
    bool fixed = true;
#if INCL_VSG == 1
    while ((render && visVSG->Run()) || !render) {
#else 
    while (1) {
#endif
        // Render scene
#if INCL_VSG == 1
        if(render && time >= render_frame / render_fps) {
            visVSG->BeginScene();
            visVSG->Render();
            visVSG->EndScene();
            render_frame++;
        }
#endif
        terrain.Advance(step_size);

        time += step_size;
        sim_frame++;

        
        if(time < t_fix) {
            sys.SetGravitationalAcceleration(ChVector3d(0, 0, 3.7)); // Can relax this if suspension is causing trouble  
            def.chassis->ForceToRest();
            bogie_left->ForceToRest();
            bogie_right->ForceToRest();
            rocker_left->ForceToRest();
            rocker_right->ForceToRest();

            def.chassis->SetAngVelLocal(ChVector3d(0,0,0));
        }

        if(time > t_settle) {
            slip_monitor.Advance(step_size);
            logger.Advance(slip_monitor.GetLastSlip(), step_size);
            controller.Advance({ def.chassis->GetFrameRefToAbs().GetPos(), def.chassis->GetFrameRefToAbs().GetRot() }, step_size);
        }        
        if(controller.IsComplete() || time - t_settle > t_fin) {
            // return 0; // This was causing exit code -11, not sure why
            exit(0);
        }

    }

    return 0;
}
