
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkTSDA.h"

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_irrlicht/ChIrrTools.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "perseverance_utils.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::vsg3d;
using namespace chrono::irrlicht;


std::shared_ptr<ChLinkLockRevolute> AddRevoluteJoint(std::shared_ptr<ChBody> body_1,
                                                     std::shared_ptr<ChBody> body_2,
                                                     std::shared_ptr<ChBody> chassis,
                                                     const ChVector3d& rel_joint_pos,
                                                     const ChQuaternion<>& rel_joint_rot) {
    const ChFrame<>& X_GP = chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);                    // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                                    // global -> child

    auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
    joint->Initialize(body_1, body_2, ChFrame<>(X_GC.GetCoordsys().pos, X_GC.GetCoordsys().rot));
    chassis->GetSystem()->AddLink(joint);

    return joint;
}

int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath("../../resources/");
    


    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -3.7));
    // sys.SetTimestepperType(ChTimestepper::Type::NEWMARK);
    // sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    std::string filename = "M2020/m2020.urdf";
    auto def = PerseveranceUtils::InitializeRover(filename,"",sys, 0,0);

    auto parser = def.parser;
    
    float friction = 0.8f;
    float Y = 1e5f;
    float cr = 0.0f;
    double length = 10;
    double width = 10;
    double height = 0.1; 
    double offset = 2.0;

    auto ground_mat = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());
    ground_mat->SetFriction(friction);
    ground_mat->SetRestitution(cr);

    if (sys.GetContactMethod() == ChContactMethod::SMC) {
        std::static_pointer_cast<ChContactMaterialSMC>(ground_mat)->SetYoungModulus(Y);
    }

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->SetPos(ChVector3d(offset, 0, 0.0));
    ground->EnableCollision(true);

    auto ct_shape = chrono_types::make_shared<ChCollisionShapeBox>(ground_mat, length, width, 0.2);
    ground->AddCollisionShape(ct_shape);

    auto box = chrono_types::make_shared<ChVisualShapeBox>(length, width, 0.2);
    box->SetTexture(GetChronoDataFile("textures/checker1.png"), 4, 1);
    ground->AddVisualShape(box);


    sys.AddBody(ground);
    sys.GetCollisionSystem()->BindItem(ground);


    std::cout << "Finished Initializing" << std::endl;


    ChVisualSystemIrrlicht vis;
    vis.AttachSystem(&sys);
    vis.SetCameraVertical(CameraVerticalDir::Z);
    vis.SetWindowSize(1200, 800);
    vis.SetWindowTitle("M2020");


    vis.Initialize();
    vis.AddLogo();
    vis.AddCamera(ChVector3d(2, 2, -5), ChVector3d(0, 1, 0));
    vis.AddTypicalLights();

    
    // 5 - Simulation loop
    double step_size = 5e-3;

    vis.Initialize();

    // vis.EnableLinkDrawing(LinkDrawMode::LINK_REACT_FORCE);
    // vis.EnableLinkFrameDrawing(true);
    vis.EnableBodyFrameDrawing(true);
    // vis.EnableCollisionShapeDrawing(true);

    double time = 0;

    while (vis.Run()) {
        // Render scene
        vis.BeginScene();
        vis.Render();
        vis.EndScene();

        // Perform the integration step
        sys.DoStepDynamics(step_size);

        time += step_size;

        // if(joint_l->GetConstraintViolation().size() > 1) {
        //     std::cout << "WARN: Constraint violation LEFT DIFFERENTIAL " << joint_l->GetConstraintViolation().size() <<  std::endl;
        // }
        // if(joint_r->GetConstraintViolation().size() > 1) {
        //     std::cout << "WARN: Constraint violation RIGHT DIFFERENTIAL " << joint_r->GetConstraintViolation().size() << std::endl;
        // }

    }

    return 0;
}
