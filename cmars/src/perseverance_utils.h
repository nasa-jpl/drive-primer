#ifndef PERSEVERENCE_UTILS_H
#define PERSEVERENCE_UTILS_H

#include "chrono_parsers/ChParserURDF.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkLockGear.h"

using namespace chrono::parsers;

using namespace chrono;

#if INCL_VSG == 1
using namespace chrono::vsg3d;
#endif

class PerseveranceUtils {

public:

    struct RoverDefinition {
        ChParserURDF parser;
        std::vector<std::shared_ptr<ChBody>> wheels;
        std::vector<std::shared_ptr<ChBody>> steers;
        std::vector<std::shared_ptr<ChLinkBase>> steer_joints;
        std::shared_ptr<ChBody> chassis;
        ChFrame<> init_pose;
    };

    /*
    * Iniialize the rover within the given system
    */

    static RoverDefinition InitializeRover(double t_init, std::string filename, std::string open_loop_csv, ChSystem& sys, double z_off, bool position_based_control, bool verbose=true, bool fixed = false) {
        ChFrame<> pose = InitializeRoverIncons(t_init, open_loop_csv, z_off);
        return InitializeRover(filename, pose, sys, z_off, position_based_control, verbose, fixed); 
    }


    /*
    * Iniialize the rover within the given system
    */

    static RoverDefinition InitializeRover(std::string filename, ChFrame<> pose, ChSystem& sys, double z_off, bool position_based_control, bool verbose=true, bool fixed = false) {
        ChParserURDF parser(GetChronoDataFile(filename));
        
        InitializeArmJoints(parser);
        // Setup control modes for actuated joints
        InitializeJointControl(parser, position_based_control);

        if(verbose) {
            parser.PrintModelBodyTree();
            parser.PrintModelBodies();
            parser.PrintModelJoints();
        }
       
        parser.SetRootInitPose(pose);

        // Parse tree
        parser.PopulateSystem(sys);

        // Must be called after PopulateSystem
        parser.GetRootChBody()->SetFixed(fixed);

        parser.GetRootChBody()->EnableCollision(true);

        // Robot bounding box 
        if(verbose) {
            auto aabb_coll = parser.GetCollisionBoundingBox();
            auto aabb_vis = parser.GetVisualizationBoundingBox();
            std::cout << "Collision AABB" << std::endl;
            std::cout << "  min: " << aabb_coll.min << std::endl;
            std::cout << "  max: " << aabb_coll.max << std::endl;
            std::cout << "Visualization AABB" << std::endl;
            std::cout << "  min: " << aabb_vis.min << std::endl;
            std::cout << "  max: " << aabb_vis.max << std::endl;
        }

        // Get commonly used bodies for returning ('rover definition')
        auto WRF = parser.GetChBody("Body_WheelRightFront");
        auto WRC = parser.GetChBody("Body_WheelRightMiddle");
        auto WRB = parser.GetChBody("Body_WheelRightRear");
        auto WLF = parser.GetChBody("Body_WheelLeftFront");
        auto WLC = parser.GetChBody("Body_WheelLeftMiddle");
        auto WLB = parser.GetChBody("Body_WheelLeftRear");
        auto SRF = parser.GetChBody("Body_SteerRightFront");
        auto SRC = parser.GetChBody("Body_SteerRightMiddle");
        auto SRB = parser.GetChBody("Body_SteerRightRear");
        auto SLF = parser.GetChBody("Body_SteerLeftFront");
        auto SLC = parser.GetChBody("Body_SteerLeftMiddle");
        auto SLB = parser.GetChBody("Body_SteerLeftRear");
        auto RF_STEER = parser.GetChLink("RF_STEER");
        auto RR_STEER = parser.GetChLink("RR_STEER");
        auto LF_STEER = parser.GetChLink("LF_STEER");
        auto LR_STEER = parser.GetChLink("LR_STEER");


        std::vector<std::shared_ptr<ChBody>> wheels {
            WRF, WRC, WRB, WLF, WLC, WLB
        };

        std::vector<std::shared_ptr<ChBody>> steers {
            SRF, SRC, SRB, SLF, SLC, SLB
        };

        std::vector<std::shared_ptr<ChLinkBase>> steer_joints { 
            RF_STEER, RR_STEER, LF_STEER, LR_STEER 
        };

        InitializeWheels(sys, wheels);
        SetArmJointIncons(parser);
        InitializeDiffBar(sys,parser);

        return { parser, wheels, steers, steer_joints, parser.GetRootChBody(), pose };
    }
    static void InitializeJointControl(ChParserURDF& parser, bool position_based = false) {

        // Make default position
        // parser.SetAllJointsActuationType(ChParserURDF::ActuationType::POSITION); 

        parser.SetJointActuationType("RF_STEER", ChParserURDF::ActuationType::POSITION);
        parser.SetJointActuationType("RR_STEER", ChParserURDF::ActuationType::POSITION);
        parser.SetJointActuationType("LF_STEER", ChParserURDF::ActuationType::POSITION);
        parser.SetJointActuationType("LR_STEER", ChParserURDF::ActuationType::POSITION);

        if(position_based) {
            parser.SetJointActuationType("RF_DRIVE", ChParserURDF::ActuationType::POSITION);
            parser.SetJointActuationType("RM_DRIVE", ChParserURDF::ActuationType::POSITION);
            parser.SetJointActuationType("RR_DRIVE", ChParserURDF::ActuationType::POSITION);
            parser.SetJointActuationType("LF_DRIVE", ChParserURDF::ActuationType::POSITION);
            parser.SetJointActuationType("LM_DRIVE", ChParserURDF::ActuationType::POSITION);
            parser.SetJointActuationType("LR_DRIVE", ChParserURDF::ActuationType::POSITION);
        } else {
            parser.SetJointActuationType("LF_DRIVE",ChParserURDF::ActuationType::SPEED);
            parser.SetJointActuationType("LM_DRIVE",ChParserURDF::ActuationType::SPEED);
            parser.SetJointActuationType("LR_DRIVE",ChParserURDF::ActuationType::SPEED);
            parser.SetJointActuationType("RF_DRIVE",ChParserURDF::ActuationType::SPEED);
            parser.SetJointActuationType("RM_DRIVE",ChParserURDF::ActuationType::SPEED);
            parser.SetJointActuationType("RR_DRIVE",ChParserURDF::ActuationType::SPEED);
        }
    

    }

    static void InitializeWheels(ChSystem& sys, const std::vector<std::shared_ptr<ChBody>>& wheels) {
        ChContactMaterialData mat;
        mat.kn = 2.5e6;
        auto cmat = mat.CreateMaterial(sys.GetContactMethod());

        // for(const auto& w : wheels) {
        //     w->AddAccumulator();
        //     w->EnableCollision(true);
        //     w->GetCollisionModel()->SetAllShapesMaterial(cmat);
        // }
    }

    static void InitializeArmJoints(ChParserURDF& parser) {
        parser.SetJointActuationType("JOINT1_ENC", ChParserURDF::ActuationType::POSITION);
        parser.SetJointActuationType("JOINT2_ENC", ChParserURDF::ActuationType::POSITION);
        parser.SetJointActuationType("JOINT3_ENC", ChParserURDF::ActuationType::POSITION);
        parser.SetJointActuationType("JOINT4_ENC", ChParserURDF::ActuationType::POSITION);
        parser.SetJointActuationType("JOINT5_ENC", ChParserURDF::ActuationType::POSITION);

    }

    static void SetArmJointIncons(ChParserURDF& parser) {

        auto joint1_enc = std::make_shared<ChFunctionConst>(1.57291036891924763);
        auto joint2_enc = std::make_shared<ChFunctionConst>(-0.28614050738365315);
        auto joint3_enc = std::make_shared<ChFunctionConst>(-2.81942165195030592);
        auto joint4_enc = std::make_shared<ChFunctionConst>(3.11076404826112807);
        auto joint5_enc = std::make_shared<ChFunctionConst>(4.86054363393466282);

        parser.SetMotorFunction("JOINT1_ENC", joint1_enc);
        parser.SetMotorFunction("JOINT2_ENC", joint2_enc);
        parser.SetMotorFunction("JOINT3_ENC", joint3_enc);
        parser.SetMotorFunction("JOINT4_ENC", joint4_enc);
        parser.SetMotorFunction("JOINT5_ENC", joint5_enc);

    }

    static ChFrame<> InitializeRoverIncons(double t_init, std::string csv, double z_off) {
         std::ifstream inputFile(csv);

        if (!inputFile.is_open()) {
            throw std::runtime_error("Error opening input CSV");
        }
        
        bool initialized = false;
        for (std::string line; std::getline(inputFile, line);) {
            if(!initialized) {
                initialized = true; // Skip first line
                continue;
            }

            std::istringstream ss(std::move(line));
            std::vector<std::string> row;

            int i = 0;
            std::vector<double> tokens;

            for(std::string token; std::getline(ss,token,',');) {
                // std::cout << i << std::endl;
                // std::cout << token << std::endl;

                tokens.push_back(std::stod(token));
                i++;
            }
            
            if(fabs(tokens[0] - t_init) < 0.1) {
                double x = tokens[2];
                double y = tokens[3];
                double z = tokens[4];
                double qx = tokens[5];
                double qy = tokens[6];
                double qz = tokens[7];
                double qw = tokens[8];

                ChFrame<> pose (ChVector3d(x,y,z-z_off), ChQuaterniond(qw,qx,qy,qz));

                std::cout << pose << std::endl;
                // ChVector3d local_up(0, 0, z_off);  
                // ChVector3d world_up = pose.GetRot().Rotate(local_up);
                // pose.SetPos(pose.GetPos() - world_up);

                return pose;
            }
        }
        return ChFrame<>(ChVector3d(0,0,0), ChQuaterniond(1,0,0,0));
    }

    static void InitializeDiffBar(ChSystem& sys, ChParserURDF& parser)  {
        std::shared_ptr<ChLinkBase> ldiff = parser.GetChLink("LEFT_DIFFERENTIAL");
        std::shared_ptr<ChLinkBase> rdiff = parser.GetChLink("RIGHT_DIFFERENTIAL");

        std::shared_ptr<ChBody> ldiff_b2 = parser.GetChBody("Body_RockerLeft");
        std::shared_ptr<ChBody> rdiff_b2 = parser.GetChBody("Body_RockerRight");

        auto gear = chrono_types::make_shared<ChLinkLockGear>();

        gear->SetTransmissionRatio(-1.0);
        // gear->SetEpicyclic(true);
        gear->SetPhase(0.0);
        // gear->SetEnforcePhase(true);
        
        ChFrame<double> shaft1_frame;
        ChFrame<double> shaft2_frame;

        auto rev1_frame = ldiff_b2->GetFrameCOMToAbs();  // Frame on moving body
        auto rev2_frame = rdiff_b2->GetFrameCOMToAbs();  // Frame on moving body

        
        ChQuaternion<double> y_axis_rotation;
        y_axis_rotation.SetFromAngleX(3.14/2);
        
        shaft1_frame.SetPos(ChVector3d(0, 0, 0));
        shaft1_frame.SetRot(y_axis_rotation);
        
        shaft2_frame.SetPos(ChVector3d(0, 0, 0));
        shaft2_frame.SetRot(y_axis_rotation);
            
        gear->SetFrameShaft1(shaft1_frame);
        gear->SetFrameShaft2(shaft2_frame);
        

        ChFrame<> link_coords;
        link_coords.SetPos(ChVector3d(0, 0, 0));  // Position of the link reference
        link_coords.SetRot(y_axis_rotation);  // Orientation (identity)
        
        // Initialize the link between the two bodies with coordinate system
        gear->Initialize(ldiff_b2, rdiff_b2, link_coords);
        
        // sys.AddLink(gear);
    }
};

#endif