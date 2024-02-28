// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Wei Hu, Jason Zhou
// Chrono::FSI demo to show usage of VIPER rover models on CRM granular terrain
// This demo uses a plug-in VIPER rover model from chrono::models
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/visualization/ChFsiVisualization.h"
#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::geometry;
using namespace chrono::viper;

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_Viper_Bulldozing/";

// If true, save as Wavefront OBJ; if false, save as VTK
bool save_obj = false;

// Physical properties of terrain particles
double iniSpacing = 0.02;
double kernelLength = 0.02;
double density = 1700.0;

// Total dimension of the simulation
double bxSim = 8.0;
double bySim = 4.0;
double bzSim = 0.12;

// Dimension of cylinder of sand that is dropped
double cylRadiusOut = 1.0;
double cylRadiusIn = 0.75;
double cylHalfHeight = 0.3;

// Rover initial location
// The cylinder of the sand axis is along the global z axis, so
// the rover is place far enough to the left of the cylinder
// -4, 0, 0.312
ChVector<> init_loc(-bxSim / 2.0 + 1.0, 0, -bzSim + 0.2);

// Define blade relative position to the rover COM
// TODO: See if there is a better way to do this, seems error prone
ChVector<> blade_rel_pos = ChVector<>(1.2, 0.0, -0.55);

// Blade rotation
// TODO: Play around with this to get optimal soil movement
double rot_ang_x = 0.0;
// double rot_ang_y = -30.0 / 180.0 * CH_C_PI;
double rot_ang_y = 0.0;
double rot_ang_z = 90.0 / 180.0 * CH_C_PI;

// Simulation time and stepsize
double total_time = 20.0;
double dT = 2.5e-4;

// Save data as csv files to see the results off-line using Paraview
bool output = true;
int out_fps = 20;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 100;

// Pointer to store the VIPER instance
std::shared_ptr<Viper> rover;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

// Wheel and blade obj files
std::string wheel_obj = "robot/viper/obj/viper_simplewheel.obj";
std::string blade_obj = "robot/viper/obj/blade_2.obj";

std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.2f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

// Forward declaration of helper functions
void SaveParaViewFiles(ChSystemFsi& sysFSI, ChSystemNSC& sysMBS, double mTime);
void CreateSolidPhase(ChSystemNSC& sysMBS, ChSystemFsi& sysFSI);

int main(int argc, char* argv[]) {
    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
        std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/rover"))) {
        std::cerr << "Error creating directory " << out_dir + "/rover" << std::endl;
        return 1;
    }

    // Create a physical system and a corresponding FSI system
    ChSystemNSC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    ChVector<> gravity = ChVector<>(0, 0, -9.81);
    sysMBS.Set_G_acc(gravity);
    sysFSI.Set_G_acc(gravity);

    // Read JSON file with simulation parameters
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Viper_Bulldozing.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc != 1) {
        std::cout << "usage: ./demo_ROBOT_Viper_SPH <json_file>" << std::endl;
        return 1;
    }
    sysFSI.ReadParametersFromFile(inputJson);

    // Set the initial particle spacing
    sysFSI.SetInitialSpacing(iniSpacing);

    // Set the SPH kernel length
    sysFSI.SetKernelLength(kernelLength);

    // Set the terrain density
    sysFSI.SetDensity(density);

    // Set the simulation stepsize
    sysFSI.SetStepSize(dT);

    // Set the simulation domain size
    sysFSI.SetContainerDim(ChVector<>(bxSim, bySim, bzSim));

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetDiscreType(false, false);

    // Set wall boundary condition
    sysFSI.SetWallBC(BceVersion::ADAMI);

    // Setup the solver based on the input value of the prameters
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);

    // Set the periodic boundary condition
    double initSpace0 = sysFSI.GetInitialSpacing();
    ChVector<> cMin(-bxSim * 2, -bySim * 2, -bzSim * 10);
    ChVector<> cMax(bxSim * 2, bySim * 2, bzSim * 20);
    sysFSI.SetBoundaries(cMin, cMax);

    // Set simulation data output length
    sysFSI.SetOutputLength(0);

    // Create an initial box for the terrain patch
    chrono::utils::GridSampler<> sampler(initSpace0);
    ChVector<> center(0, 0, bzSim + 0.4);

    std::vector<ChVector<>> points = sampler.SampleCylinderZ(center, cylRadiusOut, cylHalfHeight);

    // Add SPH particles from the sampler points to the FSI system
    std::cout << "Generate SPH particles (" << points.size() << ")" << std::endl;
    auto gz = std::abs(gravity.z());
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        double RR = points[i].x() * points[i].x() + points[i].y() * points[i].y();
        if (RR > cylRadiusIn * cylRadiusIn) {
            double pre_ini = sysFSI.GetDensity() * gz * (-points[i].z() + bzSim);
            double rho_ini = sysFSI.GetDensity() + pre_ini / (sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed());
            sysFSI.AddSPHParticle(points[i], rho_ini, pre_ini, sysFSI.GetViscosity(), ChVector<>(0., 0., 0.));
        }
    }

    // Create MBD and BCE particles for the solid domain
    std::cout << "Generate BCE markers" << std::endl;
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Get the body from the FSI system for visualization
    std::vector<std::shared_ptr<ChBody>>& FSI_Bodies = sysFSI.GetFsiBodies();
    auto Rover = FSI_Bodies[0];

    // Write position and velocity to file
    std::ofstream ofile;
    if (output)
        ofile.open(out_dir + "./body_position.txt");

        // Create run-time visualization
#ifndef CHRONO_OPENGL
    if (vis_type == ChVisualSystem::Type::OpenGL)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::OpenGL;
#endif

#if !defined(CHRONO_OPENGL) && !defined(CHRONO_VSG)
    render = false;
#endif

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
#endif
                break;
            }
        }

        visFSI->SetTitle("Viper on CRM terrain");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector<>(0, -3 * bySim, bzSim), ChVector<>(0, 0, 0));
        visFSI->SetCameraMoveScale(1.0f);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(
            chrono_types::make_shared<HeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f), 0, bzSim));
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));
    double time = 0.0;
    int current_step = 0;

    auto body = sysMBS.Get_bodylist()[0];

    ChTimer timer;
    while (time < total_time) {
        std::cout << current_step << "  time: " << time << "  sim. time: " << timer() << std::endl;

        rover->Update();

        std::cout << "  pos: " << body->GetPos() << std::endl;
        std::cout << "  vel: " << body->GetPos_dt() << std::endl;
        if (output) {
            ofile << time << "  " << body->GetPos() << "    " << body->GetPos_dt() << std::endl;
            if (current_step % output_steps == 0) {
                sysFSI.PrintParticleToFile(out_dir + "/particles");
                sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
                SaveParaViewFiles(sysFSI, sysMBS, time);
            }
        }

        // Render system
        if (render && current_step % render_steps == 0) {
            if (!visFSI->Render())
                break;
        }

        timer.start();
        sysFSI.DoStepDynamics_FSI();
        timer.stop();

        time += dT;
        current_step++;
    }

    if (output)
        ofile.close();

    return 0;
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies and their
// BCE representations are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemNSC& sysMBS, ChSystemFsi& sysFSI) {
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    // Create a body for the rigid soil container
    auto box = chrono_types::make_shared<ChBodyEasyBox>(bxSim, bySim, bzSim, 1000, false, false);
    box->SetPos(ChVector<>(0, 0, 0));
    box->SetBodyFixed(true);

    // This does not seem to do collision with the wheels for some reason
    chrono::utils::AddBoxContainer(box, ground_mat,                               //
                                   ChFrame<>(ChVector<>(0., 0., -bzSim), QUNIT),  //
                                   ChVector<>(bxSim, bySim, bzSim), 0.1,          //
                                   ChVector<int>(2, 2, -1),                       //
                                   false);
    box->SetCollide(true);
    // box->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 60, 45);
    sysMBS.Add(box);

    // Get the initial SPH particle spacing
    double initSpace0 = sysFSI.GetInitialSpacing();

    // Set common material Properties

    // Fluid-Solid Coupling at the walls via BCE particles
    // TODO: Since we are only moving across X, check if we need BCE particles along Y
    sysFSI.AddBoxContainerBCE(box,                                         //
                              ChFrame<>(ChVector<>(0, 0, -bzSim), QUNIT),  //
                              ChVector<>(bxSim, bySim, bzSim),             //
                              ChVector<int>(2, 2, -1));

    auto driver = chrono_types::make_shared<ViperSpeedDriver>(0.1, 0.5 * CH_C_PI);
    rover = chrono_types::make_shared<Viper>(&sysMBS, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    rover->Initialize(ChFrame<>(init_loc, QUNIT));

    // Create the wheel's BCE particles
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    trimesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    std::vector<ChVector<>> BCE_wheel;
    sysFSI.CreateMeshPoints(*trimesh, initSpace0, BCE_wheel);

    // Add BCE particles and mesh of wheels to the system
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> wheel_body;
        if (i == 0) {
            wheel_body = rover->GetWheel(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            wheel_body = rover->GetWheel(ViperWheelID::V_RF)->GetBody();
        }
        if (i == 2) {
            wheel_body = rover->GetWheel(ViperWheelID::V_LB)->GetBody();
        }
        if (i == 3) {
            wheel_body = rover->GetWheel(ViperWheelID::V_RB)->GetBody();
        }

        sysFSI.AddFsiBody(wheel_body);
        if (i == 0 || i == 2) {
            sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI)), true);
        } else {
            sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, QUNIT), true);
        }
    }
    // ===========================
    // Add the blade to the rover
    // ===========================

    // Add a blade on the chassis and create BCE particles
    std::shared_ptr<ChBodyAuxRef> rover_body = rover->GetChassis()->GetBody();
    std::string blade_name = "blade";

    ChVector<> rover_body_pos = rover_body->GetPos();
    auto blade_trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    blade_trimesh->LoadWavefrontMesh(GetChronoDataFile(blade_obj), true, true);
    blade_trimesh->RepairDuplicateVertexes(1e-9);

    // Create a visual asset for the blade
    auto visual_blade_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    visual_blade_shape->SetMesh(blade_trimesh);

    // Compute mass inertia from mesh
    // TODO: See if this can be computed reliably from the mesh rather than hard coded
    double mmass = 1.0;
    double mdensity = 1.0;
    ChVector<> mcog = ChVector<>(0.5, 0.0, -0.5);
    ChVector<> principal_I = ChVector<>(1.0, 1.0, 1.0);
    ChMatrix33<> principal_inertia_rot = ChMatrix33<>(1.0);
    auto blade_body = chrono_types::make_shared<ChBodyAuxRef>();
    blade_body->AddVisualShape(visual_blade_shape);
    blade_body->SetNameString(blade_name);

    // Get absolute blade pos
    ChVector<> blade_pos = rover_body_pos + blade_rel_pos;
    ChQuaternion<> blade_rot = Q_from_Euler123(ChVector<double>(rot_ang_x, rot_ang_y, rot_ang_z));

    // Set the COG coordinates to barycenter, without displacing the REF reference.
    // Make the COG frame a principal frame.
    blade_body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));
    // Set inertia
    std::cout << "\n"
              << "The mass of the blade is " << mmass * mdensity << "\n"
              << std::endl;
    blade_body->SetMass(mmass * mdensity);
    blade_body->SetInertiaXX(mdensity * principal_I);

    // Set the absolute position of the body:
    blade_body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(blade_pos), ChQuaternion<>(blade_rot)));
    blade_body->SetBodyFixed(false);

    // Set normie collision material
    auto blade_material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    blade_material->SetFriction(0.9);
    blade_material->SetRestitution(0.4);

    // Add blade collision
    auto blade_col_shape =
        chrono_types::make_shared<ChCollisionShapeTriangleMesh>(blade_material, blade_trimesh, false, false, 0.005);
    blade_body->AddCollisionShape(blade_col_shape);
    blade_body->SetCollide(true);

    sysMBS.AddBody(blade_body);

    // Fiz the blade to the rover
    auto fix_link = chrono_types::make_shared<ChLinkLockLock>();
    ChVector<> link_pos = blade_pos;
    fix_link->Initialize(rover_body, blade_body, ChCoordsys<>(link_pos, QUNIT));
    sysMBS.AddLink(fix_link);

    // Create BCE particles for the blade
    std::vector<ChVector<>> BCE_blade;
    sysFSI.CreateMeshPoints(*blade_trimesh, initSpace0, BCE_blade);
    sysFSI.AddFsiBody(blade_body);
    sysFSI.AddPointsBCE(blade_body, BCE_blade, ChFrame<>(VNULL, QUNIT), true);
}

//------------------------------------------------------------------
// Function to save the povray files of the MBD
//------------------------------------------------------------------
void SaveParaViewFiles(ChSystemFsi& sysFSI, ChSystemNSC& sysMBS, double mTime) {
    std::string rover_dir = out_dir + "/rover_bulldozing";
    std::string filename;
    static int frame_number = -1;
    frame_number++;

    // save the VIPER body to obj/vtk files
    for (int i = 0; i < 1; i++) {
        auto body = rover->GetChassis()->GetBody();
        ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
        ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = (GetChronoDataFile("robot/viper/obj/viper_chassis.obj"));
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/body_" + std::to_string(frame_number) + ".obj";
            std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
            geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/body_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->getCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->getCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->getIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->getIndicesVertexes())
                file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
            file << "CELL_TYPES " << nf << std::endl;
            for (size_t ii = 0; ii < nf; ii++)
                file << "5 " << std::endl;
            file.close();
        }
    }

    // save the wheels to obj/vtk files
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> body;
        if (i == 0) {
            body = rover->GetWheel(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            body = rover->GetWheel(ViperWheelID::V_RF)->GetBody();
        }
        if (i == 2) {
            body = rover->GetWheel(ViperWheelID::V_LB)->GetBody();
        }
        if (i == 3) {
            body = rover->GetWheel(ViperWheelID::V_RB)->GetBody();
        }

        ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
        ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();
        if (i == 0 || i == 2) {
            body_rot.Cross(body_rot, Q_from_AngZ(CH_C_PI));
        }

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = GetChronoDataFile(wheel_obj);
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/wheel_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".obj";
            std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
            geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/wheel_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->getCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->getCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->getIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->getIndicesVertexes())
                file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
            file << "CELL_TYPES " << nf << std::endl;
            for (size_t ii = 0; ii < nf; ii++)
                file << "5 " << std::endl;
            file.close();
        }
    }

    // save the steering rod to obj/vtk files
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> body;
        if (i == 0) {
            body = rover->GetUpright(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            body = rover->GetUpright(ViperWheelID::V_RF)->GetBody();
        }
        if (i == 2) {
            body = rover->GetUpright(ViperWheelID::V_LB)->GetBody();
        }
        if (i == 3) {
            body = rover->GetUpright(ViperWheelID::V_RB)->GetBody();
        }
        ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
        ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = "";
        if (i == 0 || i == 2) {
            obj_path = GetChronoDataFile("robot/viper/obj/viper_L_steer.obj");
        }
        if (i == 1 || i == 3) {
            obj_path = GetChronoDataFile("robot/viper/obj/viper_R_steer.obj");
        }
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/steerRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".obj";
            std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
            geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/steerRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->getCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->getCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->getIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->getIndicesVertexes())
                file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
            file << "CELL_TYPES " << nf << std::endl;
            for (size_t ii = 0; ii < nf; ii++)
                file << "5 " << std::endl;
            file.close();
        }
    }

    // save the lower rod to obj/vtk files
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> body;
        if (i == 0) {
            body = rover->GetLowerArm(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            body = rover->GetLowerArm(ViperWheelID::V_RF)->GetBody();
        }
        if (i == 2) {
            body = rover->GetLowerArm(ViperWheelID::V_LB)->GetBody();
        }
        if (i == 3) {
            body = rover->GetLowerArm(ViperWheelID::V_RB)->GetBody();
        }
        ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
        ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = "";
        if (i == 0 || i == 2) {
            obj_path = GetChronoDataFile("robot/viper/obj/viper_L_bt_sus.obj");
        }
        if (i == 1 || i == 3) {
            obj_path = GetChronoDataFile("robot/viper/obj/viper_R_bt_sus.obj");
        }
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/lowerRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".obj";
            std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
            geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/lowerRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->getCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->getCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->getIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->getIndicesVertexes())
                file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
            file << "CELL_TYPES " << nf << std::endl;
            for (size_t ii = 0; ii < nf; ii++)
                file << "5 " << std::endl;
            file.close();
        }
    }

    // save the upper rod to obj/vtk files
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> body;
        if (i == 0) {
            body = rover->GetUpperArm(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            body = rover->GetUpperArm(ViperWheelID::V_RF)->GetBody();
        }
        if (i == 2) {
            body = rover->GetUpperArm(ViperWheelID::V_LB)->GetBody();
        }
        if (i == 3) {
            body = rover->GetUpperArm(ViperWheelID::V_RB)->GetBody();
        }
        ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
        ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = "";
        if (i == 0 || i == 2) {
            obj_path = GetChronoDataFile("robot/viper/obj/viper_L_up_sus.obj");
        }
        if (i == 1 || i == 3) {
            obj_path = GetChronoDataFile("robot/viper/obj/viper_R_up_sus.obj");
        }

        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/upperRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".obj";
            std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
            geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/upperRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->getCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->getCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->getIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->getIndicesVertexes())
                file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
            file << "CELL_TYPES " << nf << std::endl;
            for (size_t ii = 0; ii < nf; ii++)
                file << "5 " << std::endl;
            file.close();
        }
    }
    // ===============================
    // Save blade to obj and vtk files
    // ===============================

    auto blade_body = sysMBS.SearchBody("blade");
    ChFrame<> blade_ref_frame = blade_body->GetFrame_REF_to_abs();
    ChVector<> blade_pos = blade_ref_frame.GetPos();      // body->GetPos();
    ChQuaternion<> blade_rot = blade_ref_frame.GetRot();  // body->GetRot();

    auto blade_mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    std::string blade_obj_path = GetChronoDataFile(blade_obj);
    double scale_ratio = 1.0;
    blade_mmesh->LoadWavefrontMesh(blade_obj_path, false, true);
    blade_mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    blade_mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    double mmass;
    ChVector<> mcog;
    ChMatrix33<> minertia;
    blade_mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
    blade_mmesh->Transform(blade_pos, ChMatrix33<>(blade_rot));  // rotate the mesh based on the orientation of body

    if (save_obj) {
        filename = rover_dir + "/blade_" + std::to_string(frame_number) + ".obj";
        std::vector<geometry::ChTriangleMeshConnected> meshes = {*blade_mmesh};
        geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
    } else {
        filename = rover_dir + "/blade_" + std::to_string(frame_number) + ".vtk";
        std::ofstream file;
        file.open(filename);
        file << "# vtk DataFile Version 2.0" << std::endl;
        file << "VTK from simulation" << std::endl;
        file << "ASCII" << std::endl;
        file << "DATASET UNSTRUCTURED_GRID" << std::endl;
        auto nv = blade_mmesh->getCoordsVertices().size();
        file << "POINTS " << nv << " float" << std::endl;
        for (auto& v : blade_mmesh->getCoordsVertices())
            file << v.x() << " " << v.y() << " " << v.z() << std::endl;
        auto nf = blade_mmesh->getIndicesVertexes().size();
        file << "CELLS " << nf << " " << 4 * nf << std::endl;
        for (auto& f : blade_mmesh->getIndicesVertexes())
            file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
        file << "CELL_TYPES " << nf << std::endl;
        for (size_t ii = 0; ii < nf; ii++)
            file << "5 " << std::endl;
        file.close();
    }

    // save box obstacle to vtk files
    double lx = 0.1;
    double ly = 0.25;
    double lz = 0.05;
    ChVector<double> Node1 = ChVector<double>(-lx, -ly, -lz);
    ChVector<double> Node2 = ChVector<double>(lx, -ly, -lz);
    ChVector<double> Node3 = ChVector<double>(lx, -ly, lz);
    ChVector<double> Node4 = ChVector<double>(-lx, -ly, lz);
    ChVector<double> Node5 = ChVector<double>(-lx, ly, -lz);
    ChVector<double> Node6 = ChVector<double>(lx, ly, -lz);
    ChVector<double> Node7 = ChVector<double>(lx, ly, lz);
    ChVector<double> Node8 = ChVector<double>(-lx, ly, lz);

    for (int i = 0; i < 2; i++) {
        filename = rover_dir + "/obstacle_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
        std::ofstream file;
        file.open(filename);
        file << "# vtk DataFile Version 2.0" << std::endl;
        file << "VTK from simulation" << std::endl;
        file << "ASCII" << std::endl;
        file << "DATASET POLYDATA" << std::endl;

        file << "POINTS " << 8 << " "
             << "float" << std::endl;
        auto Body = sysMBS.Get_bodylist()[i + 2 + 16];
        ChVector<> center = Body->GetPos();
        ChMatrix33<> Rotation = Body->GetRot();
        ChVector<double> vertex1 = Rotation * Node1 + center;
        ChVector<double> vertex2 = Rotation * Node2 + center;
        ChVector<double> vertex3 = Rotation * Node3 + center;
        ChVector<double> vertex4 = Rotation * Node4 + center;
        ChVector<double> vertex5 = Rotation * Node5 + center;
        ChVector<double> vertex6 = Rotation * Node6 + center;
        ChVector<double> vertex7 = Rotation * Node7 + center;
        ChVector<double> vertex8 = Rotation * Node8 + center;
        file << vertex1.x() << " " << vertex1.y() << " " << vertex1.z() << "\n";
        file << vertex2.x() << " " << vertex2.y() << " " << vertex2.z() << "\n";
        file << vertex3.x() << " " << vertex3.y() << " " << vertex3.z() << "\n";
        file << vertex4.x() << " " << vertex4.y() << " " << vertex4.z() << "\n";
        file << vertex5.x() << " " << vertex5.y() << " " << vertex5.z() << "\n";
        file << vertex6.x() << " " << vertex6.y() << " " << vertex6.z() << "\n";
        file << vertex7.x() << " " << vertex7.y() << " " << vertex7.z() << "\n";
        file << vertex8.x() << " " << vertex8.y() << " " << vertex8.z() << "\n";

        file << "POLYGONS " << 6 << " " << 30 << std::endl;
        file << "4 " << 0 << " " << 1 << " " << 2 << " " << 3 << "\n";
        file << "4 " << 0 << " " << 1 << " " << 5 << " " << 4 << "\n";
        file << "4 " << 0 << " " << 4 << " " << 7 << " " << 3 << "\n";
        file << "4 " << 4 << " " << 5 << " " << 6 << " " << 7 << "\n";
        file << "4 " << 1 << " " << 5 << " " << 6 << " " << 2 << "\n";
        file << "4 " << 3 << " " << 2 << " " << 6 << " " << 7 << "\n";
    }

    // save rigid body position and rotation
    for (int i = 1; i < sysMBS.Get_bodylist().size(); i++) {
        auto body = sysMBS.Get_bodylist()[i];
        ChFrame<> ref_frame = body->GetFrame_REF_to_abs();
        ChVector<> pos = ref_frame.GetPos();
        ChQuaternion<> rot = ref_frame.GetRot();
        ChVector<> vel = body->GetPos_dt();

        std::string delim = ",";
        filename = rover_dir + "/body_pos_rot_vel" + std::to_string(i) + ".csv";
        std::ofstream file;
        if (sysMBS.GetChTime() > 0)
            file.open(filename, std::fstream::app);
        else {
            file.open(filename);
            file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim << "q1" << delim
                 << "q2" << delim << "q3" << delim << "Vx" << delim << "Vy" << delim << "Vz" << std::endl;
        }

        file << sysMBS.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim << rot.e0()
             << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << delim << vel.x() << delim << vel.y()
             << delim << vel.z() << std::endl;

        file.close();
    }

    std::cout << "-------------------------------------" << std::endl;
    std::cout << " Output frame:  " << frame_number << std::endl;
    std::cout << " Time:          " << mTime << std::endl;
    std::cout << "-------------------------------------" << std::endl;
}
