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
// Author: Wei Hu, Radu Serban
// =============================================================================

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/ChConfig.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fsi/ChSystemFsi.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualization.h"
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

#include <cuda_runtime.h>

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;

double smalldis = 1.0e-9;
// double bxDim = 4.0 + smalldis;
// double byDim = 0.6 + smalldis;
// double bzDim = 4.0 + smalldis;

double bxDim = 5.0 + smalldis;
double byDim = 0.8 + smalldis;
double bzDim = 0.2 + smalldis;
// Variables from DEM sim
double safe_x = 1.;
double z_adv_targ = 0.2;

double wheel_radius = 0.2339;
double wheel_slip = 0.0;
double wheel_width = 0.293;
double grouser_wide = 0.005;
int grouser_num = 24;
std::string wheel_obj = "robot/viper/obj/viper_wheel_right_dem.obj";

double total_time = 10.0;

// linear actuator and angular actuator
auto actuator = chrono_types::make_shared<ChLinkLockLinActuator>();
auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();

// Save data as csv files to see the results off-line using Paraview
bool output = true;
int out_fps = 10;

int print_fps = 100;

// Output directories and settings
std::string out_dir = GetChronoOutputPath() + "FSI_Single_Wheel_Test";

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = false;
float render_fps = 100;

// Verbose terminal output
bool verbose_fsi = true;
bool verbose = true;

//------------------------------------------------------------------
// Function to save wheel to Paraview VTK files
//------------------------------------------------------------------
void WriteWheelVTK(const std::string& filename, ChTriangleMeshConnected& mesh, const ChFrame<>& frame) {
    std::ofstream outf;
    outf.open(filename);
    outf << "# vtk DataFile Version 2.0" << std::endl;
    outf << "VTK from simulation" << std::endl;
    outf << "ASCII" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID" << std::endl;
    outf << "POINTS " << mesh.GetCoordsVertices().size() << " "
         << "float" << std::endl;
    for (auto& v : mesh.GetCoordsVertices()) {
        auto w = frame.TransformPointLocalToParent(v);
        outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
    }
    auto nf = mesh.GetIndicesVertexes().size();
    outf << "CELLS " << nf << " " << 4 * nf << std::endl;
    for (auto& f : mesh.GetIndicesVertexes()) {
        outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
    }
    outf << "CELL_TYPES " << nf << std::endl;
    for (int i = 0; i < nf; i++) {
        outf << "5 " << std::endl;
    }
    outf.close();
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS,
                      ChSystemFsi& sysFSI,
                      ChVector3d& wheel_IniPos,
                      ChVector3d& wheel_IniVel,
                      double wheel_AngVel,
                      bool render_wheel,
                      double wheel_radius,
                      double grouser_height,
                      double wheel_wide,
                      double grouser_wide,
                      int grouser_num,
                      double kernelLength,
                      double total_mass = 17.5,
                      double iniSpacing = 0.01) {
    // Common contact material
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e9);
    cmaterial->SetFriction(1.f);
    cmaterial->SetRestitution(0.4f);
    cmaterial->SetAdhesion(0);

    // Create a container -- always FIRST body in the system
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetPos(ChVector3d(0., 0., 0.));
    ground->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    chrono::utils::AddBoxContainer(ground, cmaterial,                                 //
                                   ChFrame<>(ChVector3d(0., 0., 0.), QUNIT),          //
                                   ChVector3d(1.2 * bxDim, 1.2 * byDim, bzDim), 0.1,  //
                                   ChVector3i(2, 2, 2),                               //
                                   false);
    ground->EnableCollision(true);

    // Add BCE particles attached on the walls into FSI system
    sysFSI.AddBoxContainerBCE(ground,                                       //
                              ChFrame<>(ChVector3d(0., 0., 0.), QUNIT),     //
                              ChVector3d(1.2 * bxDim, 1.2 * byDim, bzDim),  //
                              ChVector3i(2, 0, -1));

    // Create the wheel -- always SECOND body in the system
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    trimesh->Transform(ChVector3d(0, 0, 0),
                       ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);         // if meshes are not watertight

    // Compute mass inertia from mesh
    double mmass;
    double mdensity = 1500.;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);
    mcog = ChVector3d(0.0, 0.0, 0.0);
    std::cout << "This is unused but... " << std::endl;
    std::cout << "Wheel Mass: " << mmass << std::endl;
    std::cout << "Wheel Density: " << mdensity << std::endl;
    std::cout << "Inertia Matrix: " << std::endl << minertia << std::endl;

    // Set the abs orientation, position and velocity
    auto wheel = chrono_types::make_shared<ChBodyAuxRef>();
    ChQuaternion<> wheel_Rot = QUNIT;

    // Set the COG coordinates to barycenter, without displacing the REF
    // reference. Make the COG frame a principal frame.
    wheel->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    wheel->SetMass(total_mass * 1.0 / 3.0);
    wheel->SetInertiaXX(mdensity * principal_I);
    wheel->SetPosDt(wheel_IniVel);
    wheel->SetAngVelLocal(ChVector3d(0.0, 0.0, 0.0));  // set an initial anular velocity (rad/s)

    // wheel material
    auto cmaterial2 = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial2->SetYoungModulus(1e9);
    cmaterial2->SetFriction(0.9f);
    cmaterial2->SetRestitution(0.4f);
    cmaterial2->SetAdhesion(0);
    // Set the absolute position of the body:
    wheel->SetFrameRefToAbs(ChFrame<>(ChVector3d(wheel_IniPos), ChQuaternion<>(wheel_Rot)));
    wheel->SetFixed(false);
    auto wheel_shape =
        chrono_types::make_shared<ChCollisionShapeTriangleMesh>(cmaterial2, trimesh, false, false, 0.005);
    wheel->AddCollisionShape(wheel_shape);
    wheel->EnableCollision(false);
    if (render_wheel) {
        auto wheel_visual_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        wheel_visual_shape->SetMesh(trimesh);
        wheel->AddVisualShape(wheel_visual_shape);
    }

    sysMBS.AddBody(wheel);

    // Use custom function by Wei
    double inner_radius = wheel_radius;
    double outer_radius = wheel_radius + grouser_height;
    sysFSI.AddWheelBCE_Grouser(wheel, ChFrame<>(), inner_radius, wheel_wide - iniSpacing, grouser_height, grouser_wide,
                               grouser_num, kernelLength, false);
    sysFSI.AddFsiBody(wheel);

    // Create the chassis -- always THIRD body in the system
    auto chassis = chrono_types::make_shared<ChBody>();
    chassis->SetMass(total_mass * 1.0 / 3.0);
    chassis->SetPos(wheel->GetPos());
    chassis->EnableCollision(false);
    chassis->SetFixed(false);

    sysMBS.AddBody(chassis);

    // Create the axle -- always FOURTH body in the system
    auto axle = chrono_types::make_shared<ChBody>();
    axle->SetMass(total_mass * 1.0 / 3.0);
    axle->SetPos(wheel->GetPos());
    axle->EnableCollision(false);
    axle->SetFixed(false);

    // Add geometry of the axle.
    // chrono::utils::AddSphereGeometry(axle.get(), cmaterial, 0.5,
    //                                  ChVector3d(0, 0, 0));
    sysMBS.AddBody(axle);

    // Connect the chassis to the containing bin (ground) through a translational
    // joint and create a linear actuator.
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(ground, chassis, ChFrame<>(chassis->GetPos(), QuatFromAngleY(CH_PI_2)));
    prismatic1->SetName("prismatic_chassis_ground");
    sysMBS.AddLink(prismatic1);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(chassis, axle, ChFrame<>(chassis->GetPos(), QUNIT));
    prismatic2->SetName("prismatic_axle_chassis");
    sysMBS.AddLink(prismatic2);

    // Connect the wheel to the axle through a engine joint.
    motor->SetName("engine_wheel_axle");
    motor->Initialize(wheel, axle, ChFrame<>(wheel->GetPos(), chrono::QuatFromAngleX(-CH_PI_2)));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, wheel_AngVel));
    sysMBS.AddLink(motor);
}

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& sim_number_str,
                     double& total_mass,
                     double& slope_angle,
                     double& wheel_AngVel,
                     double& grav_mag,
                     double& grouser_height,
                     int& neighbor_search,
                     bool& verbose,
                     bool& output,
                     bool& render) {
    ChCLI cli(argv[0], "FSI Single Wheel Acceleration Benchmark");

    cli.AddOption<std::string>("Simulation", "s,sim_number", "Simulation number", "0");
    cli.AddOption<double>("Simulation", "m,mass", "Total mass", "17.5");
    cli.AddOption<double>("Simulation", "a,angle", "Slope angle (degrees)", "0.0");
    cli.AddOption<double>("Simulation", "w,angular_vel", "Wheel angular velocity", "0.0");
    cli.AddOption<double>("Simulation", "g,gravity", "Gravity magnitude", "9.81");
    cli.AddOption<double>("Simulation", "grouser_height", "Grouser height", "0.01");
    cli.AddOption<int>("Simulation", "n,neighbor_search", "Neighbor search steps", "1");

    cli.AddOption<bool>("Output", "quiet", "Disable verbose terminal output", "false");
    cli.AddOption<bool>("Output", "o,output", "Enable output", "true");
    cli.AddOption<bool>("Visualization", "r,render", "Enable run-time visualization", "false");

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    sim_number_str = cli.GetAsType<std::string>("sim_number");
    total_mass = cli.GetAsType<double>("mass");
    slope_angle = cli.GetAsType<double>("angle");
    wheel_AngVel = cli.GetAsType<double>("angular_vel");
    grav_mag = cli.GetAsType<double>("gravity");
    grouser_height = cli.GetAsType<double>("grouser_height");
    neighbor_search = cli.GetAsType<int>("neighbor_search");

    verbose = !cli.GetAsType<bool>("quiet");
    output = cli.GetAsType<bool>("output");
    render = cli.GetAsType<bool>("render");

    return true;
}

int main(int argc, char* argv[]) {
    // Create the MBS and FSI systems
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Parse command-line arguments
    std::string sim_number_str;
    double total_mass;
    double slope_angle;
    double wheel_AngVel;
    double grav_mag;
    double grouser_height;
    double art_visc_al;
    int neighbor_search;
    bool verbose;
    bool output;
    bool render;

    if (!GetProblemSpecs(argc, argv, sim_number_str, total_mass, slope_angle, wheel_AngVel, grav_mag, grouser_height,
                         neighbor_search, verbose, output, render)) {
        return 1;
    }

    sysFSI.SetVerbose(verbose);

    std::cout << "Sim Number: " << sim_number_str << std::endl;
    std::cout << "Total Mass: " << total_mass << std::endl;
    std::cout << "Slope Angle: " << slope_angle << std::endl;
    std::cout << "Wheel Angular Velocity: " << wheel_AngVel << std::endl;
    std::cout << "Gravity Magnitude: " << grav_mag << std::endl;
    std::cout << "Grouser Height: " << grouser_height << std::endl;
    slope_angle = slope_angle / 180.0 * CH_PI;

    std::cout << "verbose: " << verbose << std::endl;
    out_dir = out_dir + std::to_string(neighbor_search) + "_0" + "/";

    if (output) {
        // Create oputput directories
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    out_dir = out_dir + sim_number_str + "/";

    // Output the result to verify
    std::cout << "Output directory: " << out_dir << std::endl;

    if (output) {
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
        if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
            std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
            return 1;
        }
    }

    sysFSI.ReadParametersFromFile(GetChronoDataFile("fsi/input_json/demo_FSI_SingleWheelTest_Granular.json"));
    sysFSI.SetNumProximitySearchSteps(neighbor_search);
    sysFSI.SetActiveDomainDelay(1.0);

    double gravity_G = -grav_mag;
    ChVector3d gravity = ChVector3d(gravity_G * sin(slope_angle), 0, gravity_G * cos(slope_angle));
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    double iniSpacing = sysFSI.GetInitialSpacing();
    double dT = sysFSI.GetStepSize();
    double kernelLength = sysFSI.GetKernelLength();

    // Initial Position of wheel
    ChVector3d wheel_IniPos(-bxDim / 2 + wheel_radius * 2.0, 0.0, wheel_radius + 10 * iniSpacing);
    ChVector3d wheel_IniVel(0.0, 0.0, 0.0);

    // Set the terrain container size
    sysFSI.SetContainerDim(ChVector3d(1.2 * bxDim, 1.2 * byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    // This is where that G matrix is set
    sysFSI.SetConsistentDerivativeDiscretization(false, false);

    // Set up the periodic boundary condition
    // Since we are using an active domain, the periodic domain can be exactly the same as the container
    ChVector3d cMin(-(1.2 * bxDim) / 2 - 20 * iniSpacing, -(1.2 * byDim) / 2 - 20 * iniSpacing,
                    -bzDim - 30 * iniSpacing);
    ChVector3d cMax((1.2 * bxDim) / 2 + 20 * iniSpacing, (1.2 * byDim) / 2 + 20 * iniSpacing, bzDim + 30 * iniSpacing);
    sysFSI.SetBoundaries(cMin, cMax);

    // Initialize the SPH particles
    chrono::utils::ChGridSampler<> sampler(iniSpacing);
    ChVector3d boxCenter(0.0, 0.0, 0.0);
    ChVector3d boxHalfDim(bxDim / 2 - iniSpacing, byDim / 2 - iniSpacing, bzDim / 2 - iniSpacing);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = (int)points.size();
    double gz = gravity_G;
    for (int i = 0; i < numPart; i++) {
        double pre_ini = sysFSI.GetDensity() * gz * (-points[i].z() + bzDim);
        double rho_ini = sysFSI.GetDensity() + pre_ini / (sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed());
        sysFSI.AddSPHParticle(points[i], rho_ini, pre_ini, sysFSI.GetViscosity(), ChVector3i(0.0, 0, 0));
    }

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(sysMBS, sysFSI, wheel_IniPos, wheel_IniVel, wheel_AngVel, render, wheel_radius, grouser_height,
                     wheel_width, grouser_wide, grouser_num, kernelLength, total_mass, iniSpacing);

    // Set simulation data output length
    sysFSI.SetOutputLength(0);

    // Construction of the FSI system must be finalized before running
    sysFSI.Initialize();

    auto wheel = sysMBS.GetBodies()[1];
    auto reaction = actuator->GetReaction2();
    ChVector3d force = reaction.force;
    ChVector3d torque = motor->GetReaction1().torque;
    ChVector3d w_pos = wheel->GetPos();
    ChVector3d w_vel = wheel->GetPosDt();
    ChVector3d angvel = wheel->GetAngVelLocal();

    // Save wheel mesh
    ChTriangleMeshConnected wheel_mesh;
    wheel_mesh.LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    wheel_mesh.RepairDuplicateVertexes(1e-9);

    // Write the information into a txt file
    std::ofstream myFile;
    std::ofstream myDBP_Torque;
    if (output) {
        myFile.open(out_dir + "/results.txt", std::ios::trunc);
        myDBP_Torque.open(out_dir + "/DBP_Torque.txt", std::ios::trunc);
    }

    // Create a run-tme visualizer

#ifdef CHRONO_VSG
    ChVisualSystem::Type vis_type;
    // Set up real-time visualization of the FSI system
    vis_type = ChVisualSystem::Type::VSG;
    std::shared_ptr<ChFsiVisualization> visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
    if (render) {
        visFSI->SetTitle("Chrono::CRM single wheel test");
        visFSI->SetSize(720, 720);
        visFSI->AddCamera(ChVector3d(-bxDim / 2. + 1, -5 * byDim, 5 * bzDim), ChVector3d(-bxDim / 2. + 1, 0., 0));
        visFSI->SetCameraMoveScale(0.1f);
        visFSI->EnableBoundaryMarkers(true);
        ;
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetImageOutputDirectory(out_dir + "images");
        visFSI->SetImageOutput(1);
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }
#endif

    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int print_steps = (unsigned int)round(1 / (print_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));

    double time = 0.0;
    int current_step = 0;

    // Some things that even DEM domes
    double x1 = wheel->GetPos().x();
    double z1 = x1 * std::sin(slope_angle);
    double x2, z2, z_adv = 0.;

    ChTimer timer;
    timer.start();
    while (time < total_time) {
        double adv = x2 - x1;
        if (current_step % print_steps == 0) {
            std::cout << "time: " << time << std::endl;
            std::cout << "  wheel position:         " << w_pos << std::endl;
            std::cout << "  wheel linear velocity:  " << w_vel << std::endl;
            // Compute slip
            // double linear_velocity = w_vel.x();
            double angular_velocity = angvel.z();
            // double slip = 1 - (std::abs(linear_velocity) / (std::abs(angular_velocity) * (wheel_radius +
            // grouser_height))); Compute slip like DEM
            double slip = 1 - (adv / (std::abs(angular_velocity) * (wheel_radius + grouser_height) * time));
            // Print slip
            std::cout << "Slip: " << slip << std::endl;

            std::cout << "  wheel angular velocity: " << angvel << std::endl;
            std::cout << "  drawbar pull:           " << force << std::endl;
            std::cout << "  wheel torque:           " << torque << std::endl;
        }

        if (z_adv >= z_adv_targ) {
            break;
        }

        if (output) {
            myFile << time << "\t" << w_pos.x() << "\t" << w_pos.y() << "\t" << w_pos.z() << "\t" << w_vel.x() << "\t"
                   << w_vel.y() << "\t" << w_vel.z() << "\t" << angvel.x() << "\t" << angvel.y() << "\t" << angvel.z()
                   << "\t" << force.x() << "\t" << force.y() << "\t" << force.z() << "\t" << torque.x() << "\t"
                   << torque.y() << "\t" << torque.z() << "\n";
            myDBP_Torque << time << "\t" << force.x() << "\t" << torque.z() << "\n";
        }

        if (output && current_step % output_steps == 0) {
            std::cout << "-------- Output" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
            static int counter = 0;
            std::string filename = out_dir + "/vtk/wheel." + std::to_string(counter++) + ".vtk";
            WriteWheelVTK(filename, wheel_mesh, wheel->GetFrameRefToAbs());
        }

        // Render SPH particles
#ifdef CHRONO_VSG
        if (render && current_step % render_steps == 0) {
            if (!visFSI->Render())
                break;
        }
#endif

        // Get the infomation of the wheel
        reaction = actuator->GetReaction2();
        force = reaction.force;
        torque = motor->GetReaction1().torque;
        w_pos = wheel->GetPos();
        w_vel = wheel->GetPosDt();
        angvel = wheel->GetAngVelLocal();

        x2 = w_pos.x();
        z_adv = x2 * std::sin(slope_angle) - z1;
        if (x2 > safe_x) {
            break;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();
        time += dT;
        current_step++;
    }
    timer.stop();
    std::cout << "Runtime: " << timer() << " seconds\n" << std::endl;
    std::cout << "Simulation time: " << time << std::endl;
    std::cout << "Simulation finished" << std::endl;

    if (output) {
        myFile.close();
        myDBP_Torque.close();
    }

    return 0;
}
