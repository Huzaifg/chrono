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
// Author: Wei Hu, Huzaifa Mustafa Unjhawala
// =============================================================================

// General Includes
#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono/utils/ChUtilsSamplers.h"

using namespace chrono;
using namespace chrono::fsi;

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Truths from Wei Paper -
// https://www.sciencedirect.com/science/article/pii/S0045782521003534?ref=pdf_download&fr=RR-2&rr=8c4472d7d99222ff

double bulk_density = 1500;
double mu_s = 0.3819;
double granular_particle_diameter = 0.002;  // Set in JSON - Not overwritten
double youngs_modulus = 2e6;                // Set in JSON - Not overwritten

// Global arguments
bool render = true;
double render_fps = 100;
std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Angle_Repose_Granular.json");

// Function to handle CLI arguments
bool GetProblemSpecs(int argc,
                     char** argv,
                     double& t_end,
                     bool& verbose,
                     bool& output,
                     double& output_fps,
                     bool& snapshots,
                     int& ps_freq,
                     double& cylinder_radius,
                     double& cylinder_height,
                     double& init_spacing) {
    ChCLI cli(argv[0], "FSI Angle of Repose Demo");

    cli.AddOption<double>("Simulation", "t_end", "End time", std::to_string(t_end));
    cli.AddOption<bool>("Simulation", "verbose", "Verbose output", std::to_string(verbose));
    cli.AddOption<bool>("Output", "output", "Enable output", std::to_string(output));
    cli.AddOption<double>("Output", "output_fps", "Output FPS", std::to_string(output_fps));
    cli.AddOption<bool>("Output", "snapshots", "Enable snapshots", std::to_string(snapshots));
    cli.AddOption<int>("Simulation", "ps_freq", "Proximity search frequency", std::to_string(ps_freq));
    cli.AddOption<double>("Geometry", "cylinder_radius", "Cylinder radius", std::to_string(cylinder_radius));
    cli.AddOption<double>("Geometry", "cylinder_height", "Cylinder height", std::to_string(cylinder_height));
    cli.AddOption<double>("Simulation", "init_spacing", "Initial particle spacing", std::to_string(init_spacing));

    if (!cli.Parse(argc, argv))
        return false;

    t_end = cli.GetAsType<double>("t_end");
    verbose = cli.GetAsType<bool>("verbose");
    output = cli.GetAsType<bool>("output");
    output_fps = cli.GetAsType<double>("output_fps");
    snapshots = cli.GetAsType<bool>("snapshots");
    ps_freq = cli.GetAsType<int>("ps_freq");
    cylinder_radius = cli.GetAsType<double>("cylinder_radius");
    cylinder_height = cli.GetAsType<double>("cylinder_height");
    init_spacing = cli.GetAsType<double>("init_spacing");

    return true;
}

int main(int argc, char* argv[]) {
    // Default values
    double t_end = 10.0;
    bool verbose = false;
    bool output = true;
    double output_fps = 20;
    bool snapshots = false;
    int ps_freq = 1;
    double cylinder_radius = 0.5;
    double cylinder_height = 1.0;
    double init_spacing = 0.01;

    // Parse command-line arguments
    if (!GetProblemSpecs(argc, argv, t_end, verbose, output, output_fps, snapshots, ps_freq, cylinder_radius,
                         cylinder_height, init_spacing)) {
        return 1;
    }

    ChSystemSMC sysMBS;
    ChFsiSystemSPH sysFSI(&sysMBS);

    sysFSI.SetVerbose(verbose);
    sysFSI.ReadParametersFromFile(inputJson);
    sysFSI.SetNumProximitySearchSteps(ps_freq);

    // Modify based on command line
    sysFSI.SetInitialSpacing(init_spacing);

    // Set SPH kernel length.
    sysFSI.SetKernelLength(1.2 * init_spacing);

    // Set density
    sysFSI.SetDensity(bulk_density);

    // Dimension of the space domain
    double bxDim = 10 * cylinder_radius;
    double byDim = 10 * cylinder_radius;
    double bzDim = 1.5 * cylinder_height;  // Higher than the cylinder to allow forparticle settling

    // Set the periodic boundary condition
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector3d cMin(-bxDim / 2 - 10.0 * initSpace0 / 2.0, -byDim / 2 - 1.0 * initSpace0 / 2.0,
                    -1.0 * bzDim - 5 * initSpace0);
    ChVector3d cMax(bxDim / 2 + 10.0 * initSpace0 / 2.0, byDim / 2 + 1.0 * initSpace0 / 2.0,
                    2.0 * bzDim + 5 * initSpace0);
    sysFSI.SetBoundaries(cMin, cMax);

    // Create SPH particle locations using a sampler
    chrono::utils::ChGridSampler<> sampler(init_spacing);
    ChVector3d cylinderCenter(0.0, 0.0, -bzDim / 2 + cylinder_height / 2 + 2 * init_spacing);
    std::vector<ChVector3d> points = sampler.SampleCylinderZ(cylinderCenter, cylinder_radius, cylinder_height / 2);

    // Add fluid particles
    double gz = std::abs(sysFSI.GetGravitationalAcceleration().z());
    for (const auto& p : points) {
        double pre_ini = sysFSI.GetDensity() * gz * (-p.z() + bzDim);
        double rho_ini = sysFSI.GetDensity() + pre_ini / (sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed());
        sysFSI.AddSPHParticle(p, rho_ini, pre_ini, sysFSI.GetViscosity(), ChVector3d(0));
    }

    // Add a box
    // Set common material Properties
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.389f);
    cmaterial->SetRestitution(0.05f);
    cmaterial->SetAdhesion(0);

    // Create a container
    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetFixed(true);
    sysMBS.AddBody(box);

    // Add collision geometry for the container walls
    chrono::utils::AddBoxContainer(box, cmaterial,                         //
                                   ChFrame<>(ChVector3d(0, 0, 0), QUNIT),  //
                                   ChVector3d(bxDim, byDim, bzDim), 0.1,   //
                                   ChVector3i(0, 0, -1),                   //
                                   false);
    box->EnableCollision(false);

    // Add BCE particles attached on the walls into FSI system
    sysFSI.AddBoxContainerBCE(box,                                    //
                              ChFrame<>(ChVector3d(0, 0, 0), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),        //
                              ChVector3i(0, 0, -1));

    sysFSI.Initialize();

    // Set up output directories
    std::stringstream ss;
    ss << "FSI_Angle_Repose_ps" << ps_freq << "_r" << std::fixed << std::setprecision(3) << std::setfill('0');

    // Create temporary strings and modify them
    std::string radius_str = std::to_string(cylinder_radius);
    std::replace(radius_str.begin(), radius_str.end(), '.', '_');
    ss << radius_str << "_h";

    std::string height_str = std::to_string(cylinder_height);
    std::replace(height_str.begin(), height_str.end(), '.', '_');
    ss << std::fixed << std::setprecision(3) << std::setfill('0') << height_str << "_s";

    std::string spacing_str = std::to_string(init_spacing);
    std::replace(spacing_str.begin(), spacing_str.end(), '.', '_');
    ss << std::fixed << std::setprecision(3) << std::setfill('0') << spacing_str;

    std::string out_dir = GetChronoOutputPath() + ss.str();

    if (output || snapshots) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        if (output) {
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

        if (snapshots) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
                std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
                return 1;
            }
        }
    }
    // Create a run-tme visualizer
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

        if (visFSI) {
            visFSI->SetTitle("FSI Angle of Repose Demo");
            visFSI->SetSize(1280, 720);
            visFSI->AddCamera(ChVector3d(0, -3 * byDim, bzDim), ChVector3d(0, 0, 0));
            visFSI->SetCameraMoveScale(0.1f);
            visFSI->EnableFluidMarkers(true);
            visFSI->EnableBoundaryMarkers(true);
            visFSI->EnableRigidBodyMarkers(false);
            visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
            visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
            visFSI->AttachSystem(&sysMBS);
            visFSI->Initialize();
        }
    }

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
            out_frame++;
        }

        // Render SPH particles
        if (render && time >= render_frame / render_fps) {
            if (!visFSI->Render())
                break;

            if (snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        if (sim_frame % 1000 == 0) {
            std::cout << "step: " << sim_frame << "\ttime: " << time << "\tRTF: " << sysFSI.GetRTF() << std::endl;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();

        time += sysFSI.GetStepSize();
        sim_frame++;
    }
    timer.stop();
    std::cout << "End Time: " << t_end << std::endl;
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}
