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

using namespace chrono;
using namespace chrono::fsi;

double render_fps = 100;

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Truths from Wei Paper -
// https://www.sciencedirect.com/science/article/pii/S0045782521003534?ref=pdf_download&fr=RR-2&rr=8c4472d7d99222ff
// Global parameters
const double sphere_radius = 0.0125;

// Function to handle CLI arguments
bool GetProblemSpecs(int argc,
                     char** argv,
                     double& t_end,
                     bool& verbose,
                     bool& output,
                     double& output_fps,
                     bool& snapshots,
                     int& ps_freq,
                     double& init_spacing,
                     double& sphere_density,
                     double& Hdrop,
                     bool& render) {
    ChCLI cli(argv[0], "FSI Sphere Drop Demo");

    cli.AddOption<double>("Simulation", "t_end", "End time", std::to_string(t_end));
    cli.AddOption<bool>("Simulation", "verbose", "Verbose output", std::to_string(verbose));
    cli.AddOption<bool>("Output", "output", "Enable output", std::to_string(output));
    cli.AddOption<double>("Output", "output_fps", "Output FPS", std::to_string(output_fps));
    cli.AddOption<bool>("Output", "snapshots", "Enable snapshots", std::to_string(snapshots));
    cli.AddOption<int>("Simulation", "ps_freq", "Proximity search frequency", std::to_string(ps_freq));
    cli.AddOption<double>("Simulation", "init_spacing", "Initial particle spacing", std::to_string(init_spacing));
    cli.AddOption<double>("Geometry", "sphere_density", "Sphere density", std::to_string(sphere_density));
    cli.AddOption<double>("Geometry", "Hdrop", "Drop height", std::to_string(Hdrop));
    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");
    if (!cli.Parse(argc, argv))
        return false;

    t_end = cli.GetAsType<double>("t_end");
    verbose = cli.GetAsType<bool>("verbose");
    output = cli.GetAsType<bool>("output");
    output_fps = cli.GetAsType<double>("output_fps");
    snapshots = cli.GetAsType<bool>("snapshots");
    ps_freq = cli.GetAsType<int>("ps_freq");
    init_spacing = cli.GetAsType<double>("init_spacing");
    sphere_density = cli.GetAsType<double>("sphere_density");
    Hdrop = cli.GetAsType<double>("Hdrop");
    render = !cli.GetAsType<bool>("no_vis");
    return true;
}

int main(int argc, char* argv[]) {
    // Default values
    double t_end = 2.0;
    bool verbose = true;
    bool output = true;
    double output_fps = 20;
    bool snapshots = false;
    int ps_freq = 1;
    double init_spacing = 0.01;
    double sphere_density = 700;
    double Hdrop = 0.5;
    bool render = true;

    // Parse command-line arguments
    if (!GetProblemSpecs(argc, argv, t_end, verbose, output, output_fps, snapshots, ps_freq, init_spacing,
                         sphere_density, Hdrop, render)) {
        return 1;
    }

    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    ChFsiSystemSPH sysFSI(&sysMBS);

    sysFSI.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    // Set gravity to the rigid body system in chrono
    sysMBS.SetGravitationalAcceleration(sysFSI.GetGravitationalAcceleration());

    // Get the pointer to the system parameter and use a JSON file to fill it out with the user parameters
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_SphereDrop_granular.json");
    sysFSI.ReadParametersFromFile(inputJson);

    // Modify based on command line
    sysFSI.SetVerbose(verbose);
    sysFSI.SetNumProximitySearchSteps(ps_freq);
    auto initSpace0 = sysFSI.GetInitialSpacing();
    // Dimension of the space domain
    double bxDim = 0.14;
    double byDim = 0.1;
    double fzDim = 0.15;
    double bzDim = fzDim;

    // Set the periodic boundary condition

    ChVector3d cMin(-bxDim / 2 * 1.2, -byDim / 2 * 1.2, -bzDim * 1.2);
    ChVector3d cMax(bxDim / 2 * 1.2, byDim / 2 * 1.2, (bzDim + Hdrop + sphere_radius + initSpace0) * 1.2);
    sysFSI.SetBoundaries(cMin, cMax);

    // Create SPH particle locations using a regular grid sampler
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    ChVector3d boxCenter(0, 0, fzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2 - initSpace0, byDim / 2 - initSpace0, fzDim / 2 - initSpace0);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles to the FSI system
    double gz = std::abs(sysFSI.GetGravitationalAcceleration().z());
    for (const auto& p : points) {
        double pre_ini = sysFSI.GetDensity() * gz * (-p.z() + fzDim);
        double rho_ini = sysFSI.GetDensity() + pre_ini / (sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed());
        sysFSI.AddSPHParticle(p, rho_ini, pre_ini, sysFSI.GetViscosity(), ChVector3d(0));
    }

    // Create MBD and BCE particles for the solid domain
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.3f);
    cmaterial->SetRestitution(0.05f);
    cmaterial->SetAdhesion(0);

    // Create a container
    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetFixed(true);
    sysMBS.AddBody(box);

    // Add collision geometry for the container walls
    chrono::utils::AddBoxContainer(box, cmaterial,                                 //
                                   ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                                   ChVector3d(bxDim, byDim, bzDim), 0.1,           //
                                   ChVector3i(2, 2, -1),                           //
                                   false);
    box->EnableCollision(false);

    // Add BCE particles attached on the walls into FSI system
    sysFSI.AddBoxContainerBCE(box,                                            //
                              ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),                //
                              ChVector3i(2, 2, -1));

    // Create a falling sphere
    auto sphere = chrono_types::make_shared<ChBody>();
    double volume = ChSphere::GetVolume(sphere_radius);
    double mass = sphere_density * volume;
    auto inertia = mass * ChSphere::GetGyration(sphere_radius);

    double sphere_z_pos = Hdrop + fzDim + sphere_radius + 0.5 * initSpace0;
    ChVector3d sphere_pos = ChVector3d(0, 0, sphere_z_pos);
    // ChVector3d sphere_vel = ChVector3d(0.0, 0.0, -2);
    sphere->SetPos(sphere_pos);
    // sphere->SetPosDt(sphere_vel);
    sphere->SetMass(mass);
    sphere->SetInertia(inertia);

    chrono::utils::AddSphereGeometry(sphere.get(), cmaterial, sphere_radius);
    sphere->GetCollisionModel()->SetSafeMargin(initSpace0);

    sysMBS.AddBody(sphere);
    sysFSI.AddFsiBody(sphere);
    sysFSI.AddSphereBCE(sphere, ChFrame<>(VNULL, QUNIT), sphere_radius, true, true);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Set up output directories
    std::stringstream ss;
    ss << "FSI_SphereDrop_ps" << ps_freq << "_d";

    std::string density_str = std::to_string(sphere_density);
    std::replace(density_str.begin(), density_str.end(), '.', '_');
    ss << density_str << "_h";

    std::string hdrop_str = std::to_string(Hdrop);
    std::replace(hdrop_str.begin(), hdrop_str.end(), '.', '_');
    ss << hdrop_str << "_s";

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

    // Create a run-time visualizer
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
            visFSI->SetTitle("FSI Sphere Drop Demo");
            visFSI->SetSize(1280, 720);
            visFSI->AddCamera(ChVector3d(0, -7 * byDim, bzDim), ChVector3d(0, 0, 0));
            visFSI->SetCameraMoveScale(0.1f);
            visFSI->EnableFluidMarkers(true);
            visFSI->EnableBoundaryMarkers(true);
            visFSI->EnableRigidBodyMarkers(true);
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

    std::string out_file = out_dir + "/sphere_penetration_depth.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
            out_frame++;
        }

        if (render && time >= render_frame / render_fps) {
            if (!visFSI->Render())
                break;

            if (snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/" << std::setw(5) << std::setfill('0') << render_frame << ".jpg";
                visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        // Advance simulation for one timestep for all systems
        sysFSI.DoStepDynamics_FSI();
        time += sysFSI.GetStepSize();

        // Write penetration depth to file
        double d_pen = fzDim + sphere_radius + 0.5 * initSpace0 - sphere->GetPos().z();
        ofile << time << " " << d_pen << " " << sphere->GetPos().x() << " " << sphere->GetPos().y() << " "
              << sphere->GetPos().z() << " " << sphere->GetPosDt().x() << " " << sphere->GetPosDt().y() << " "
              << sphere->GetPosDt().z() << std::endl;

        sim_frame++;
    }
    timer.stop();
    std::cout << "End Time: " << t_end << std::endl;
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}
