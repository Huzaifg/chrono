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
// Author: Huzaifa
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/core/ChTransform.h"

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_fsi/visualization/ChFsiVisualization.h"
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;

// -----------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "BAFFLE_FLOW/";

// Output frequency
bool output = true;
double out_fps = 20;

// Dimension of the space domain
double bxDim = 1.8;
double byDim = 1.8;
double bzDim = 0.8;

// Size of the baffles
double baffle_thickness = 0.15;
double baffle_height = 0.3;
double baffle_width = 0.15;

// Baffle position
double baffle1_x = 0.775;
double baffle1_y = 0.375;
double baffle1_z = 0;  // Base is on the ground

double baffle2_x = 0.775;
double baffle2_y = 1.625;
double baffle2_z = 0;  // Base is on the ground

double baffle3_x = 1.275;
double baffle3_y = 1.0;
double baffle3_z = 0;  // Base is on the ground

// SPH Particles Initial Position
double granular_x = 0.5;
double granular_y = 0.5;
double granular_z = 0.45;

// Final simulation time
double t_end = 1.0;

// Enable/disable run-time visualization
bool render = true;
float render_fps = 1000;

//------------------------------------------------------------------
// Create the objects of the MBD system. BCE markers added next to
// the boundaries of the fluid domain.
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    // Set gravity to the rigid body system in chrono
    sysMBS.Set_G_acc(sysFSI.Get_G_acc());

    // Set common material Properties
    auto cmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.2f);
    cmaterial->SetRestitution(0.05f);
    cmaterial->SetAdhesion(0);

    // Get particle spacing in the simulation
    auto initSpace0 = sysFSI.GetInitialSpacing();

    // Create a container
    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector<>(bxDim / 2., byDim / 2., bzDim / 2.));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetIdentifier(-1);
    box->SetBodyFixed(true);
    sysMBS.AddBody(box);

    // Add collision geometry for the container walls
    chrono::utils::AddBoxContainer(box, cmaterial,                                                    //
                                   ChFrame<>(ChVector<>(bxDim / 2., byDim / 2., bzDim / 2.), QUNIT),  //
                                   ChVector<>(bxDim, byDim, bzDim), 0.1,                              //
                                   ChVector<int>(2, 2, 2),                                            //
                                   false);
    box->SetCollide(true);

    // Add BCE particles attached on the walls into FSI system
    sysFSI.AddBoxContainerBCE(box,                                                               //
                              ChFrame<>(ChVector<>(bxDim / 2., byDim / 2., bzDim / 2.), QUNIT),  //
                              ChVector<>(bxDim, byDim, bzDim),                                   //
                              ChVector<int>(2, 2, 2));

    // Create the baffles as Boxes
    // Baffle 1
    auto baffle1 = chrono_types::make_shared<ChBody>();
    baffle1->SetPos(ChVector<>(baffle1_x, baffle1_y, baffle1_z + baffle_height / 2.));
    baffle1->SetRot(ChQuaternion<>(1, 0, 0, 0));
    baffle1->SetIdentifier(-2);
    baffle1->SetBodyFixed(true);
    sysMBS.AddBody(baffle1);

    // Add collision geometry for the baffle
    chrono::utils::AddBoxGeometry(baffle1.get(), cmaterial,
                                  ChVector<>(baffle_width / 2., baffle_thickness / 2., baffle_height / 2.),
                                  ChVector<>(baffle1_x, baffle1_y, baffle1_z + baffle_height / 2.), QUNIT);
    baffle1->SetCollide(true);

    // Add BCE particles attached on the baffle into FSI system
    sysFSI.AddBoxContainerBCE(baffle1,  //
                              ChFrame<>(ChVector<>(baffle1_x, baffle1_y, baffle1_z + baffle_height / 2.), QUNIT),
                              ChVector<>(baffle_width / 2., baffle_thickness / 2., baffle_height / 2.),
                              ChVector<int>(2, 2, 2));
    // Add as FSI Body
    // sysFSI.AddFsiBody(baffle1);

    // Baffle 2
    auto baffle2 = chrono_types::make_shared<ChBody>();
    baffle2->SetPos(ChVector<>(baffle2_x, baffle2_y, baffle2_z + baffle_height / 2.));
    baffle2->SetRot(ChQuaternion<>(1, 0, 0, 0));
    baffle2->SetIdentifier(-3);
    baffle2->SetBodyFixed(true);
    sysMBS.AddBody(baffle2);

    // Add collision geometry for the baffle
    chrono::utils::AddBoxGeometry(baffle2.get(), cmaterial,
                                  ChVector<>(baffle_thickness / 2., baffle_width / 2., baffle_height / 2.),
                                  ChVector<>(baffle2_x, baffle2_y, baffle2_z + baffle_height / 2.), QUNIT);
    baffle2->SetCollide(true);

    // Add BCE particles attached on the baffle into FSI system
    sysFSI.AddBoxContainerBCE(baffle2,  //
                              ChFrame<>(ChVector<>(baffle2_x, baffle2_y, baffle2_z + baffle_height / 2.), QUNIT),
                              ChVector<>(baffle_thickness / 2., baffle_width / 2., baffle_height / 2.),
                              ChVector<int>(2, 2, 2));
    // sysFSI.AddFsiBody(baffle2);

    // Baffle 3
    auto baffle3 = chrono_types::make_shared<ChBody>();
    baffle3->SetPos(ChVector<>(baffle3_x, baffle3_y, baffle3_z + baffle_height / 2.));
    baffle3->SetRot(ChQuaternion<>(1, 0, 0, 0));
    baffle3->SetIdentifier(-4);
    baffle3->SetBodyFixed(true);
    sysMBS.AddBody(baffle3);

    // Add collision geometry for the baffle
    chrono::utils::AddBoxGeometry(baffle3.get(), cmaterial,
                                  ChVector<>(baffle_thickness / 2., baffle_width / 2., baffle_height / 2.),
                                  ChVector<>(baffle3_x, baffle3_y, baffle3_z + baffle_height / 2.), QUNIT);
    baffle3->SetCollide(true);

    // Add BCE particles attached on the baffle into FSI system
    sysFSI.AddBoxContainerBCE(baffle3,  //
                              ChFrame<>(ChVector<>(baffle3_x, baffle3_y, baffle3_z + baffle_height / 2.), QUNIT),
                              ChVector<>(baffle_thickness / 2., baffle_width / 2., baffle_height / 2.),
                              ChVector<int>(2, 2, 2));
    // sysFSI.AddFsiBody(baffle3);
}

// =============================================================================
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
    if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
        std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
        return 1;
    }

    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // TODO : Check these JSON files and change them accordingly
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_CylinderDrop_Explicit.json");
    if (argc == 1) {
        std::cout << "Use the default JSON file" << std::endl;
    } else if (argc == 2) {
        std::cout << "Use the specified JSON file" << std::endl;
        std::string my_inputJson = std::string(argv[1]);
        inputJson = my_inputJson;
    } else {
        std::cout << "usage: ./demo_FSI_CylinderDrop <json_file>" << std::endl;
        return 1;
    }
    sysFSI.ReadParametersFromFile(inputJson);
    sysFSI.SetNumBoundaryLayers(3);

    // Set the periodic boundary condition (if not, set relative larger values)
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector<> cMin(-10, -10, -10);
    ChVector<> cMax(10, 10, 10);
    sysFSI.SetBoundaries(cMin, cMax);

    // Create an initial box for the terrain patch
    chrono::utils::GridSampler<> sampler(initSpace0);

    // Use a chrono sampler to create a bucket of granular material
    // TODO : Move these values as variables to the top
    ChVector<> boxCenter(granular_x, granular_y, granular_z);
    ChVector<> boxHalfDim(granular_x + 0.5, granular_y + 0.5, granular_z + 0.05);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles from the sampler points to the FSI system
    // TODO : Check if the number of particles is the same as that in the paper
    size_t numPart = (int)points.size();
    double gz = std::abs(sysFSI.Get_G_acc().z());
    for (int i = 0; i < numPart; i++) {
        double pre_ini = sysFSI.GetDensity() * gz * (-points[i].z() + bzDim);
        double rho_ini = sysFSI.GetDensity() + pre_ini / (sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed());
        sysFSI.AddSPHParticle(points[i], rho_ini, pre_ini, sysFSI.GetViscosity(), ChVector<>(0));
    }

    // Create MBD and BCE particles for the solid domain
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Set up integrator for the multi-body dynamics system
    sysMBS.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(sysMBS.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(1000);
    mystepper->SetAbsTolerances(1e-6);

    // Set up real-time visualization of the FSI system
    vis_type = ChVisualSystem::Type::VSG;
    std::shared_ptr<ChFsiVisualization> visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
    auto origin = sysMBS.Get_bodylist()[1]->GetPos();
    visFSI->SetTitle("Chrono::FSI Baffle Flow");
    visFSI->SetSize(1280, 720);
    visFSI->AddCamera(origin - ChVector<>(2 * bxDim, 2 * byDim, 0), origin);
    visFSI->SetCameraMoveScale(0.1f);
    visFSI->EnableBoundaryMarkers(true);
    visFSI->EnableRigidBodyMarkers(true);
    visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
    visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
    visFSI->SetSPHColorCallback(chrono_types::make_shared<HeightColorCallback>(0, 1.2));
    visFSI->AttachSystem(&sysMBS);
    visFSI->Initialize();

    // Start the simulation
    double dT = sysFSI.GetStepSize();
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));
    double time = 0.0;
    int current_step = 0;
    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && current_step % output_steps == 0) {
            std::cout << "-------- Output" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
        }

        // Render SPH particles
        if (render && current_step % render_steps == 0) {
            if (!visFSI->Render())
                break;
        }

        std::cout << "step: " << current_step << "\ttime: " << time << "\tRTF: " << sysFSI.GetRTF() << std::endl;

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();
        time += dT;
        current_step++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}