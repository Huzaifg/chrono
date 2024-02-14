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
#include "chrono_thirdparty/filesystem/resolver.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include <random>

using namespace chrono;
using namespace chrono::fsi;
using namespace rapidjson;

// -----------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
#ifdef CHRONO_VSG
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;
#endif

// Variables that are not randomized
// Output frequency
bool output = true;
double out_fps = 20;

// Dimension of the space domain
double bxDim = 0.8;
double byDim = 0.8;
double bzDim = 0.8;

// Size of the baffles
double baffle_thickness = 0.1;  // Dimension in the x direction
double baffle_height = 0.3;     // Dimension in the z direction
double baffle_width = 0.1;      // Dimension in the y direction

// Final simulation time
double t_end = 350 * 0.0025;

// Enable/disable run-time visualization
bool render = false;
float render_fps = 1000;

// Marker viz
bool enableBoundaryMarkers = false;
bool enableRigidBodyMarkers = true;

// ----------------------------------------------------------------------------
// Callback class for particle color based on x-displacement
// ----------------------------------------------------------------------------
class ChApi DisplacementColorCallback : public ChParticleCloud::ColorCallback {
  public:
    DisplacementColorCallback(double xmin, double xmax, const ChVector<>& up = ChVector<>(1, 0, 0))
        : m_monochrome(false), m_xmin(xmin), m_xmax(xmax), m_up(up) {}
    DisplacementColorCallback(const ChColor& base_color,
                              double xmin,
                              double xmax,
                              const ChVector<>& up = ChVector<>(1, 0, 0))
        : m_monochrome(true), m_base_color(base_color), m_xmin(xmin), m_xmax(xmax), m_up(up) {}

    virtual ChColor get(unsigned int n, const ChParticleCloud& cloud) const override {
        double displacement = Vdot(cloud.GetParticlePos(n), m_up);  // particle height
        if (m_monochrome) {
            float factor = (float)((displacement - m_xmin) / (m_xmax - m_xmin));  // color scaling factor (0,1)
            return ChColor(factor * m_base_color.R, factor * m_base_color.G, factor * m_base_color.B);
        } else
            return ChColor::ComputeFalseColor(displacement, m_xmin, m_xmax);
    }

  private:
    bool m_monochrome;
    ChColor m_base_color;
    double m_xmin;
    double m_xmax;
    ChVector<> m_up;
};

// ----------------------------------------------------------------------------
// Struct for storing the ranges of random parameters obtained from a JSON file
// ----------------------------------------------------------------------------

struct RandomParams {
    RandomParams() : sim_id(0) {}

    int sim_id;  // Simulation ID

    // Granular Material
    int no_granular_piles[2];   // Range of number of granular piles to sample from
    double pile_size_range[2];  // Range of dimensions along x,y and z of the granular pile
    double granular_x[2];       // Range of starting x coordinate of the granular pile
    double granular_y[2];       // Range of starting y coordinate of the granular pile
    double granular_z[2];       // Range of starting z coordinate of the granular pile
    double pile_velx_range[2];  // Range of x component of the velocity of the granular pile
    double pile_vely_range[2];  // Range of y component of the velocity of the granular pile
    double pile_velz_range[2];  // Range of z component of the velocity of the granular pile

    // Baffles
    double no_obstacles[2];  // Range of number of obstacles

    double baffle_x[2];  // Range of x coordinate of the baffle
    double baffle_y[2];  // Range of y coordinate of the baffle
    double baffle_z[2];  // Range of z coordinate of the baffle

    // Once the granualr particles are initialized, the following variables are set in order to ensure the baffles are
    // randomly placed at a sufficient distance
    std::vector<ChVector<double>> granular_pile_start;  // Vector of granular pile start coordinates
    std::vector<ChVector<double>> granular_pile_size;   // Vector of granular pile dimensions
    int sampled_granular_piles;                         // Number of granular piles sampled
};

// ----------------------------------------------------------------------------
// Read range of randomized variables from JSON file
// ----------------------------------------------------------------------------
void readRandomRanges(RandomParams& ranges, const std::string& ranges_file) {
    std::cout << "Reading parameters from: " << ranges_file << std::endl;

    FILE* fp = fopen(ranges_file.c_str(), "r");
    if (!fp) {
        std::cerr << "Invalid JSON file!" << std::endl;
        return;
    }

    char readBuffer[32768];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    fclose(fp);

    Document doc;

    doc.ParseStream<ParseFlag::kParseCommentsFlag>(is);
    if (!doc.IsObject()) {
        std::cerr << "Invalid JSON file!!" << std::endl;
        return;
    }

    if (doc.HasMember("sim_id")) {
        ranges.sim_id = doc["sim_id"].GetInt();
    }

    if (doc.HasMember("no_granular_piles")) {
        ranges.no_granular_piles[0] = doc["no_granular_piles"][0].GetInt();
        ranges.no_granular_piles[1] = doc["no_granular_piles"][1].GetInt();
    }

    if (doc.HasMember("pile_size_range")) {
        ranges.pile_size_range[0] = doc["pile_size_range"][0].GetDouble();
        ranges.pile_size_range[1] = doc["pile_size_range"][1].GetDouble();
    }

    if (doc.HasMember("pile_start_range")) {
        ranges.granular_x[0] = doc["pile_start_range"][0][0u].GetDouble();
        ranges.granular_x[1] = doc["pile_start_range"][0][1u].GetDouble();

        ranges.granular_y[0] = doc["pile_start_range"][1][0u].GetDouble();
        ranges.granular_y[1] = doc["pile_start_range"][1][1u].GetDouble();

        ranges.granular_z[0] = doc["pile_start_range"][2][0u].GetDouble();
        ranges.granular_z[1] = doc["pile_start_range"][2][1u].GetDouble();
    }

    if (doc.HasMember("pile_vel_range")) {
        ranges.pile_velx_range[0] = doc["pile_vel_range"][0][0u].GetDouble();
        ranges.pile_velx_range[1] = doc["pile_vel_range"][0][1u].GetDouble();

        ranges.pile_vely_range[0] = doc["pile_vel_range"][1][0u].GetDouble();
        ranges.pile_vely_range[1] = doc["pile_vel_range"][1][1u].GetDouble();

        ranges.pile_velz_range[0] = doc["pile_vel_range"][2][0u].GetDouble();
        ranges.pile_velz_range[1] = doc["pile_vel_range"][2][1u].GetDouble();
    }

    if (doc.HasMember("no_obstacles")) {
        ranges.no_obstacles[0] = doc["no_obstacles"][0].GetDouble();
        ranges.no_obstacles[1] = doc["no_obstacles"][1].GetDouble();
    }

    if (doc.HasMember("obstacle_cg_range")) {
        ranges.baffle_x[0] = doc["obstacle_cg_range"][0][0u].GetDouble();
        ranges.baffle_x[1] = doc["obstacle_cg_range"][0][1u].GetDouble();

        ranges.baffle_y[0] = doc["obstacle_cg_range"][1][0u].GetDouble();
        ranges.baffle_y[1] = doc["obstacle_cg_range"][1][1u].GetDouble();

        ranges.baffle_z[0] = doc["obstacle_cg_range"][2][0u].GetDouble();
        ranges.baffle_z[1] = doc["obstacle_cg_range"][2][1u].GetDouble();
    }
}

// ------------------------------------------
// Some utility functions for random numbers
// ------------------------------------------
int random_int(int min, int max) {
    std::random_device rd;                              // Seed for random number generator
    std::uniform_int_distribution<int> dist(min, max);  // Distribution over the range
    return dist(rd);
}

double random_double(double min, double max) {
    std::random_device rd;   // Seed random number generator
    std::mt19937 gen(rd());  // Use Mersenne Twister engine
    std::uniform_real_distribution<> dist(min, max);

    return dist(gen);
}

// ----------------------------------------------
// Check if the granular pile is within the box
// ----------------------------------------------
bool checkGranularPlacement(const double granular_start_x,
                            const double granular_start_y,
                            const double granular_start_z,
                            const double granular_thickness,
                            const double granular_width,
                            const double granular_height) {
    // Calculate end positions of granular pile
    double granular_end_x = granular_start_x + granular_thickness;
    double granular_end_y = granular_start_y + granular_width;
    double granular_end_z = granular_start_z + granular_height;

    // Calculate end positions of box
    double box_end_x = bxDim;
    double box_end_y = byDim;
    double box_end_z = bzDim;

    // Check if granular pile is completely within the box in all dimensions
    return (0. < granular_start_x && granular_end_x < box_end_x) &&
           (0. < granular_start_y && granular_end_y < box_end_y) &&
           (0. < granular_start_z && granular_end_z < box_end_z);
}
// -----------------------------------------------------------
// Check if the baffle is placed away from the granular piles
// Also checks if the baffles are placed away from each other
// -----------------------------------------------------------
bool checkBafflePlacement(const double baffle_x,
                          const double baffle_y,
                          const double baffle_z,
                          const RandomParams ranges,
                          const std::vector<std::shared_ptr<ChBody>>& baffles,
                          const int baffle_index) {
    // Calculate baffle extents (considering half the dimensions)
    double baffle_x_start = baffle_x - baffle_thickness / 2;
    double baffle_y_start = baffle_y - baffle_width / 2;
    double baffle_z_start = baffle_z - baffle_height / 2;

    double baffle_x_end = baffle_x + baffle_thickness / 2;
    double baffle_y_end = baffle_y + baffle_width / 2;
    double baffle_z_end = baffle_z + baffle_height / 2;

    // Loop over all the granular piles
    for (int i = 0; i < ranges.sampled_granular_piles; i++) {
        // Calculate granular pile start and end coordinates
        double gp_start_x = ranges.granular_pile_start[i].x();
        double gp_start_y = ranges.granular_pile_start[i].y();
        double gp_start_z = ranges.granular_pile_start[i].z();

        double gp_end_x = gp_start_x + ranges.granular_pile_size[i].x();
        double gp_end_y = gp_start_y + ranges.granular_pile_size[i].y();
        double gp_end_z = gp_start_z + ranges.granular_pile_size[i].z();

        // Check for overlap -> If there is any overlap, return false
        if (!((gp_end_x < baffle_x_start) ||  // Granular pile completely left of baffle
              (gp_start_x > baffle_x_end) ||  // Granular pile completely right of baffle
              (gp_end_y < baffle_y_start) ||  // Granular pile completely below baffle
              (gp_start_y > baffle_y_end) ||  // Granular pile completely above baffle
              (gp_end_z < baffle_z_start) ||  // Granular pile completely in front of baffle
              (gp_start_z > baffle_z_end))    // Granular pile completely behind baffle
        ) {
            return false;
        }
    }

    if (baffle_index == 0)
        return true;  // No baffles to check against (first baffle is always valid

    // Loop over all the other baffles to ensure they are placed away from each other
    for (int i = 0; i < baffle_index; i++) {
        // Calculate baffle extents (considering half the dimensions)
        double other_baffle_x_start = baffles[i]->GetPos().x() - baffle_thickness / 2;
        double other_baffle_y_start = baffles[i]->GetPos().y() - baffle_width / 2;
        double other_baffle_z_start = baffles[i]->GetPos().z() - baffle_height / 2;

        double other_baffle_x_end = baffles[i]->GetPos().x() + baffle_thickness / 2;
        double other_baffle_y_end = baffles[i]->GetPos().y() + baffle_width / 2;
        double other_baffle_z_end = baffles[i]->GetPos().z() + baffle_height / 2;

        // Check for overlap -> If there is any overlap, return false
        if (!((other_baffle_x_end < baffle_x_start) ||  // Other baffle completely left of baffle
              (other_baffle_x_start > baffle_x_end) ||  // Other baffle completely right of baffle
              (other_baffle_y_end < baffle_y_start) ||  // Other baffle completely below baffle
              (other_baffle_y_start > baffle_y_end) ||  // Other baffle completely above baffle
              (other_baffle_z_end < baffle_z_start) ||  // Other baffle completely in front of baffle
              (other_baffle_z_start > baffle_z_end))    // Other baffle completely behind baffle
        ) {
            return false;
        }
    }
    // If we reach here, the baffle doesn't overlap any piles
    return true;
}

//------------------------------------------------------------------
// Create the objects of the MBD system. BCE markers added next to
// the boundaries of the fluid domain.
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI, const RandomParams& ranges) {
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
    chrono::utils::AddBoxContainer(box, cmaterial,                            //
                                   ChFrame<>(ChVector<>(0., 0., 0.), QUNIT),  //
                                   ChVector<>(bxDim, byDim, bzDim), 0.1,      //
                                   ChVector<int>(2, 2, 2),                    //
                                   false);
    box->SetCollide(true);

    // Add BCE particles attached on the walls into FSI system
    // Note: BCE marker frame is relative to body frame
    sysFSI.AddBoxContainerBCE(box,                                       //
                              ChFrame<>(ChVector<>(0., 0., 0.), QUNIT),  //
                              ChVector<>(bxDim, byDim, bzDim),           //
                              ChVector<int>(2, 2, 2));

    // Sample the number of baffles from ranges
    int numBaffles = random_int(ranges.no_obstacles[0], ranges.no_obstacles[1]);
    std::vector<std::shared_ptr<ChBody>> baffles(numBaffles);

    for (int i = 0; i < numBaffles; i++) {
        bool valid = false;
        double baffle_x, baffle_y, baffle_z;

        int attempt = 0;
        int max_attempts = 10000;
        while (!valid && attempt < max_attempts) {
            // Sample the cg position of the baffle
            baffle_x = random_double(ranges.baffle_x[0], ranges.baffle_x[1]);
            baffle_y = random_double(ranges.baffle_y[0], ranges.baffle_y[1]);
            baffle_z = random_double(ranges.baffle_z[0], ranges.baffle_z[1]);

            // Check if baffle placed away from granular material and each other
            valid = checkBafflePlacement(baffle_x, baffle_y, baffle_z, ranges, baffles, i);
            attempt++;
        }
        if (attempt == max_attempts) {
            std::cerr << "Max attempts reached for placing baffle " << i << std::endl;
        }
        // Setup the baffle
        auto baffle = chrono_types::make_shared<ChBody>();
        baffle->SetPos(ChVector<>(baffle_x, baffle_y, baffle_z));
        baffle->SetRot(ChQuaternion<>(1, 0, 0, 0));
        baffle->SetIdentifier(-2 - i);  // Assign a unique (negative) ID
        baffle->SetBodyFixed(true);

        // Add collision geometry for the baffle
        chrono::utils::AddBoxGeometry(baffle.get(), cmaterial,
                                      ChVector<>(baffle_thickness, baffle_width, baffle_height), ChVector<>(0., 0., 0.),
                                      QUNIT);
        baffle->SetCollide(true);

        // Store the baffle in the vector
        baffles[i] = baffle;
    }

    // Loop through the baffles and add to the systems with BCE markers
    for (auto baffle : baffles) {
        sysMBS.AddBody(baffle);
        sysFSI.AddBoxBCE(baffle,  //
                         ChFrame<>(ChVector<>(0., 0., 0.), QUNIT),
                         ChVector<>(baffle_thickness, baffle_width, baffle_height), true);
        sysFSI.AddFsiBody(baffle);
    }
}

// =============================================================================
int main(int argc, char* argv[]) {
    // Output directories and settings
    std::string out_dir;
    if (argc < 2) {
        out_dir = GetChronoOutputPath() + "BAFFLE_FLOW_TRAIN/";
    } else {
        out_dir = GetChronoOutputPath() + "BAFFLE_FLOW_TRAIN_" + argv[1] + "/";
    }

    // Create output directories
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

    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Baffle_flow_train.json");
    std::cout << "Use the default JSON file" << std::endl;
    
    sysFSI.ReadParametersFromFile(inputJson);
    sysFSI.SetNumBoundaryLayers(3);

    // Read the ranges of randomized parameters from the input JSON file
    RandomParams ranges;
    std::string ranges_file = GetChronoDataFile("fsi/input_json/demo_FSI_Baffle_flow_train_ranges.json");

    // Read the random ranges
    readRandomRanges(ranges, ranges_file);

    // Set the periodic boundary condition (if not, set relative larger values)
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector<> cMin(-10, -10, -10);
    ChVector<> cMax(10, 10, 10);
    sysFSI.SetBoundaries(cMin, cMax);

    // Create an initial box for the terrain patch
    chrono::utils::GridSampler<> sampler(initSpace0);

    // Sample number of granular material piles
    int numPiles = random_int(ranges.no_granular_piles[0], ranges.no_granular_piles[1]);
    ranges.sampled_granular_piles = numPiles;

    // Generate the granular piles and add them to the FSI system
    for (int i = 0; i < numPiles; i++) {
        bool valid = false;

        double granular_x, granular_y, granular_z;
        double granular_thickness, granular_width, granular_height;

        int attempt = 0;
        int max_attempts = 10000;  // Set a maximum number of attempts to avoid infinite loop
        while (!valid && attempt < max_attempts) {
            // Sample x,y and z starting positions of granular pile
            granular_x = random_double(ranges.granular_x[0], ranges.granular_x[1]);
            granular_y = random_double(ranges.granular_y[0], ranges.granular_y[1]);
            granular_z = random_double(ranges.granular_z[0], ranges.granular_z[1]);

            // Sample the dimensions of the granular pile
            granular_thickness = random_double(ranges.pile_size_range[0], ranges.pile_size_range[1]);
            // Cube shaped granular pile
            granular_height = granular_thickness;
            granular_width = granular_thickness;

            valid = checkGranularPlacement(granular_x, granular_y, granular_z, granular_thickness, granular_width,
                                           granular_height);
            attempt++;  // Increment attempt counter
        }
        if (attempt == max_attempts) {
            std::cerr << "Max attempts reached for placing granular pile" << std::endl;
        }

        // Store start position in ranges
        ranges.granular_pile_start.push_back(ChVector<double>(granular_x, granular_y, granular_z));

        // Store granular pile size in ranges
        ranges.granular_pile_size.push_back(ChVector<double>(granular_thickness, granular_width, granular_height));

        // Use a chrono sampler to create a bucket of granular material
        ChVector<> boxCenter(granular_x + 0.5 * granular_thickness, granular_y + 0.5 * granular_width,
                             granular_z + 0.5 * granular_height);
        ChVector<> boxHalfDim(0.5 * granular_thickness, 0.5 * granular_width, 0.5 * granular_height);
        std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);

        // Add SPH particles from the sampler points to the FSI system
        size_t numPart = (int)points.size();
        double gz = std::abs(sysFSI.Get_G_acc().z());
        for (int i = 0; i < numPart; i++) {
            double pre_ini = sysFSI.GetDensity() * gz * (-points[i].z() + bzDim);
            double rho_ini = sysFSI.GetDensity() + pre_ini / (sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed());

            // Sample the velocity of the granular pile
            double pile_velx = random_double(ranges.pile_velx_range[0], ranges.pile_velx_range[1]);
            double pile_vely = random_double(ranges.pile_vely_range[0], ranges.pile_vely_range[1]);
            double pile_velz = random_double(ranges.pile_velz_range[0], ranges.pile_velz_range[1]);

            sysFSI.AddSPHParticle(points[i], rho_ini, pre_ini, sysFSI.GetViscosity(),
                                  ChVector<>(pile_velx, pile_vely, pile_velz));
        }
    }

    // NOTE: CreateSolidPhase() must be called after the granular piles are initialized -> This is because the baffle
    // positioning is dependent on the granular piles
    // Create MBD and BCE particles for the solid domain
    CreateSolidPhase(sysMBS, sysFSI, ranges);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Set up integrator for the multi-body dynamics system
    sysMBS.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(sysMBS.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(1000);
    mystepper->SetAbsTolerances(1e-6);

    #ifdef CHRONO_VSG
    // Set up real-time visualization of the FSI system
    vis_type = ChVisualSystem::Type::VSG;
    std::shared_ptr<ChFsiVisualization> visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
    auto origin = sysMBS.Get_bodylist()[0]->GetPos();
    visFSI->SetTitle("Chrono::FSI Baffle Flow");
    visFSI->SetSize(2560, 1440);
    visFSI->AddCamera(ChVector<>(bxDim + 0.75, 0, bzDim + 0.5), ChVector<>(0.4, 0.4, 0.4));
    visFSI->SetCameraMoveScale(0.1f);
    visFSI->EnableBoundaryMarkers(enableBoundaryMarkers);
    visFSI->EnableRigidBodyMarkers(enableRigidBodyMarkers);
    visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
    visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
    visFSI->SetSPHColorCallback(chrono_types::make_shared<DisplacementColorCallback>(0, 0.8));
    visFSI->AttachSystem(&sysMBS);
    visFSI->Initialize();
    #endif

    // Start the simulation
    double dT = sysFSI.GetStepSize();
    unsigned int output_steps = 25;
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));
    double time = 0.0;
    int current_step = 0;
    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && current_step % output_steps == 0) {
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
        }

        #ifdef CHRONO_VSG
        // Render SPH particles
        if (render && current_step % render_steps == 0) {
            if (!visFSI->Render())
                break;
        }
        #endif

        if (current_step % 1000 == 0) {
            std::cout << "step: " << current_step << "\ttime: " << time << "\tRTF: " << sysFSI.GetRTF() << std::endl;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();
        time += dT;
        current_step++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}