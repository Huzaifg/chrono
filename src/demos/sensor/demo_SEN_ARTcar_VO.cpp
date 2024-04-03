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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// Main driver function for the ARTcar model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
// #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
#include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"

#include "chrono_vehicle/driver/ChDataDriver.h"
// #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/artcar/ARTcar.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
// #include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

using namespace chrono;
using namespace chrono::vsg3d;
using namespace chrono::vehicle;
using namespace chrono::sensor;

// =============================================================================

// Initial vehicle location and orientation
ChVector3d initLoc(2.5, 0., 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);


// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 0.2);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "ARTcar";
const std::string pov_dir = out_dir + "/POVRAY";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1;  // FPS = 1

// POV-Ray output
bool povray_output = false;

// =============================================================================

int main(int argc, char* argv[]) {

    // --------------
    // Create systems
    // --------------

    // Create the Sedan vehicle, set parameters, and initialize
    artcar::ARTcar car;
    car.SetContactMethod(contact_method);
    car.SetChassisCollisionType(chassis_collision_type);
    car.SetChassisFixed(false);
    ChQuaternion<> initRot;
    initRot.SetFromAngleAxis(CH_PI_2, ChVector3i(0, 0, 1));
    car.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    car.SetTireType(tire_model);
    car.SetTireStepSize(tire_step_size);
    car.SetTireRollingResistance(0.06);
    car.SetMaxMotorVoltageRatio(0.18);
    car.SetStallTorque(0.5);
    car.Initialize();

    VisualizationType tire_vis_type = VisualizationType::MESH;

    car.SetChassisVisualizationType(chassis_vis_type);
    car.SetSuspensionVisualizationType(suspension_vis_type);
    car.SetSteeringVisualizationType(steering_vis_type);
    car.SetWheelVisualizationType(wheel_vis_type);
    car.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(car.GetSystem());

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    terrain.Initialize();

    auto room_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    room_mesh->LoadWavefrontMesh(GetChronoDataFile("me3038/rm3038_v2.obj"), false, true);
    room_mesh->Transform(ChVector3i(0, 0, 0), ChMatrix33<>(1));
    auto room_mesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    room_mesh_shape->SetMesh(room_mesh);
    room_mesh_shape->SetName("me3038");
    room_mesh_shape->SetMutable(true);
    auto room_mesh_body = chrono_types::make_shared<ChBody>();
    // auto frame = chrono_types::make_shared<ChFrame>();
    room_mesh_body->AddVisualShape(room_mesh_shape);
    room_mesh_body->SetFixed(true);

    car.GetSystem()->AddBody(room_mesh_body);

    // Create the vehicle Irrlicht interface
    // auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    // vis->SetWindowTitle("ARTcar Demo");
    // vis->SetChaseCamera(trackPoint, 1.5, 0.05);
    // vis->Initialize();
    // vis->AddLightDirectional();
    // vis->AddSkyBox();
    // vis->AddLogo();
    // vis->AttachVehicle(&car.GetVehicle());

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
    vis->SetWindowTitle("ARTcar Demo");
    vis->AttachVehicle(&car.GetVehicle());
    vis->SetChaseCamera(trackPoint, 1.5, 0.05);
    vis->SetWindowSize(ChVector2<int>(1200, 900));
    vis->SetWindowPosition(ChVector2<int>(100, 300));
    vis->SetUseSkyBox(true);
    vis->SetCameraAngleDeg(40);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->SetShadows(true);
    vis->Initialize();

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    std::string driver_file = out_dir + "/driver_inputs.txt";

    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system
    auto driver_vsg = chrono_types::make_shared<ChInteractiveDriverVSG>(*vis);
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver_vsg->SetSteeringDelta(render_step_size / steering_time);
    driver_vsg->SetThrottleDelta(render_step_size / throttle_time);
    driver_vsg->SetBrakingDelta(render_step_size / braking_time);
    // if (driver_mode == DriverMode::PLAYBACK) {
    //     driver_vsg->SetInputDataFile(driver_file);
    //     driver_vsg->SetInputMode(ChInteractiveDriverVSG::InputMode::DATAFILE);
    // }
    driver_vsg->Initialize();

    // ------------------
    // Add Two Camera Sensors
    // ------------------
    float intensity = 1.0;
    auto manager = chrono_types::make_shared<ChSensorManager>(car.GetSystem());
    manager->scene->AddPointLight({0, 0, 5}, {intensity, intensity, intensity}, 500);
    manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});

    // Parameters
    CameraLensModelType lens_model = CameraLensModelType::PINHOLE;

    // Update rate in Hz
    float update_rate = 15.f;

    // Image width and height
    unsigned int image_width = 1280;
    unsigned int image_height = 720;

    // Camera's horizontal field of view
    float fov = (float)CH_PI / 3.;

    // Lag (in seconds) between sensing and when data becomes accessible
    float lag = .05f;

    // Exposure (in seconds) of each image
    float exposure_time = 0.02f;

    int alias_factor = 2;

    bool use_gi = true;  // whether cameras should use global illumination

    // Camera 1  at 59 mm in -y, along x axis and z axis
    chrono::ChFrame<double> offset_pose1({2.1, -0.59, 0.5}, QuatFromAngleAxis(0, {1, 0, 0}));
    auto cam1 = chrono_types::make_shared<ChCameraSensor>(car.GetChassisBody(),  // body camera is attached to
                                                          update_rate,           // update rate in Hz
                                                          offset_pose1,          // offset pose
                                                          image_width,           // image width
                                                          image_height,          // image height
                                                          fov,                   // camera's horizontal field of view
                                                          alias_factor,          // super sampling factor
                                                          lens_model,            // lens model type
                                                          use_gi, 2.2);          // lag
    cam1->SetRadialLensParameters(ChVector3f(0.0, 0.0, 0.0));
    cam1->SetName("Camera Sensor");
    cam1->SetLag(lag);
    cam1->SetCollectionWindow(exposure_time);
    cam1->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "Global Illumination"));
    // Provides the host access to this RGBA8 buffer
    cam1->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    // add sensor to the manager
    manager->AddSensor(cam1);
    ChMatrix33<float> I1;
    I1 = cam1->GetCameraIntrinsicMatrix();

    // Camera 2 at 59 mm in -y, along x axis and z axis
    chrono::ChFrame<double> offset_pose2({2.1, 0.59, 0.5}, QuatFromAngleAxis(0, {1, 0, 0}));
    auto cam2 = chrono_types::make_shared<ChCameraSensor>(car.GetChassisBody(),  // body camera is attached to
                                                          update_rate,           // update rate in Hz
                                                          offset_pose2,          // offset pose
                                                          image_width,           // image width
                                                          image_height,          // image height
                                                          fov,                   // camera's horizontal field of view
                                                          alias_factor,          // super sampling factor
                                                          lens_model,            // lens model type
                                                          use_gi, 2.2);          // lag
    cam2->SetRadialLensParameters(ChVector3f(0.0, 0.0, 0.0));
    cam2->SetName("Camera Sensor");
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);
    cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "Global Illumination"));
    // Provides the host access to this RGBA8 buffer
    cam2->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    // add sensor to the manager
    manager->AddSensor(cam2);
    ChMatrix33<float> I2;
    I2 = cam2->GetCameraIntrinsicMatrix();

    // Print camera intrinsic matrix
    std::cout << "Camera 1 intrinsic matrix: " << I1 << std::endl;
    std::cout << "Camera 2 intrinsic matrix: " << I2 << std::endl;

    // add an accelerometer, gyroscope, and magnetometer
    chrono::ChFrame<double> imu_offset_pose({0.0, 0.0, 0.0}, QuatFromAngleAxis(0, {1, 0, 0}));
    ChVector3d gps_reference(-89.400, 43.070, 260.0);
    auto noise_none = chrono_types::make_shared<ChNoiseNone>();

    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(car.GetChassisBody(), 100.f, imu_offset_pose, noise_none);
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
    manager->AddSensor(acc);

    auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(car.GetChassisBody(), 100.f, imu_offset_pose, noise_none);
    gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
    manager->AddSensor(gyro);

    auto mag =
        chrono_types::make_shared<ChMagnetometerSensor>(car.GetChassisBody(), 100.f, imu_offset_pose, noise_none, gps_reference);
    mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
    manager->AddSensor(mag);

    // ================
    // Compute oritentation of IMU with respect to the camera
    // ================
    // IMU Rotation Matrix
    ChMatrix33<double> imu_rot_matrix = imu_offset_pose.GetRotMat();
    
    // Camera 1
    ChMatrix33<double> cam1_rot_matrix = offset_pose1.GetRotMat();
    
    // IMU to Camera 1 frame
    ChMatrix33<double> imu_to_cam1 = cam1_rot_matrix.transpose() * imu_rot_matrix;

    // Compute camera 1 frame to IMU frame
    ChMatrix33<double> cam1_to_imu = imu_to_cam1.transpose();
    
    // Camera 2
    ChMatrix33<double> cam2_rot_matrix = offset_pose2.GetRotMat();
    // IMU to Camera 2 frame
    ChMatrix33<double> imu_to_cam2 = cam2_rot_matrix.transpose() * imu_rot_matrix;
    // Compute camera 2 frame to IMU frame
    ChMatrix33<double> cam2_to_imu = imu_to_cam2.transpose();

    // Print IMU to Camera 1 and Camera 2 rotation matrices
    std::cout << "IMU to Camera Left rotation matrix: " << imu_to_cam1 << std::endl;
    std::cout << "Camera 1 to IMU rotation matrix: " << cam1_to_imu << std::endl;
    std::cout << "IMU to Camera Right rotation matrix: " << imu_to_cam2 << std::endl;
    std::cout << "Camera 2 to IMU rotation matrix: " << cam2_to_imu << std::endl;

    // Find transalation between IMU and Camera 1 and Camera 2
    std::cout << "IMU to Camera 1 transalation" << offset_pose1.GetPos() << std::endl;
    std::cout << "IMU to Camera 2 transalation" << offset_pose2.GetPos() << std::endl;
    

    // ---------------
    // Add depth camera
    // ---------------
    // chrono::ChFrame<double> offset_pose_depth({2.1, 0, 0.5}, SetFromAngleAxis(0, {1, 0, 0}));
    // auto depth = chrono_types::make_shared<ChDepthCamera>(car.GetChassisBody(),  // body camera is attached to
    //                                                       update_rate,           // update rate in Hz
    //                                                       offset_pose_depth,     // offset pose
    //                                                       image_width,           // image width
    //                                                       image_height,          // image height
    //                                                       fov,                   // camera's horizontal field of view
    //                                                       lens_model             // lens model
    // );
    // depth->SetName("Depth Camera");
    // depth->SetLag(lag);
    // depth->SetCollectionWindow(exposure_time);  // lag

    // depth->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "Depth"));

    // manager->AddSensor(depth);
    // ---------------
    // Simulation loop
    // ---------------

    if (debug_output) {
        car.LogHardpointLocations();
    }

    // output vehicle mass
    std::cout << "VEHICLE MASS: " << car.GetVehicle().GetMass() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    car.GetVehicle().EnableRealtime(true);
    while (vis->Run()) {
        double time = car.GetSystem()->GetChTime();

        // End simulation
        if (time >= t_end)
            break;

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output) {
                // Zero-pad frame numbers in file names for postprocessing
                std::ostringstream filename;
                filename << pov_dir << "/data_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".dat";
                utils::WriteVisualizationAssets(car.GetSystem(), filename.str());
            }

            render_frame++;
        }

        // Debug logging
        if (debug_output && step_number % debug_steps == 0) {
            car.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
        }

        // Get driver inputs
        DriverInputs driver_inputs = driver_vsg->GetInputs();

        // Update modules (process inputs from other modules)
        driver_vsg->Synchronize(time);
        terrain.Synchronize(time);
        car.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver_vsg->Advance(step_size);
        terrain.Advance(step_size);
        car.Advance(step_size);
        vis->Advance(step_size);
        manager->Update();

        // Increment frame number
        step_number++;
    }


    return 0;
}