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
// Authors: Huzaifa Unjhawala, Harry Zhang
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

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSTFHandler.h"
#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"
#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSIMUHandler.h"
#include "chrono_ros/handlers/ChROSTFHandler.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include <chrono>

using namespace chrono;
using namespace chrono::vsg3d;
using namespace chrono::ros;
using namespace chrono::vehicle;
using namespace chrono::sensor;

enum IMUNoiseModel {
    NORMAL_DRIFT,  // gaussian drifting noise with noncorrelated equal distributions
    IMU_NONE       // no noise added
};
// IMUNoiseModel imu_noise_type = NORMAL_DRIFT;
IMUNoiseModel imu_noise_type = IMU_NONE;

// IMU update rate in Hz
int imu_update_rate = 1000;

// IMU lag (in seconds) between sensing and when data becomes accessible
float imu_lag = 0;

// IMU collection time (in seconds) of each sample
float imu_collection_time = 0;
// =============================================================================

// Initial vehicle location and orientation
ChVector3d initLoc(-2.5, 2.5, 0.5);
// ChQuaternion<> initRot(1, 0, 0, 0);


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
bool render = true;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 25;  // FPS = 50

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
    // initRot = QuatFromAngleZ(CH_PI /2.);
    // Create the Sedan vehicle, set parameters, and initialize
    artcar::ARTcar car;
    car.SetContactMethod(contact_method);
    car.SetChassisCollisionType(chassis_collision_type);
    car.SetChassisFixed(false);
    ChQuaternion<> initRot;
    initRot.SetFromAngleAxis(CH_PI, ChVector3i(0, 0, 1));
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
    float update_rate = 10.f;

    // Image width and height
    unsigned int image_width = 720;
    unsigned int image_height = 720;

    // Camera's horizontal field of view
    float fov = (float)CH_PI / 6.;

    // Lag (in seconds) between sensing and when data becomes accessible
    float lag = .0f;

    // Exposure (in seconds) of each image
    float exposure_time = 0.02f;

    int alias_factor = 3;

    bool use_gi = true;  // whether cameras should use global illumination

    // Camera 1  at 59 mm in -y, along x axis and z axis
    chrono::ChFrame<double> offset_pose1({0.21, -0.025, 0.00}, QuatFromAngleAxis(0, {1, 0, 0}));
    auto cam1 = chrono_types::make_shared<ChCameraSensor>(car.GetChassisBody(),  // body camera is attached to
                                                          update_rate,           // update rate in Hz
                                                          offset_pose1,          // offset pose
                                                          image_width,           // image width
                                                          image_height,          // image height
                                                          fov,                   // camera's horizontal field of view
                                                          alias_factor,          // super sampling factor
                                                          lens_model,            // lens model type
                                                          use_gi);          // Use global illumination
    cam1->SetRadialLensParameters(ChVector3f(0.0, 0.0, 0.0));
    cam1->SetName("Camera Sensor");
    cam1->SetLag(lag);
    cam1->SetCollectionWindow(exposure_time);
    // cam1->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "right camera"));
    // Provides the host access to this RGBA8 buffer
    cam1->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    // add sensor to the manager
    manager->AddSensor(cam1);

    // Camera 2 at 59 mm in -y, along x axis and z axis
    chrono::ChFrame<double> offset_pose2({0.21, 0.025, 0.00}, QuatFromAngleAxis(0, {1, 0, 0}));
    auto cam2 = chrono_types::make_shared<ChCameraSensor>(car.GetChassisBody(),  // body camera is attached to
                                                          update_rate,           // update rate in Hz
                                                          offset_pose2,          // offset pose
                                                          image_width,           // image width
                                                          image_height,          // image height
                                                          fov,                   // camera's horizontal field of view
                                                          alias_factor,          // super sampling factor
                                                          lens_model,            // lens model type
                                                          use_gi); // Use global illumination
    cam2->SetRadialLensParameters(ChVector3f(0.0, 0.0, 0.0));
    cam2->SetName("Camera Sensor");
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);
    // cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "left camera"));
    // Provides the host access to this RGBA8 buffer
    cam2->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    // add sensor to the manager
    manager->AddSensor(cam2);

   // ---------------------------------------------
    // Create a IMU and add it to the sensor manager
    // ---------------------------------------------
    // Create the imu noise model
    std::shared_ptr<ChNoiseModel> acc_noise_model;
    std::shared_ptr<ChNoiseModel> gyro_noise_model;
    std::shared_ptr<ChNoiseModel> mag_noise_model;
    switch (imu_noise_type) {
        case NORMAL_DRIFT:
            // Set the imu noise model to a gaussian model
            acc_noise_model =
                chrono_types::make_shared<ChNoiseNormalDrift>(imu_update_rate,                    //
                                                              ChVector3d({0., 0., 0.}),           // mean,
                                                              ChVector3d({0.2, 0.2, 0.2}),  // stdev,
                                                              .02,                              // bias_drift,
                                                              .1);                                // tau_drift,
            gyro_noise_model =
                chrono_types::make_shared<ChNoiseNormalDrift>(imu_update_rate,                    // float updateRate,
                                                              ChVector3d({0., 0., 0.}),           // float mean,
                                                              ChVector3d({0.05, 0.05, 0.05}),  // float
                                                              .00004,                               // double bias_drift,
                                                              .1);                                // double tau_drift,
            mag_noise_model =
                chrono_types::make_shared<ChNoiseNormal>(ChVector3d({0., 0., 0.}),            // float mean,
                                                         ChVector3d({0.00, 0.00, 0.00}));  // float stdev,
            break;
        case IMU_NONE:
            // Set the imu noise model to none (does not affect the data)
            acc_noise_model = chrono_types::make_shared<ChNoiseNone>();
            gyro_noise_model = chrono_types::make_shared<ChNoiseNone>();
            mag_noise_model = chrono_types::make_shared<ChNoiseNone>();
            break;
    }



    // add an accelerometer, gyroscope, and magnetometer
    chrono::ChFrame<double> imu_offset_pose({0.0, 0.0, 0.0}, QuatFromAngleAxis(0, {1, 0, 0}));
    // chrono::ChFrame<double> imu_offset_pose({0.0, 0.0, 0.0}, QuatFromAngleY( CH_PI));

    ChVector3d gps_reference(-89.400, 43.070, 260.0);
    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(car.GetChassisBody(),    // body to which the IMU is attached
                                                                imu_update_rate,   // update rate
                                                                imu_offset_pose,   // offset pose from body
                                                                acc_noise_model);  // IMU noise model
    acc->SetName("IMU - Accelerometer");
    acc->SetLag(imu_lag);
    acc->SetCollectionWindow(imu_collection_time);
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());  // Add a filter to access the imu data
    manager->AddSensor(acc);                                            // Add the IMU sensor to the sensor manager

    auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(car.GetChassisBody(),     // body to which the IMU is attached
                                                             imu_update_rate,    // update rate
                                                             imu_offset_pose,    // offset pose from body
                                                             gyro_noise_model);  // IMU noise model
    gyro->SetName("IMU - Accelerometer");
    gyro->SetLag(imu_lag);
    gyro->SetCollectionWindow(imu_collection_time);
    gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());  // Add a filter to access the imu data
    manager->AddSensor(gyro);                                           // Add the IMU sensor to the sensor manager

    auto mag = chrono_types::make_shared<ChMagnetometerSensor>(car.GetChassisBody(),            // body to which the IMU is attached
                                                               imu_update_rate,  // update rate
                                                               imu_offset_pose,  // offset pose from body
                                                               mag_noise_model,  // IMU noise model
                                                               gps_reference);
    mag->SetName("IMU - Accelerometer");
    mag->SetLag(imu_lag);
    mag->SetCollectionWindow(imu_collection_time);
    mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());  // Add a filter to access the imu data
    manager->AddSensor(mag);      

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


    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>("ARTcar");

    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager->RegisterHandler(clock_handler);

    // cameras managers
    auto camera_rate = cam1->GetUpdateRate();
    auto camera_topic_name_r = "~/right_camera";
    auto camera_topic_name_l = "~/left_camera";
    // adding right camera
    auto camera_handler_r = chrono_types::make_shared<ChROSCameraHandler>(camera_rate, cam2, camera_topic_name_r);
    ros_manager->RegisterHandler(camera_handler_r);
    // adding left camera
    auto camera_handler_l = chrono_types::make_shared<ChROSCameraHandler>(camera_rate, cam1, camera_topic_name_l);
    ros_manager->RegisterHandler(camera_handler_l);

    // imu manager
    auto acc_rate = acc->GetUpdateRate();
    std::cout << "IMU RATE: " << acc_rate << std::endl;
    auto acc_topic_name = "~/accelerometer";
    auto acc_handler = chrono_types::make_shared<ChROSAccelerometerHandler>(acc_rate, acc, acc_topic_name);
    ros_manager->RegisterHandler(acc_handler);

    // Create the publisher for the gyroscope
    auto gyro_topic_name = "~/gyroscope";
    auto gyro_handler = chrono_types::make_shared<ChROSGyroscopeHandler>(gyro, gyro_topic_name);
    ros_manager->RegisterHandler(gyro_handler);

    // Create the publisher for the magnetometer
    auto mag_topic_name = "~/magnetometer";
    auto mag_handler = chrono_types::make_shared<ChROSMagnetometerHandler>(mag, mag_topic_name);
    ros_manager->RegisterHandler(mag_handler);

    // Create the publisher for _all_ imu sensors
    auto imu_topic_name = "~/imu";
    auto imu_handler = chrono_types::make_shared<ChROSIMUHandler>(acc_rate, imu_topic_name);
    imu_handler->SetAccelerometerHandler(acc_handler);
    imu_handler->SetGyroscopeHandler(gyro_handler);
    imu_handler->SetMagnetometerHandler(mag_handler);
    ros_manager->RegisterHandler(imu_handler);

    // Set name
    car.GetChassisBody()->SetName("chassis");
    room_mesh_body->SetName("world");

    auto tf_handler = chrono_types::make_shared<ChROSTFHandler>(100);
    tf_handler->AddTransform(room_mesh_body, car.GetChassisBody());
    ros_manager->RegisterHandler(tf_handler);

    // initialize ros manager
    ros_manager->Initialize();

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

    // car.GetVehicle().EnableRealtime(true);
        // Start timing
    auto overall_start = std::chrono::high_resolution_clock::now();
    double previous_time = 0.0; // Assuming simulation starts at time = 0.0
    while (vis->Run()) {
        double time = car.GetSystem()->GetChTime();

        // End simulation
        if (time >= t_end)
            break;

        // Render scene and output POV-Ray data
        if(render){
            if (step_number % render_steps == 0) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();


                render_frame++;
            }
        }

        // Get driver inputs
        DriverInputs driver_inputs = driver_vsg->GetInputs();
        if(time < 5){
            driver_inputs.m_throttle = 0.0;
            driver_inputs.m_steering = 0.0;
            driver_inputs.m_braking = 0.0;

        }
        else{
            driver_inputs.m_throttle = 0.5;
            driver_inputs.m_steering = 0.5;
            driver_inputs.m_braking = 0.0;
        }

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

        if (!ros_manager->Update(time, step_size))
            break;

        // Increment frame number
        step_number++;

        // Calculate and update RTF at each step
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = current_time - overall_start;
        double simulated_time = time; // Change in simulation time since last update
        // previous_time = time; // Update previous_time for the next iteration

        // To prevent division by zero if simulated_time is 0 (e.g., at the very start)
        if(simulated_time > 0) {
            double step_rtf = elapsed.count() / simulated_time;
            std::cout << "Current RTF: " << step_rtf << std::endl;
        }
    }


    return 0;
}