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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Radu Serban
// =============================================================================
//
// Implementation of FSI system based on an SPH fluid solver.
//
// =============================================================================

//// TODO:
////   - use SimParams::C_Wi (kernel threshold) for both CFD and CRM (currently, only CRM)

#include <cmath>

#include "chrono/core/ChTypes.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/sph/ChFsiInterfaceSPH.h"
#include "chrono_fsi/sph/physics/ChParams.h"
#include "chrono_fsi/sph/physics/FsiDataManager.cuh"
#include "chrono_fsi/sph/physics/ChFluidDynamics.cuh"
#include "chrono_fsi/sph/physics/BceManager.cuh"
#include "chrono_fsi/sph/utils/ChUtilsTypeConvert.h"
#include "chrono_fsi/sph/utils/ChUtilsPrintSph.cuh"
#include "chrono_fsi/sph/utils/ChUtilsDevice.cuh"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {

using namespace sph;

ChFsiSystemSPH::ChFsiSystemSPH(ChSystem* sysMBS)
    : ChFsiSystem(sysMBS),
      m_is_initialized(false),
      m_time(0),
      m_pattern1D(BcePatternMesh1D::FULL),
      m_pattern2D(BcePatternMesh2D::CENTERED),
      m_remove_center1D(false),
      m_remove_center2D(false) {
    m_paramsH = chrono_types::make_shared<SimParams>();
    InitParams();

    m_num_flex1D_elements = 0;
    m_num_flex2D_elements = 0;

    m_num_flex1D_nodes = 0;
    m_num_flex2D_nodes = 0;

    m_data_mgr = chrono_types::make_unique<FsiDataManager>(m_paramsH);
    m_fsi_interface = chrono_types::make_unique<ChFsiInterfaceSPH>(*m_data_mgr, m_verbose);
}

ChFsiSystemSPH::~ChFsiSystemSPH() {}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiSystemSPH::InitParams() {
    //// RADU TODO
    //// Provide default values for *all* parameters!

    m_paramsH->output_length = 1;

    // Fluid properties
    m_paramsH->rho0 = Real(1000.0);
    m_paramsH->invrho0 = 1 / m_paramsH->rho0;
    m_paramsH->rho_solid = m_paramsH->rho0;
    m_paramsH->mu0 = Real(0.001);
    m_paramsH->bodyForce3 = mR3(0, 0, 0);
    m_paramsH->gravity = mR3(0, 0, 0);
    m_paramsH->kappa = Real(0.0);
    m_paramsH->L_Characteristic = Real(1.0);

    // SPH parameters
    m_paramsH->sph_method = SPHMethod::WCSPH;
    m_paramsH->HSML = Real(0.01);
    m_paramsH->INVHSML = 1 / m_paramsH->HSML;
    m_paramsH->INITSPACE = m_paramsH->HSML;
    m_paramsH->volume0 = cube(m_paramsH->INITSPACE);
    m_paramsH->INV_INIT = 1 / m_paramsH->INITSPACE;
    m_paramsH->v_Max = Real(1.0);
    m_paramsH->EPS_XSPH = Real(0.5);
    m_paramsH->beta_shifting = Real(1.0);
    m_paramsH->densityReinit = 2147483647;
    m_paramsH->Conservative_Form = true;
    m_paramsH->gradient_type = 0;
    m_paramsH->laplacian_type = 0;
    m_paramsH->USE_Consistent_L = false;
    m_paramsH->USE_Consistent_G = false;

    m_paramsH->epsMinMarkersDis = Real(0.01);

    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;

    m_paramsH->NUM_BCE_LAYERS = 3;

    // Time stepping
    m_paramsH->Adaptive_time_stepping = false;
    m_paramsH->Co_number = Real(0.1);
    m_paramsH->dT = Real(0.0001);
    m_paramsH->INV_dT = 1 / m_paramsH->dT;
    m_paramsH->dT_Flex = m_paramsH->dT;
    m_paramsH->dT_Max = Real(1.0);

    // Pressure equation
    m_paramsH->DensityBaseProjection = false;
    m_paramsH->Alpha = m_paramsH->HSML;
    m_paramsH->PPE_relaxation = Real(1.0);
    m_paramsH->LinearSolver = SolverType::JACOBI;
    m_paramsH->LinearSolver_Abs_Tol = Real(0.0);
    m_paramsH->LinearSolver_Rel_Tol = Real(0.0);
    m_paramsH->LinearSolver_Max_Iter = 1000;
    m_paramsH->Verbose_monitoring = false;
    m_paramsH->Pressure_Constraint = false;
    m_paramsH->BASEPRES = Real(0.0);
    m_paramsH->ClampPressure = false;

    // Elastic SPH
    m_paramsH->C_Wi = Real(0.8);

    //
    m_paramsH->bodyActiveDomain = mR3(1e10, 1e10, 1e10);
    m_paramsH->settlingTime = Real(0);

    //
    m_paramsH->Max_Pressure = Real(1e20);

    //// RADU TODO
    //// material model

    // Elastic SPH
    ElasticMaterialProperties mat_props;
    SetElasticSPH(mat_props);
    m_paramsH->elastic_SPH = false;  // default: fluid dynamics

    m_paramsH->Cs = 10 * m_paramsH->v_Max;

    m_paramsH->use_default_limits = true;
    m_paramsH->use_init_pressure = false;

    m_paramsH->num_proximity_search_steps = 4;
}

//--------------------------------------------------------------------------------------------------------------------------------

Real3 LoadVectorJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return mR3(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

void ChFsiSystemSPH::ReadParametersFromFile(const std::string& json_file) {
    if (m_verbose)
        cout << "Reading parameters from: " << json_file << endl;

    FILE* fp = fopen(json_file.c_str(), "r");
    if (!fp) {
        cerr << "Invalid JSON file!" << endl;
        return;
    }

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    fclose(fp);

    Document doc;

    doc.ParseStream<ParseFlag::kParseCommentsFlag>(is);
    if (!doc.IsObject()) {
        cerr << "Invalid JSON file!!" << endl;
        return;
    }

    if (doc.HasMember("Data Output Length"))
        m_paramsH->output_length = doc["Data Output Length"].GetInt();

    if (doc.HasMember("Physical Properties of Fluid")) {
        if (doc["Physical Properties of Fluid"].HasMember("Density"))
            m_paramsH->rho0 = doc["Physical Properties of Fluid"]["Density"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Solid Density"))
            m_paramsH->rho_solid = doc["Physical Properties of Fluid"]["Solid Density"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Viscosity"))
            m_paramsH->mu0 = doc["Physical Properties of Fluid"]["Viscosity"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Body Force"))
            m_paramsH->bodyForce3 = LoadVectorJSON(doc["Physical Properties of Fluid"]["Body Force"]);

        if (doc["Physical Properties of Fluid"].HasMember("Gravity"))
            m_paramsH->gravity = LoadVectorJSON(doc["Physical Properties of Fluid"]["Gravity"]);

        if (doc["Physical Properties of Fluid"].HasMember("Surface Tension Kappa"))
            m_paramsH->kappa = doc["Physical Properties of Fluid"]["Surface Tension Kappa"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Characteristic Length"))
            m_paramsH->L_Characteristic = doc["Physical Properties of Fluid"]["Characteristic Length"].GetDouble();
    }

    if (doc.HasMember("SPH Parameters")) {
        if (doc["SPH Parameters"].HasMember("Method")) {
            std::string SPH = doc["SPH Parameters"]["Method"].GetString();
            if (m_verbose)
                cout << "Modeling method is: " << SPH << endl;
            if (SPH == "I2SPH")
                m_paramsH->sph_method = SPHMethod::I2SPH;
            else if (SPH == "WCSPH")
                m_paramsH->sph_method = SPHMethod::WCSPH;
            else {
                cerr << "Incorrect SPH method in the JSON file: " << SPH << endl;
                cerr << "Falling back to WCSPH " << endl;
                m_paramsH->sph_method = SPHMethod::WCSPH;
            }
        }

        if (doc["SPH Parameters"].HasMember("Kernel h"))
            m_paramsH->HSML = doc["SPH Parameters"]["Kernel h"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Initial Spacing"))
            m_paramsH->INITSPACE = doc["SPH Parameters"]["Initial Spacing"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Epsilon"))
            m_paramsH->epsMinMarkersDis = doc["SPH Parameters"]["Epsilon"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Maximum Velocity"))
            m_paramsH->v_Max = doc["SPH Parameters"]["Maximum Velocity"].GetDouble();

        if (doc["SPH Parameters"].HasMember("XSPH Coefficient"))
            m_paramsH->EPS_XSPH = doc["SPH Parameters"]["XSPH Coefficient"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Viscous damping"))
            m_paramsH->Vis_Dam = doc["SPH Parameters"]["Viscous damping"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Shifting Coefficient"))
            m_paramsH->beta_shifting = doc["SPH Parameters"]["Shifting Coefficient"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Density Reinitialization"))
            m_paramsH->densityReinit = doc["SPH Parameters"]["Density Reinitialization"].GetInt();

        if (doc["SPH Parameters"].HasMember("Conservative Discretization"))
            m_paramsH->Conservative_Form = doc["SPH Parameters"]["Conservative Discretization"].GetBool();

        if (doc["SPH Parameters"].HasMember("Gradient Discretization Type"))
            m_paramsH->gradient_type = doc["SPH Parameters"]["Gradient Discretization Type"].GetInt();

        if (doc["SPH Parameters"].HasMember("Laplacian Discretization Type"))
            m_paramsH->laplacian_type = doc["SPH Parameters"]["Laplacian Discretization Type"].GetInt();

        if (doc["SPH Parameters"].HasMember("Consistent Discretization for Laplacian"))
            m_paramsH->USE_Consistent_L = doc["SPH Parameters"]["Consistent Discretization for Laplacian"].GetBool();

        if (doc["SPH Parameters"].HasMember("Consistent Discretization for Gradient"))
            m_paramsH->USE_Consistent_G = doc["SPH Parameters"]["Consistent Discretization for Gradient"].GetBool();

        if (doc["SPH Parameters"].HasMember("Time steps per proximity search"))
            m_paramsH->num_proximity_search_steps = doc["SPH Parameters"]["Time steps per proximity search"].GetInt();
    }

    if (doc.HasMember("Time Stepping")) {
        if (doc["Time Stepping"].HasMember("Adaptive Time stepping"))
            m_paramsH->Adaptive_time_stepping = doc["Time Stepping"]["Adaptive Time stepping"].GetBool();

        if (doc["Time Stepping"].HasMember("CFL number"))
            m_paramsH->Co_number = doc["Time Stepping"]["CFL number"].GetDouble();

        if (doc["Time Stepping"].HasMember("Fluid time step"))
            m_paramsH->dT = doc["Time Stepping"]["Fluid time step"].GetDouble();

        if (doc["Time Stepping"].HasMember("Solid time step"))
            m_paramsH->dT_Flex = doc["Time Stepping"]["Solid time step"].GetDouble();
        else
            m_paramsH->dT_Flex = m_paramsH->dT;

        if (doc["Time Stepping"].HasMember("Maximum time step"))
            m_paramsH->dT_Max = doc["Time Stepping"]["Maximum time step"].GetDouble();
    }

    if (doc.HasMember("Pressure Equation")) {
        if (doc["Pressure Equation"].HasMember("Linear solver")) {
            std::string solver = doc["Pressure Equation"]["Linear solver"].GetString();
            if (solver == "Jacobi") {
                m_paramsH->LinearSolver = SolverType::JACOBI;
            } else if (solver == "BICGSTAB") {
                m_paramsH->LinearSolver = SolverType::BICGSTAB;
            } else if (solver == "GMRES")
                m_paramsH->LinearSolver = SolverType::GMRES;
        } else {
            m_paramsH->LinearSolver = SolverType::JACOBI;
        }

        if (doc["Pressure Equation"].HasMember("Poisson source term")) {
            std::string source = doc["Pressure Equation"]["Poisson source term"].GetString();
            if (source == "Density-Based")
                m_paramsH->DensityBaseProjection = true;
            else
                m_paramsH->DensityBaseProjection = false;
        }

        if (doc["Pressure Equation"].HasMember("Alpha Source Term"))
            m_paramsH->Alpha = doc["Pressure Equation"]["Alpha Source Term"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Under-relaxation"))
            m_paramsH->PPE_relaxation = doc["Pressure Equation"]["Under-relaxation"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Absolute residual"))
            m_paramsH->LinearSolver_Abs_Tol = doc["Pressure Equation"]["Absolute residual"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Relative residual"))
            m_paramsH->LinearSolver_Rel_Tol = doc["Pressure Equation"]["Relative residual"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Maximum Iterations"))
            m_paramsH->LinearSolver_Max_Iter = doc["Pressure Equation"]["Maximum Iterations"].GetInt();

        if (doc["Pressure Equation"].HasMember("Verbose monitoring"))
            m_paramsH->Verbose_monitoring = doc["Pressure Equation"]["Verbose monitoring"].GetBool();

        if (doc["Pressure Equation"].HasMember("Constraint Pressure")) {
            m_paramsH->Pressure_Constraint = doc["Pressure Equation"]["Constraint Pressure"].GetBool();
            if (doc["Pressure Equation"].HasMember("Average Pressure"))
                m_paramsH->BASEPRES = doc["Pressure Equation"]["Average Pressure"].GetDouble();
        }

        if (doc["Pressure Equation"].HasMember("Clamp Pressure"))
            m_paramsH->ClampPressure = doc["Pressure Equation"]["Clamp Pressure"].GetBool();
    }

    // this part is for modeling granular material dynamics using elastic SPH
    if (doc.HasMember("Elastic SPH")) {
        m_paramsH->elastic_SPH = true;

        if (doc["Elastic SPH"].HasMember("Poisson ratio"))
            m_paramsH->Nu_poisson = doc["Elastic SPH"]["Poisson ratio"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Young modulus"))
            m_paramsH->E_young = doc["Elastic SPH"]["Young modulus"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Artificial stress"))
            m_paramsH->Ar_stress = doc["Elastic SPH"]["Artificial stress"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Artificial viscosity alpha"))
            m_paramsH->Ar_vis_alpha = doc["Elastic SPH"]["Artificial viscosity alpha"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Artificial viscosity beta"))
            m_paramsH->Ar_vis_beta = doc["Elastic SPH"]["Artificial viscosity beta"].GetDouble();

        if (doc["Elastic SPH"].HasMember("I0"))
            m_paramsH->mu_I0 = doc["Elastic SPH"]["I0"].GetDouble();

        if (doc["Elastic SPH"].HasMember("mu_s"))
            m_paramsH->mu_fric_s = doc["Elastic SPH"]["mu_s"].GetDouble();

        if (doc["Elastic SPH"].HasMember("mu_2"))
            m_paramsH->mu_fric_2 = doc["Elastic SPH"]["mu_2"].GetDouble();

        if (doc["Elastic SPH"].HasMember("particle diameter"))
            m_paramsH->ave_diam = doc["Elastic SPH"]["particle diameter"].GetDouble();

        if (doc["Elastic SPH"].HasMember("frictional angle"))
            m_paramsH->Fri_angle = doc["Elastic SPH"]["frictional angle"].GetDouble();

        if (doc["Elastic SPH"].HasMember("dilate angle"))
            m_paramsH->Dil_angle = doc["Elastic SPH"]["dilate angle"].GetDouble();

        if (doc["Elastic SPH"].HasMember("cohesion coefficient"))
            m_paramsH->Coh_coeff = doc["Elastic SPH"]["cohesion coefficient"].GetDouble();

        if (doc["Elastic SPH"].HasMember("kernel threshold"))
            m_paramsH->C_Wi = doc["Elastic SPH"]["kernel threshold"].GetDouble();
    }

    // Geometry Information
    if (doc.HasMember("Geometry Inf")) {
        if (doc["Geometry Inf"].HasMember("BoxDimensionX"))
            m_paramsH->boxDimX = doc["Geometry Inf"]["BoxDimensionX"].GetDouble();

        if (doc["Geometry Inf"].HasMember("BoxDimensionY"))
            m_paramsH->boxDimY = doc["Geometry Inf"]["BoxDimensionY"].GetDouble();

        if (doc["Geometry Inf"].HasMember("BoxDimensionZ"))
            m_paramsH->boxDimZ = doc["Geometry Inf"]["BoxDimensionZ"].GetDouble();
    }

    if (doc.HasMember("Body Active Domain"))
        m_paramsH->bodyActiveDomain = LoadVectorJSON(doc["Body Active Domain"]);

    if (doc.HasMember("Settling Time"))
        m_paramsH->settlingTime = doc["Settling Time"].GetDouble();

    //===============================================================
    // Material Models
    //===============================================================
    if (doc.HasMember("Material Model")) {
        m_paramsH->non_newtonian = doc["Material Model"]["Non-Newtonian"].GetBool();
        //===============================================================
        // For a simple non-newtonian flow
        //==============================================================
        if (m_paramsH->non_newtonian) {
            m_paramsH->mu_max = doc["Material Model"]["max Viscosity"].GetDouble();

            if (m_paramsH->non_newtonian) {
                if (doc["Material Model"].HasMember("HerschelBulkley")) {
                    m_paramsH->HB_k = doc["Material Model"]["HerschelBulkley"]["k"].GetDouble();
                    m_paramsH->HB_n = doc["Material Model"]["HerschelBulkley"]["n"].GetInt();
                    m_paramsH->HB_tau0 = doc["Material Model"]["HerschelBulkley"]["tau_0"].GetDouble();
                    if (doc["Material Model"]["HerschelBulkley"].HasMember("sr0"))
                        m_paramsH->HB_sr0 = doc["Material Model"]["HerschelBulkley"]["sr0"].GetDouble();
                    else
                        m_paramsH->HB_sr0 = 0.0;
                } else {
                    if (m_verbose)
                        cout << "Constants of HerschelBulkley not found. Using default Newtonian values." << endl;
                    m_paramsH->HB_k = m_paramsH->mu0;
                    m_paramsH->HB_n = 1;
                    m_paramsH->HB_tau0 = 0;
                    m_paramsH->HB_sr0 = 0.0;
                }
            }
        }
    } else {
        m_paramsH->non_newtonian = false;
    }

    // Calculate dependent parameters
    m_paramsH->INVHSML = 1 / m_paramsH->HSML;
    m_paramsH->INV_INIT = 1 / m_paramsH->INITSPACE;
    m_paramsH->volume0 = cube(m_paramsH->INITSPACE);
    m_paramsH->MULT_INITSPACE = m_paramsH->INITSPACE / m_paramsH->HSML;
    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;
    m_paramsH->INV_dT = 1 / m_paramsH->dT;
    m_paramsH->invrho0 = 1 / m_paramsH->rho0;

    if (m_paramsH->elastic_SPH) {
        m_paramsH->G_shear = m_paramsH->E_young / (2.0 * (1.0 + m_paramsH->Nu_poisson));
        m_paramsH->INV_G_shear = 1.0 / m_paramsH->G_shear;
        m_paramsH->K_bulk = m_paramsH->E_young / (3.0 * (1.0 - 2.0 * m_paramsH->Nu_poisson));
        m_paramsH->Cs = sqrt(m_paramsH->K_bulk / m_paramsH->rho0);

        Real sfri = std::sin(m_paramsH->Fri_angle);
        Real cfri = std::cos(m_paramsH->Fri_angle);
        Real sdil = std::sin(m_paramsH->Dil_angle);
        m_paramsH->Q_FA = 6 * sfri / (sqrt(3) * (3 + sfri));
        m_paramsH->Q_DA = 6 * sdil / (sqrt(3) * (3 + sdil));
        m_paramsH->K_FA = 6 * m_paramsH->Coh_coeff * cfri / (sqrt(3) * (3 + sfri));
    } else {
        m_paramsH->Cs = 10 * m_paramsH->v_Max;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiSystemSPH::SetVerbose(bool verbose) {
    ChFsiSystem::SetVerbose(verbose);
}

void ChFsiSystemSPH::SetSPHLinearSolver(SolverType lin_solver) {
    m_paramsH->LinearSolver = lin_solver;
}

void ChFsiSystemSPH::SetSPHMethod(SPHMethod SPH_method) {
    m_paramsH->sph_method = SPH_method;
}

void ChFsiSystemSPH::SetContainerDim(const ChVector3d& boxDim) {
    m_paramsH->boxDimX = boxDim.x();
    m_paramsH->boxDimY = boxDim.y();
    m_paramsH->boxDimZ = boxDim.z();
}

void ChFsiSystemSPH::SetBoundaries(const ChVector3d& cMin, const ChVector3d& cMax) {
    m_paramsH->cMin = ToReal3(cMin);
    m_paramsH->cMax = ToReal3(cMax);
    m_paramsH->use_default_limits = false;
}

void ChFsiSystemSPH::SetActiveDomain(const ChVector3d& boxHalfDim) {
    m_paramsH->bodyActiveDomain = ToReal3(boxHalfDim);
}

void ChFsiSystemSPH::SetActiveDomainDelay(double duration) {
    m_paramsH->settlingTime = duration;
}

void ChFsiSystemSPH::SetNumBCELayers(int num_layers) {
    m_paramsH->NUM_BCE_LAYERS = num_layers;
}

void ChFsiSystemSPH::SetInitPressure(const double height) {
    m_paramsH->pressure_height = height;
    m_paramsH->use_init_pressure = true;
}

void ChFsiSystemSPH::SetGravitationalAcceleration(const ChVector3d& gravity) {
    m_paramsH->gravity.x = gravity.x();
    m_paramsH->gravity.y = gravity.y();
    m_paramsH->gravity.z = gravity.z();
}

void ChFsiSystemSPH::SetBodyForce(const ChVector3d& force) {
    m_paramsH->bodyForce3.x = force.x();
    m_paramsH->bodyForce3.y = force.y();
    m_paramsH->bodyForce3.z = force.z();
}

void ChFsiSystemSPH::SetInitialSpacing(double spacing) {
    m_paramsH->INITSPACE = (Real)spacing;
    m_paramsH->INV_INIT = 1 / m_paramsH->INITSPACE;
    m_paramsH->volume0 = cube(m_paramsH->INITSPACE);
    m_paramsH->MULT_INITSPACE = m_paramsH->INITSPACE / m_paramsH->HSML;
    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;
}

void ChFsiSystemSPH::SetKernelLength(double length) {
    m_paramsH->HSML = (Real)length;
    m_paramsH->MULT_INITSPACE = m_paramsH->INITSPACE / m_paramsH->HSML;
    m_paramsH->INVHSML = 1 / m_paramsH->HSML;
}

void ChFsiSystemSPH::SetStepSize(double dT, double dT_Flex) {
    m_paramsH->dT = dT;
    m_paramsH->INV_dT = 1 / m_paramsH->dT;
    m_paramsH->dT_Flex = (dT_Flex == 0) ? m_paramsH->dT : dT_Flex;
}

void ChFsiSystemSPH::SetMaxStepSize(double dT_max) {
    m_paramsH->dT_Max = Real(dT_max);
}

void ChFsiSystemSPH::SetAdaptiveTimeStepping(bool adaptive) {
    m_paramsH->Adaptive_time_stepping = adaptive;
}

void ChFsiSystemSPH::SetDensity(double rho0) {
    m_paramsH->rho0 = rho0;
    m_paramsH->invrho0 = 1 / m_paramsH->rho0;
    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;
}

void ChFsiSystemSPH::SetConsistentDerivativeDiscretization(bool consistent_gradient, bool consistent_Laplacian) {
    m_paramsH->USE_Consistent_G = consistent_gradient;
    m_paramsH->USE_Consistent_L = consistent_Laplacian;
}

void ChFsiSystemSPH::SetOutputLength(int OutputLength) {
    m_paramsH->output_length = OutputLength;
}

void ChFsiSystemSPH::SetCohesionForce(double Fc) {
    m_paramsH->Coh_coeff = Fc;
}

void ChFsiSystemSPH::SetNumProximitySearchSteps(int steps) {
    m_paramsH->num_proximity_search_steps = steps;
}

ChFsiSystemSPH::FluidProperties::FluidProperties() : density(1000), viscosity(0.1), kappa(0), char_length(1) {}

void ChFsiSystemSPH::SetCfdSPH(const FluidProperties& fluid_props) {
    m_paramsH->elastic_SPH = false;

    SetDensity(fluid_props.density);

    m_paramsH->mu0 = Real(fluid_props.viscosity);
    m_paramsH->kappa = Real(fluid_props.kappa);
    m_paramsH->L_Characteristic = Real(fluid_props.char_length);
}

ChFsiSystemSPH::ElasticMaterialProperties::ElasticMaterialProperties()
    : density(1000),
      Young_modulus(1e6),
      Poisson_ratio(0.3),
      stress(0),
      viscosity_alpha(0.5),
      viscosity_beta(0),
      mu_I0(0.03),
      mu_fric_s(0.7),
      mu_fric_2(0.7),
      average_diam(0.005),
      friction_angle(CH_PI / 10),
      dilation_angle(CH_PI / 10),
      cohesion_coeff(0) {}

void ChFsiSystemSPH::SetElasticSPH(const ElasticMaterialProperties& mat_props) {
    m_paramsH->elastic_SPH = true;

    SetDensity(mat_props.density);

    m_paramsH->E_young = Real(mat_props.Young_modulus);
    m_paramsH->Nu_poisson = Real(mat_props.Poisson_ratio);
    m_paramsH->Ar_stress = Real(mat_props.stress);
    m_paramsH->Ar_vis_alpha = Real(mat_props.viscosity_alpha);
    m_paramsH->Ar_vis_beta = Real(mat_props.viscosity_beta);
    m_paramsH->mu_I0 = Real(mat_props.mu_I0);
    m_paramsH->mu_fric_s = Real(mat_props.mu_fric_s);
    m_paramsH->mu_fric_2 = Real(mat_props.mu_fric_2);
    m_paramsH->ave_diam = Real(mat_props.average_diam);
    m_paramsH->Fri_angle = Real(mat_props.friction_angle);
    m_paramsH->Dil_angle = Real(mat_props.dilation_angle);
    m_paramsH->Coh_coeff = Real(mat_props.cohesion_coeff);

    m_paramsH->G_shear = m_paramsH->E_young / (2.0 * (1.0 + m_paramsH->Nu_poisson));
    m_paramsH->INV_G_shear = 1.0 / m_paramsH->G_shear;
    m_paramsH->K_bulk = m_paramsH->E_young / (3.0 * (1.0 - 2.0 * m_paramsH->Nu_poisson));
    m_paramsH->Cs = sqrt(m_paramsH->K_bulk / m_paramsH->rho0);

    Real sfri = std::sin(m_paramsH->Fri_angle);
    Real cfri = std::cos(m_paramsH->Fri_angle);
    Real sdil = std::sin(m_paramsH->Dil_angle);
    m_paramsH->Q_FA = 6 * sfri / (sqrt(3) * (3 + sfri));
    m_paramsH->Q_DA = 6 * sdil / (sqrt(3) * (3 + sdil));
    m_paramsH->K_FA = 6 * m_paramsH->Coh_coeff * cfri / (sqrt(3) * (3 + sfri));
}

ChFsiSystemSPH::SPHParameters::SPHParameters()
    : sph_method(SPHMethod::WCSPH),
      kernel_h(0.01),
      initial_spacing(0.01),
      max_velocity(1.0),
      xsph_coefficient(0.5),
      shifting_coefficient(1.0),
      min_distance_coefficient(0.01),
      density_reinit_steps(2e8),
      use_density_based_projection(false),
      num_bce_layers(3),
      consistent_gradient_discretization(false),
      consistent_laplacian_discretization(false),
      kernel_threshold(0.8),
      num_proximity_search_steps(4) {}

void ChFsiSystemSPH::SetSPHParameters(const SPHParameters& sph_params) {
    m_paramsH->sph_method = sph_params.sph_method;

    m_paramsH->HSML = sph_params.kernel_h;
    m_paramsH->INITSPACE = sph_params.initial_spacing;
    m_paramsH->MULT_INITSPACE = m_paramsH->INITSPACE / m_paramsH->HSML;
    m_paramsH->INVHSML = 1 / m_paramsH->HSML;

    m_paramsH->v_Max = sph_params.max_velocity;
    m_paramsH->Cs = 10 * m_paramsH->v_Max;
    m_paramsH->EPS_XSPH = sph_params.xsph_coefficient;
    m_paramsH->beta_shifting = sph_params.shifting_coefficient;
    m_paramsH->epsMinMarkersDis = sph_params.min_distance_coefficient;

    m_paramsH->densityReinit = sph_params.density_reinit_steps;
    m_paramsH->DensityBaseProjection = sph_params.use_density_based_projection;

    m_paramsH->NUM_BCE_LAYERS = sph_params.num_bce_layers;

    m_paramsH->USE_Consistent_G = sph_params.consistent_gradient_discretization;
    m_paramsH->USE_Consistent_L = sph_params.consistent_laplacian_discretization;

    m_paramsH->C_Wi = Real(sph_params.kernel_threshold);

    m_paramsH->num_proximity_search_steps = sph_params.num_proximity_search_steps;
}

ChFsiSystemSPH::LinSolverParameters::LinSolverParameters()
    : type(SolverType::JACOBI), atol(0.0), rtol(0.0), max_num_iters(1000) {}

void ChFsiSystemSPH::SetLinSolverParameters(const LinSolverParameters& linsolv_params) {
    m_paramsH->LinearSolver = linsolv_params.type;
    m_paramsH->LinearSolver_Abs_Tol = linsolv_params.atol;
    m_paramsH->LinearSolver_Rel_Tol = linsolv_params.rtol;
    m_paramsH->LinearSolver_Max_Iter = linsolv_params.max_num_iters;
}

//--------------------------------------------------------------------------------------------------------------------------------

PhysicsProblem ChFsiSystemSPH::GetPhysicsProblem() const {
    return (m_paramsH->elastic_SPH ? PhysicsProblem::CRM : PhysicsProblem::CFD);
}

std::string ChFsiSystemSPH::GetPhysicsProblemString() const {
    return (m_paramsH->elastic_SPH ? "CRM" : "CFD");
}

std::string ChFsiSystemSPH::GetSphMethodTypeString() const {
    std::string method = "";
    switch (m_paramsH->sph_method) {
        case SPHMethod::WCSPH:
            method = "WCSPH";
            break;
        case SPHMethod::I2SPH:
            method = "I2SPH";
            break;
    }

    return method;
}

//--------------------------------------------------------------------------------------------------------------------------------

size_t ChFsiSystemSPH::AddFsiBody(std::shared_ptr<ChBody> body) {
    size_t index = m_fsi_interface->GetNumBodies();

    /* auto& fsi_body =*/m_fsi_interface->AddFsiBody(body);

    return index;
}

void ChFsiSystemSPH::SetBcePattern1D(BcePatternMesh1D pattern, bool remove_center) {
    m_pattern1D = pattern;
    m_remove_center1D = remove_center;
}

void ChFsiSystemSPH::SetBcePattern2D(BcePatternMesh2D pattern, bool remove_center) {
    m_pattern2D = pattern;
    m_remove_center2D = remove_center;
}

void ChFsiSystemSPH::AddFsiMesh(std::shared_ptr<fea::ChMesh> mesh) {
    bool has_contact1D = false;
    bool has_contact2D = false;

    // Search for contact surfaces associated with the FEA mesh
    for (const auto& surface : mesh->GetContactSurfaces()) {
        if (auto surface_segs = std::dynamic_pointer_cast<fea::ChContactSurfaceSegmentSet>(surface)) {
            if (surface_segs->GetNumSegments() > 0)
                AddFsiMesh1D(surface_segs);
            has_contact1D = true;
        } else if (auto surface_mesh = std::dynamic_pointer_cast<fea::ChContactSurfaceMesh>(surface)) {
            if (surface_mesh->GetNumTriangles() > 0)
                AddFsiMesh2D(surface_mesh);
            has_contact2D = true;
        }
    }

    // If no 1D surface contact found, create one with a default contact material and add segments from cable and beam
    // elements in the FEA mesh
    if (!has_contact1D) {
        ChContactMaterialData contact_material_data;  // default contact material
        auto surface_segs = chrono_types::make_shared<fea::ChContactSurfaceSegmentSet>(
            contact_material_data.CreateMaterial(ChContactMethod::SMC));
        mesh->AddContactSurface(surface_segs);
        surface_segs->AddAllSegments(0);
        if (surface_segs->GetNumSegments() > 0)
            AddFsiMesh1D(surface_segs);
    }

    // If no 2D surface contact found, create one with a default contact material and extract the boundary faces of the
    // FEA mesh
    if (!has_contact2D) {
        ChContactMaterialData contact_material_data;  // default contact material
        auto surface_mesh = chrono_types::make_shared<fea::ChContactSurfaceMesh>(
            contact_material_data.CreateMaterial(ChContactMethod::SMC));
        mesh->AddContactSurface(surface_mesh);
        surface_mesh->AddFacesFromBoundary(0, true, false, false);  // do not include cable and beam elements
        if (surface_mesh->GetNumTriangles() > 0)
            AddFsiMesh2D(surface_mesh);
    }
}

void ChFsiSystemSPH::AddFsiMesh1D(std::shared_ptr<fea::ChContactSurfaceSegmentSet> surface) {
    auto& fsi_mesh = m_fsi_interface->AddFsiMesh1D(surface);

    // Load index-based mesh connectivity (append to global list of 1-D flex segments)
    for (const auto& seg : surface->GetSegmentsXYZ()) {
        auto node0_index = fsi_mesh.ptr2ind_map[seg->GetNode(0)];
        auto node1_index = fsi_mesh.ptr2ind_map[seg->GetNode(1)];
        m_data_mgr->flex1D_Nodes_H.push_back(mI2(node0_index, node1_index));
    }

    // Create the BCE markers based on the mesh contact segments
    unsigned int meshID = m_fsi_interface->GetNumMeshes1D();
    auto num_bce = AddBCE_mesh1D(meshID, fsi_mesh);

    // Update total number of flex 1-D segments and associated nodes
    auto num_elements = fsi_mesh.contact_surface->GetNumSegments();
    auto num_nodes = (unsigned int)fsi_mesh.ind2ptr_map.size();
    m_num_flex1D_elements += num_elements;
    m_num_flex1D_nodes += num_nodes;

    if (m_verbose) {
        cout << "Add mesh1D" << endl;
        cout << "  Num. nodes:       " << num_nodes << endl;
        cout << "  Num. segments:    " << num_elements << endl;
        cout << "  Num. BCE markers: " << num_bce << endl;
    }
}

void ChFsiSystemSPH::AddFsiMesh2D(std::shared_ptr<fea::ChContactSurfaceMesh> surface) {
    auto& fsi_mesh = m_fsi_interface->AddFsiMesh2D(surface);

    // Load index-based mesh connectivity (append to global list of 1-D flex segments)
    for (const auto& tri : surface->GetTrianglesXYZ()) {
        auto node0_index = fsi_mesh.ptr2ind_map[tri->GetNode(0)];
        auto node1_index = fsi_mesh.ptr2ind_map[tri->GetNode(1)];
        auto node2_index = fsi_mesh.ptr2ind_map[tri->GetNode(2)];
        m_data_mgr->flex2D_Nodes_H.push_back(mI3(node0_index, node1_index, node2_index));
    }

    // Create the BCE markers based on the mesh contact surface
    unsigned int meshID = m_fsi_interface->GetNumMeshes2D();
    auto num_bce = AddBCE_mesh2D(meshID, fsi_mesh);

    // Update total number of flex 2-D faces and associated nodes
    auto num_elements = fsi_mesh.contact_surface->GetNumTriangles();
    auto num_nodes = (unsigned int)fsi_mesh.ind2ptr_map.size();
    m_num_flex2D_elements += num_elements;
    m_num_flex2D_nodes += num_nodes;

    if (m_verbose) {
        cout << "Add mesh2D" << endl;
        cout << "  Num. nodes:       " << num_nodes << endl;
        cout << "  Num. segments:    " << num_elements << endl;
        cout << "  Num. BCE markers: " << num_bce << endl;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

Real W3h_Spline(Real d, Real invh) {
    Real q = std::abs(d) * invh;
    if (q < 1)
        return (0.25f * (INVPI * invh * invh * invh) * (cube(2 - q) - 4 * cube(1 - q)));
    if (q < 2)
        return (0.25f * (INVPI * invh * invh * invh) * cube(2 - q));
    return 0;
}

void ChFsiSystemSPH::Initialize() {
    ChFsiSystem::Initialize();

    // Initialize SPH particle mass and number of neighbors
    {
        Real sum = 0;
        int count = 0;
        int IDX = 10;
        for (int i = -IDX; i <= IDX; i++) {
            for (int j = -IDX; j <= IDX; j++) {
                for (int k = -IDX; k <= IDX; k++) {
                    Real3 pos = mR3(i, j, k) * m_paramsH->INITSPACE;
                    Real W = W3h_Spline(length(pos), m_paramsH->INVHSML);
                    sum += W;
                    if (W > 0)
                        count++;
                }
            }
        }
        m_paramsH->markerMass = m_paramsH->rho0 / sum;
        m_paramsH->num_neighbors = count;
    }

    if (m_paramsH->use_default_limits) {
        m_paramsH->cMin =
            mR3(-2 * m_paramsH->boxDimX, -2 * m_paramsH->boxDimY, -2 * m_paramsH->boxDimZ) - 10 * mR3(m_paramsH->HSML);
        m_paramsH->cMax =
            mR3(+2 * m_paramsH->boxDimX, +2 * m_paramsH->boxDimY, +2 * m_paramsH->boxDimZ) + 10 * mR3(m_paramsH->HSML);
    }

    if (m_paramsH->use_init_pressure) {
        size_t numParticles = m_data_mgr->sphMarkers_H->rhoPresMuH.size();
        for (int i = 0; i < numParticles; i++) {
            double z = m_data_mgr->sphMarkers_H->posRadH[i].z;
            double p = m_paramsH->rho0 * m_paramsH->gravity.z * (z - m_paramsH->pressure_height);
            m_data_mgr->sphMarkers_H->rhoPresMuH[i].y = p;
            if (m_paramsH->elastic_SPH) {
                m_data_mgr->sphMarkers_H->tauXxYyZzH[i].x = -p;
                m_data_mgr->sphMarkers_H->tauXxYyZzH[i].y = -p;
                m_data_mgr->sphMarkers_H->tauXxYyZzH[i].z = -p;
            }
        }
    }

    // Set up subdomains for faster neighbor particle search
    m_paramsH->Apply_BC_U = false;  // You should go to CustomMath.h all the way to end of file and set your function
    int3 side0 = mI3((int)floor((m_paramsH->cMax.x - m_paramsH->cMin.x) / (RESOLUTION_LENGTH_MULT * m_paramsH->HSML)),
                     (int)floor((m_paramsH->cMax.y - m_paramsH->cMin.y) / (RESOLUTION_LENGTH_MULT * m_paramsH->HSML)),
                     (int)floor((m_paramsH->cMax.z - m_paramsH->cMin.z) / (RESOLUTION_LENGTH_MULT * m_paramsH->HSML)));
    Real3 binSize3 =
        mR3((m_paramsH->cMax.x - m_paramsH->cMin.x) / side0.x, (m_paramsH->cMax.y - m_paramsH->cMin.y) / side0.y,
            (m_paramsH->cMax.z - m_paramsH->cMin.z) / side0.z);
    m_paramsH->binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
    m_paramsH->binSize0 = binSize3.x;
    m_paramsH->boxDims = m_paramsH->cMax - m_paramsH->cMin;
    m_paramsH->straightChannelBoundaryMin = m_paramsH->cMin;  // mR3(0, 0, 0);  // 3D channel
    m_paramsH->straightChannelBoundaryMax = m_paramsH->cMax;  // SmR3(3, 2, 3) * m_paramsH->sizeScale;
    m_paramsH->deltaPress = mR3(0);
    int3 SIDE = mI3(int((m_paramsH->cMax.x - m_paramsH->cMin.x) / m_paramsH->binSize0 + .1),
                    int((m_paramsH->cMax.y - m_paramsH->cMin.y) / m_paramsH->binSize0 + .1),
                    int((m_paramsH->cMax.z - m_paramsH->cMin.z) / m_paramsH->binSize0 + .1));
    Real mBinSize = m_paramsH->binSize0;
    m_paramsH->gridSize = SIDE;
    m_paramsH->worldOrigin = m_paramsH->cMin;
    m_paramsH->cellSize = mR3(mBinSize, mBinSize, mBinSize);

    // Print information
    if (m_verbose) {
        cout << "Simulation parameters" << endl;

        cout << "  num_neighbors: " << m_paramsH->num_neighbors << endl;
        cout << "  rho0: " << m_paramsH->rho0 << endl;
        cout << "  invrho0: " << m_paramsH->invrho0 << endl;
        cout << "  mu0: " << m_paramsH->mu0 << endl;
        cout << "  bodyForce3: " << m_paramsH->bodyForce3.x << " " << m_paramsH->bodyForce3.y << " "
             << m_paramsH->bodyForce3.z << endl;
        cout << "  gravity: " << m_paramsH->gravity.x << " " << m_paramsH->gravity.y << " " << m_paramsH->gravity.z
             << endl;

        cout << "  HSML: " << m_paramsH->HSML << endl;
        cout << "  INITSPACE: " << m_paramsH->INITSPACE << endl;
        cout << "  INV_INIT: " << m_paramsH->INV_INIT << endl;
        cout << "  MULT_INITSPACE: " << m_paramsH->MULT_INITSPACE << endl;
        cout << "  NUM_BCE_LAYERS: " << m_paramsH->NUM_BCE_LAYERS << endl;
        cout << "  epsMinMarkersDis: " << m_paramsH->epsMinMarkersDis << endl;
        cout << "  markerMass: " << m_paramsH->markerMass << endl;
        cout << "  volume0: " << m_paramsH->volume0 << endl;
        cout << "  gradient_type: " << m_paramsH->gradient_type << endl;

        cout << "  v_Max: " << m_paramsH->v_Max << endl;
        cout << "  Cs: " << m_paramsH->Cs << endl;
        cout << "  EPS_XSPH: " << m_paramsH->EPS_XSPH << endl;
        cout << "  beta_shifting: " << m_paramsH->beta_shifting << endl;
        cout << "  densityReinit: " << m_paramsH->densityReinit << endl;

        cout << "  Adaptive_time_stepping: " << m_paramsH->Adaptive_time_stepping << endl;
        cout << "  Proximity search performed every " << m_paramsH->num_proximity_search_steps << " steps" << endl;
        cout << "  Co_number: " << m_paramsH->Co_number << endl;
        cout << "  dT: " << m_paramsH->dT << endl;
        cout << "  INV_dT: " << m_paramsH->INV_dT << endl;
        cout << "  dT_Max: " << m_paramsH->dT_Max << endl;
        cout << "  dT_Flex: " << m_paramsH->dT_Flex << endl;

        cout << "  non_newtonian: " << m_paramsH->non_newtonian << endl;
        cout << "  mu_of_I : " << (int)m_paramsH->mu_of_I << endl;
        cout << "  rheology_model: " << (int)m_paramsH->rheology_model << endl;
        cout << "  ave_diam: " << m_paramsH->ave_diam << endl;
        cout << "  mu_max: " << m_paramsH->mu_max << endl;
        cout << "  mu_fric_s: " << m_paramsH->mu_fric_s << endl;
        cout << "  mu_fric_2: " << m_paramsH->mu_fric_2 << endl;
        cout << "  mu_I0: " << m_paramsH->mu_I0 << endl;
        cout << "  mu_I_b: " << m_paramsH->mu_I_b << endl;
        cout << "  HB_k: " << m_paramsH->HB_k << endl;
        cout << "  HB_n: " << m_paramsH->HB_n << endl;
        cout << "  HB_tau0: " << m_paramsH->HB_tau0 << endl;
        cout << "  Coh_coeff: " << m_paramsH->Coh_coeff << endl;

        cout << "  E_young: " << m_paramsH->E_young << endl;
        cout << "  G_shear: " << m_paramsH->G_shear << endl;
        cout << "  INV_G_shear: " << m_paramsH->INV_G_shear << endl;
        cout << "  K_bulk: " << m_paramsH->K_bulk << endl;
        cout << "  C_Wi: " << m_paramsH->C_Wi << endl;

        cout << "  PPE_relaxation: " << m_paramsH->PPE_relaxation << endl;
        cout << "  Conservative_Form: " << m_paramsH->Conservative_Form << endl;
        cout << "  Pressure_Constraint: " << m_paramsH->Pressure_Constraint << endl;

        cout << "  binSize0: " << m_paramsH->binSize0 << endl;
        cout << "  boxDims: " << m_paramsH->boxDims.x << " " << m_paramsH->boxDims.y << " " << m_paramsH->boxDims.z
             << endl;
        cout << "  gridSize: " << m_paramsH->gridSize.x << " " << m_paramsH->gridSize.y << " " << m_paramsH->gridSize.z
             << endl;
        cout << "  cMin: " << m_paramsH->cMin.x << " " << m_paramsH->cMin.y << " " << m_paramsH->cMin.z << endl;
        cout << "  cMax: " << m_paramsH->cMax.x << " " << m_paramsH->cMax.y << " " << m_paramsH->cMax.z << endl;
    }

    // Initialize the underlying FSU system: set reference arrays, set counters, and resize simulation arrays
    m_data_mgr->Initialize(m_fsi_interface->m_fsi_bodies.size(),          //
                           m_num_flex1D_elements, m_num_flex2D_elements,  //
                           m_num_flex1D_nodes, m_num_flex2D_nodes);

    if (m_verbose) {
        cout << "Counters" << endl;
        cout << "  numRigidBodies:     " << m_data_mgr->countersH->numRigidBodies << endl;
        cout << "  numFlexBodies1D:    " << m_data_mgr->countersH->numFlexBodies1D << endl;
        cout << "  numFlexBodies2D:    " << m_data_mgr->countersH->numFlexBodies2D << endl;
        cout << "  numFlexNodes1D:     " << m_data_mgr->countersH->numFlexNodes1D << endl;
        cout << "  numFlexNodes2D:     " << m_data_mgr->countersH->numFlexNodes2D << endl;
        cout << "  numGhostMarkers:    " << m_data_mgr->countersH->numGhostMarkers << endl;
        cout << "  numHelperMarkers:   " << m_data_mgr->countersH->numHelperMarkers << endl;
        cout << "  numFluidMarkers:    " << m_data_mgr->countersH->numFluidMarkers << endl;
        cout << "  numBoundaryMarkers: " << m_data_mgr->countersH->numBoundaryMarkers << endl;
        cout << "  numRigidMarkers:    " << m_data_mgr->countersH->numRigidMarkers << endl;
        cout << "  numFlexMarkers1D:   " << m_data_mgr->countersH->numFlexMarkers1D << endl;
        cout << "  numFlexMarkers2D:   " << m_data_mgr->countersH->numFlexMarkers2D << endl;
        cout << "  numAllMarkers:      " << m_data_mgr->countersH->numAllMarkers << endl;
        cout << "  startRigidMarkers:  " << m_data_mgr->countersH->startRigidMarkers << endl;
        cout << "  startFlexMarkers1D: " << m_data_mgr->countersH->startFlexMarkers1D << endl;
        cout << "  startFlexMarkers2D: " << m_data_mgr->countersH->startFlexMarkers2D << endl;

        cout << "Reference array (size: " << m_data_mgr->referenceArray.size() << ")" << endl;
        for (size_t i = 0; i < m_data_mgr->referenceArray.size(); i++) {
            const int4& num = m_data_mgr->referenceArray[i];
            cout << "  " << i << ": " << num.x << " " << num.y << " " << num.z << " " << num.w << endl;
        }
        cout << "Reference array FEA (size: " << m_data_mgr->referenceArray_FEA.size() << ")" << endl;
        for (size_t i = 0; i < m_data_mgr->referenceArray_FEA.size(); i++) {
            const int4& num = m_data_mgr->referenceArray_FEA[i];
            cout << "  " << i << ": " << num.x << " " << num.y << " " << num.z << " " << num.w << endl;
        }
        cout << endl;
    }

    m_fsi_interface->LoadBodyStates();
    m_fsi_interface->LoadMeshStates();

    // Create BCE and SPH worker objects
    m_bce_mgr = chrono_types::make_unique<BceManager>(*m_data_mgr, m_verbose);
    m_fluid_dynamics = chrono_types::make_unique<ChFluidDynamics>(*m_data_mgr, *m_bce_mgr, m_verbose);

    // Initialize worker objects
    m_bce_mgr->Initialize(m_fsi_bodies_bce_num);
    m_fluid_dynamics->Initialize();

    // Mark system as initialized
    m_is_initialized = true;
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiSystemSPH::DoStepDynamics_FSI() {
    if (!m_is_initialized) {
        cout << "ERROR: FSI system not initialized!\n" << endl;
        throw std::runtime_error("FSI system not initialized!\n");
    }

    m_timer_step.reset();
    m_timer_MBS.reset();

    m_timer_step.start();

    if (m_time < 1e-6 || int(round(m_time / m_paramsH->dT)) % m_paramsH->num_proximity_search_steps == 0) {
        m_fluid_dynamics->SortParticles();
    }

    m_data_mgr->ResetData();

    switch (m_paramsH->sph_method) {
        case SPHMethod::WCSPH: {
            m_data_mgr->CopyDeviceDataToHalfStep();
            m_fluid_dynamics->IntegrateSPH(m_data_mgr->sortedSphMarkers2_D, m_data_mgr->sortedSphMarkers1_D,  //
                                           0.5 * m_paramsH->dT, m_time, true);
            m_fluid_dynamics->IntegrateSPH(m_data_mgr->sortedSphMarkers1_D, m_data_mgr->sortedSphMarkers2_D,  //
                                           1.0 * m_paramsH->dT, m_time, false);
            break;
        }

        case SPHMethod::I2SPH: {
            m_bce_mgr->updateBCEAcc();
            m_fluid_dynamics->IntegrateSPH(m_data_mgr->sortedSphMarkers2_D, m_data_mgr->sortedSphMarkers2_D,  //
                                           0.0, m_time, false);
            break;
        }
    }

    m_bce_mgr->Rigid_Forces_Torques();
    m_bce_mgr->Flex1D_Forces();
    m_bce_mgr->Flex2D_Forces();

    // Advance dynamics of the associated MBS system (if provided)
    if (m_sysMBS) {
        m_timer_MBS.start();

        m_fsi_interface->ApplyBodyForces();
        m_fsi_interface->ApplyMeshForces();

        if (m_paramsH->dT_Flex == 0)
            m_paramsH->dT_Flex = m_paramsH->dT;
        int sync = int(m_paramsH->dT / m_paramsH->dT_Flex);
        if (sync < 1)
            sync = 1;
        for (int t = 0; t < sync; t++) {
            m_sysMBS->DoStepDynamics(m_paramsH->dT / sync);
        }
        m_timer_MBS.stop();
    }

    m_fsi_interface->LoadBodyStates();
    m_bce_mgr->UpdateBodyMarkerState();

    m_fsi_interface->LoadMeshStates();
    m_bce_mgr->UpdateMeshMarker1DState();
    m_bce_mgr->UpdateMeshMarker2DState();

    m_fluid_dynamics->CopySortedToOriginal(m_data_mgr->sortedSphMarkers2_D, m_data_mgr->sphMarkers_D);

    m_time += m_paramsH->dT;

    m_timer_step.stop();
    m_RTF = m_timer_step() / m_paramsH->dT;
    if (m_sysMBS)
        m_ratio_MBS = m_timer_MBS() / m_timer_step();
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiSystemSPH::WriteParticleFile(const std::string& outfilename) const {
    if (m_write_mode == OutputMode::CSV) {
        WriteCsvParticlesToFile(m_data_mgr->sphMarkers_D->posRadD, m_data_mgr->sphMarkers_D->velMasD,
                                m_data_mgr->sphMarkers_D->rhoPresMuD, m_data_mgr->referenceArray, outfilename);
    } else if (m_write_mode == OutputMode::CHPF) {
        WriteChPFParticlesToFile(m_data_mgr->sphMarkers_D->posRadD, m_data_mgr->referenceArray, outfilename);
    }
}

void ChFsiSystemSPH::PrintParticleToFile(const std::string& dir) const {
    sph::PrintParticleToFile(m_data_mgr->sphMarkers_D->posRadD, m_data_mgr->sphMarkers_D->velMasD,
                             m_data_mgr->sphMarkers_D->rhoPresMuD, m_data_mgr->sr_tau_I_mu_i_Original,
                             m_data_mgr->derivVelRhoOriginalD, m_data_mgr->referenceArray,
                             m_data_mgr->referenceArray_FEA, dir, m_paramsH);
}

void ChFsiSystemSPH::PrintFsiInfoToFile(const std::string& dir, double time) const {
    sph::PrintFsiInfoToFile(                                                                       //
        m_data_mgr->fsiBodyState_D->pos, m_data_mgr->fsiBodyState_D->rot,                          //
        m_data_mgr->fsiBodyState_D->lin_vel,                                                       //
        m_data_mgr->fsiMesh1DState_D->pos_fsi_fea_D, m_data_mgr->fsiMesh2DState_D->pos_fsi_fea_D,  //
        m_data_mgr->fsiMesh1DState_D->vel_fsi_fea_D, m_data_mgr->fsiMesh2DState_D->vel_fsi_fea_D,  //
        m_data_mgr->rigid_FSI_ForcesD, m_data_mgr->rigid_FSI_TorquesD,                             //
        m_data_mgr->flex1D_FSIforces_D, m_data_mgr->flex2D_FSIforces_D,                            //
        dir, time);
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiSystemSPH::AddSPHParticle(const ChVector3d& pos,
                                    double rho,
                                    double pres,
                                    double mu,
                                    const ChVector3d& vel,
                                    const ChVector3d& tauXxYyZz,
                                    const ChVector3d& tauXyXzYz) {
    m_data_mgr->AddSphParticle(ToReal3(pos), rho, pres, mu, ToReal3(vel), ToReal3(tauXxYyZz), ToReal3(tauXyXzYz));
}

void ChFsiSystemSPH::AddSPHParticle(const ChVector3d& pos,
                                    const ChVector3d& vel,
                                    const ChVector3d& tauXxYyZz,
                                    const ChVector3d& tauXyXzYz) {
    AddSPHParticle(pos, m_paramsH->rho0, m_paramsH->BASEPRES, m_paramsH->mu0, vel, tauXxYyZz, tauXyXzYz);
}

void ChFsiSystemSPH::AddBoxSPH(const ChVector3d& boxCenter, const ChVector3d& boxHalfDim) {
    // Use a chrono sampler to create a bucket of points
    utils::ChGridSampler<> sampler(m_paramsH->INITSPACE);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        AddSPHParticle(points[i], m_paramsH->rho0, 0, m_paramsH->mu0,
                       ChVector3d(0),   // initial velocity
                       ChVector3d(0),   // tauxxyyzz
                       ChVector3d(0));  // tauxyxzyz
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiSystemSPH::AddWallBCE(std::shared_ptr<ChBody> body, const ChFrame<>& frame, const ChVector2d& size) {
    std::vector<ChVector3d> points;
    CreateBCE_wall(size, points);
    AddPointsBCE(body, points, frame, false);
}

void ChFsiSystemSPH::AddBoxContainerBCE(std::shared_ptr<ChBody> body,
                                        const ChFrame<>& frame,
                                        const ChVector3d& size,
                                        const ChVector3i faces) {
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    Real buffer = 2 * (m_paramsH->NUM_BCE_LAYERS - 1) * spacing;

    ChVector3d hsize = size / 2;

    ChVector3d xn(-hsize.x(), 0, 0);
    ChVector3d xp(+hsize.x(), 0, 0);
    ChVector3d yn(0, -hsize.y(), 0);
    ChVector3d yp(0, +hsize.y(), 0);
    ChVector3d zn(0, 0, -hsize.z());
    ChVector3d zp(0, 0, +hsize.z());

    // Z- wall
    if (faces.z() == -1 || faces.z() == 2)
        AddWallBCE(body, frame * ChFrame<>(zn, QUNIT), {size.x(), size.y()});
    // Z+ wall
    if (faces.z() == +1 || faces.z() == 2)
        AddWallBCE(body, frame * ChFrame<>(zp, QuatFromAngleX(CH_PI)), {size.x(), size.y()});

    // X- wall
    if (faces.x() == -1 || faces.x() == 2)
        AddWallBCE(body, frame * ChFrame<>(xn, QuatFromAngleY(+CH_PI_2)), {size.z() + buffer, size.y()});
    // X+ wall
    if (faces.x() == +1 || faces.x() == 2)
        AddWallBCE(body, frame * ChFrame<>(xp, QuatFromAngleY(-CH_PI_2)), {size.z() + buffer, size.y()});

    // Y- wall
    if (faces.y() == -1 || faces.y() == 2)
        AddWallBCE(body, frame * ChFrame<>(yn, QuatFromAngleX(-CH_PI_2)), {size.x() + buffer, size.z() + buffer});
    // Y+ wall
    if (faces.y() == +1 || faces.y() == 2)
        AddWallBCE(body, frame * ChFrame<>(yp, QuatFromAngleX(+CH_PI_2)), {size.x() + buffer, size.z() + buffer});
}

size_t ChFsiSystemSPH::AddBoxBCE(std::shared_ptr<ChBody> body,
                                 const ChFrame<>& frame,
                                 const ChVector3d& size,
                                 bool solid) {
    std::vector<ChVector3d> points;
    CreateBCE_box(size, solid, points);
    return AddPointsBCE(body, points, frame, solid);
}

size_t ChFsiSystemSPH::AddSphereBCE(std::shared_ptr<ChBody> body,
                                    const ChFrame<>& frame,
                                    double radius,
                                    bool solid,
                                    bool polar) {
    std::vector<ChVector3d> points;
    CreateBCE_sphere(radius, solid, polar, points);
    return AddPointsBCE(body, points, frame, solid);
}

size_t ChFsiSystemSPH::AddCylinderBCE(std::shared_ptr<ChBody> body,
                                      const ChFrame<>& frame,
                                      double radius,
                                      double height,
                                      bool solid,
                                      bool capped,
                                      bool polar) {
    std::vector<ChVector3d> points;
    CreateBCE_cylinder(radius, height, solid, capped, polar, points);
    return AddPointsBCE(body, points, frame, solid);
}

void ChFsiSystemSPH::CreateBCE_On_Wheel_Grouser_Hollow(std::vector<ChVector3d>& posRadBCE,
                                                       double wheel_rad,
                                                       double wheel_w,
                                                       double gro_h,
                                                       double gro_w,
                                                       int gro_num,
                                                       double marker_spacing,
                                                       bool cartesian) {
    double spacing = marker_spacing * m_paramsH->MULT_INITSPACE;
    int num_layers = (int)std::floor(1.00001 * wheel_w / spacing) + 1;
    double rim_thickness = 3 * spacing;  // 3 layers thick rim

    for (size_t si = 0; si < num_layers; si++) {
        double s = -0.5 * wheel_w + spacing * si;
        if (cartesian) {
            for (double x = -wheel_rad; x <= wheel_rad; x += spacing) {
                for (double y = -wheel_rad; y <= wheel_rad; y += spacing) {
                    double r = std::sqrt(x * x + y * y);
                    if (wheel_rad - rim_thickness <= r && r <= wheel_rad)
                        posRadBCE.push_back(ChVector3d(x, s, y));
                }
            }
        } else {
            // wheel rim
            int numr = (int)std::floor(1.00001 * rim_thickness / spacing);
            for (size_t ir = 0; ir < numr; ir++) {
                double r = wheel_rad - rim_thickness + (ir + 1) * spacing;
                int numTheta = (int)std::floor(2 * CH_PI * r / spacing);
                for (size_t t = 0; t < numTheta; t++) {
                    double teta = t * 2 * CH_PI / numTheta;
                    ChVector3d BCE_Pos_local = {r * std::cos(teta), s, r * std::sin(teta)};
                    posRadBCE.push_back(BCE_Pos_local);
                }
            }

            // grouser
            int numr_g = std::max(1, (int)std::floor(1.00001 * gro_h / spacing));
            int numw_g = std::max(1, (int)std::floor(1.00001 * gro_w / spacing));
            for (size_t ir_g = 0; ir_g < numr_g; ir_g++) {
                double r = 1. * spacing + ir_g * spacing + wheel_rad;
                for (size_t t = 0; t < gro_num; t++) {
                    for (size_t iw_g = 0; iw_g < numw_g; iw_g++) {
                        double teta = t * 2 * 3.1415 / gro_num + iw_g * spacing / wheel_rad;
                        ChVector3d BCE_Pos_local = {r * std::cos(teta), s, r * std::sin(teta)};
                        posRadBCE.push_back(BCE_Pos_local);
                    }
                }
            }
        }
    }
}

void ChFsiSystemSPH::CreateBCE_On_Wheel_Grouser(std::vector<ChVector3d>& posRadBCE,
                                                double wheel_rad,
                                                double wheel_w,
                                                double gro_h,
                                                double gro_w,
                                                int gro_num,
                                                double kernel_h,
                                                bool cartesian) {
    Real spacing = kernel_h * m_paramsH->MULT_INITSPACE;
    int num_layers = (int)std::floor(1.00001 * wheel_w / spacing) + 1;
    for (size_t si = 0; si < num_layers; si++) {
        Real s = -0.5 * wheel_w + spacing * si;
        if (cartesian)
            for (Real x = -wheel_rad; x <= wheel_rad; x += spacing) {
                for (Real y = -wheel_rad; y <= wheel_rad; y += spacing) {
                    if (x * x + y * y <= wheel_rad * wheel_rad)
                        posRadBCE.push_back(ChVector3d(x, s, y));
                }
            }
        else {
            ChVector3d centerPointLF = {0, s, 0};
            posRadBCE.push_back(ChVector3d(0, s, 0));
            // wheel
            int numr = (int)std::floor(1.00001 * wheel_rad / spacing);
            for (size_t ir = 0; ir < numr; ir++) {
                Real r = spacing + ir * spacing;
                int numTheta = (int)std::floor(2 * 3.1415 * r / spacing);
                for (size_t t = 0; t < numTheta; t++) {
                    double teta = t * 2 * 3.1415 / numTheta;
                    ChVector3d BCE_Pos_local = {r * cos(teta), 0, r * sin(teta)};
                    BCE_Pos_local += centerPointLF;
                    posRadBCE.push_back(BCE_Pos_local);
                }
            }
            // grouser
            int numr_g = (int)std::floor(1.00001 * gro_h / spacing);
            int numw_g = (int)std::floor(1.00001 * gro_w / spacing) + 1;
            for (size_t ir_g = 0; ir_g < numr_g; ir_g++) {
                Real r = 0.5 * spacing + ir_g * spacing + wheel_rad;
                for (size_t t = 0; t < gro_num; t++) {
                    for (size_t iw_g = 0; iw_g < numw_g; iw_g++) {
                        Real teta = t * 2 * 3.1415 / gro_num + iw_g * spacing / wheel_rad;
                        ChVector3d BCE_Pos_local = {r * cos(teta), 0, r * sin(teta)};
                        BCE_Pos_local += centerPointLF;
                        posRadBCE.push_back(BCE_Pos_local);
                    }
                }
            }
        }
    }
}

void ChFsiSystemSPH::AddWheelBCE_Grouser(std::shared_ptr<ChBody> body,
                                         const ChFrame<>& frame,
                                         double radius,
                                         double wide,
                                         double grouser_height,
                                         double grouser_wide,
                                         int grouser_num,
                                         double kernel_h,
                                         bool cartesian,
                                         bool hollow) {
    std::vector<ChVector3d> bce;
    if (hollow) {
        CreateBCE_On_Wheel_Grouser_Hollow(bce, radius, wide, grouser_height, grouser_wide, grouser_num, kernel_h,
                                          cartesian);
    } else {
        CreateBCE_On_Wheel_Grouser(bce, radius, wide, grouser_height, grouser_wide, grouser_num, kernel_h, cartesian);
    }
    AddPointsBCE(body, bce, frame, true);
}
size_t ChFsiSystemSPH::AddCylinderAnnulusBCE(std::shared_ptr<ChBody> body,
                                             const ChFrame<>& frame,
                                             double radius_inner,
                                             double radius_outer,
                                             double height,
                                             bool polar) {
    auto delta = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    std::vector<ChVector3d> points;
    CreateCylinderAnnulusPoints(radius_inner, radius_outer, height, polar, delta, points);
    return AddPointsBCE(body, points, frame, true);
}

size_t ChFsiSystemSPH::AddConeBCE(std::shared_ptr<ChBody> body,
                                  const ChFrame<>& frame,
                                  double radius,
                                  double height,
                                  bool solid,
                                  bool capped,
                                  bool polar) {
    std::vector<ChVector3d> points;
    CreateBCE_cone(radius, height, solid, capped, polar, points);
    return AddPointsBCE(body, points, frame, true);
}

size_t ChFsiSystemSPH::AddPointsBCE(std::shared_ptr<ChBody> body,
                                    const std::vector<ChVector3d>& points,
                                    const ChFrame<>& rel_frame,
                                    bool solid) {
    // Set BCE marker type
    MarkerType type = solid ? MarkerType::BCE_RIGID : MarkerType::BCE_WALL;

    for (const auto& p : points) {
        auto pos_body = rel_frame.TransformPointLocalToParent(p);
        auto pos_abs = body->GetFrameRefToAbs().TransformPointLocalToParent(pos_body);
        auto vel_abs = body->GetFrameRefToAbs().PointSpeedLocalToParent(pos_body);

        m_data_mgr->AddBceMarker(type, ToReal3(pos_abs), ToReal3(vel_abs));
    }

    if (solid)
        m_fsi_bodies_bce_num.push_back((int)points.size());

    return points.size();
}

//--------------------------------------------------------------------------------------------------------------------------------

const Real pi = Real(CH_PI);

void ChFsiSystemSPH::CreateBCE_wall(const ChVector2d& size, std::vector<ChVector3d>& bce) {
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BCE_LAYERS;

    // Calculate actual spacing in x-y directions
    ChVector2d hsize = size / 2;
    int2 np = {(int)std::round(hsize.x() / spacing), (int)std::round(hsize.y() / spacing)};
    ChVector2d delta = {hsize.x() / np.x, hsize.y() / np.y};

    for (int il = 0; il < num_layers; il++) {
        for (int ix = -np.x; ix <= np.x; ix++) {
            for (int iy = -np.y; iy <= np.y; iy++) {
                bce.push_back({ix * delta.x(), iy * delta.y(), -il * spacing});
            }
        }
    }
}

void ChFsiSystemSPH::CreateBCE_box(const ChVector3d& size, bool solid, std::vector<ChVector3d>& bce) {
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BCE_LAYERS;

    // Calculate actual spacing in all 3 directions
    ChVector3d hsize = size / 2;
    int3 np = {(int)std::round(hsize.x() / spacing), (int)std::round(hsize.y() / spacing),
               (int)std::round(hsize.z() / spacing)};
    ChVector3d delta = {hsize.x() / np.x, hsize.y() / np.y, hsize.z() / np.z};

    // Inflate box if boundary
    if (!solid) {
        np += num_layers - 1;
        hsize = hsize + (num_layers - 1.0) * delta;
    }

    for (int il = 0; il < num_layers; il++) {
        // faces in Z direction
        for (int ix = -np.x; ix <= np.x; ix++) {
            for (int iy = -np.y; iy <= np.y; iy++) {
                bce.push_back({ix * delta.x(), iy * delta.y(), -hsize.z() + il * delta.z()});
                bce.push_back({ix * delta.x(), iy * delta.y(), +hsize.z() - il * delta.z()});
            }
        }

        // faces in Y direction
        for (int ix = -np.x; ix <= np.x; ix++) {
            for (int iz = -np.z + num_layers; iz <= np.z - num_layers; iz++) {
                bce.push_back({ix * delta.x(), -hsize.y() + il * delta.y(), iz * delta.z()});
                bce.push_back({ix * delta.x(), +hsize.y() - il * delta.y(), iz * delta.z()});
            }
        }

        // faces in X direction
        for (int iy = -np.y + num_layers; iy <= np.y - num_layers; iy++) {
            for (int iz = -np.z + num_layers; iz <= np.z - num_layers; iz++) {
                bce.push_back({-hsize.x() + il * delta.x(), iy * delta.y(), iz * delta.z()});
                bce.push_back({+hsize.x() - il * delta.x(), iy * delta.y(), iz * delta.z()});
            }
        }
    }
}

void ChFsiSystemSPH::CreateBCE_sphere(double rad, bool solid, bool polar, std::vector<ChVector3d>& bce) {
    double spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BCE_LAYERS;

    // Use polar coordinates
    if (polar) {
        double rad_out = solid ? rad : rad + num_layers * spacing;
        double rad_in = rad_out - num_layers * spacing;
        int np_r = (int)std::round((rad - rad_in) / spacing);
        double delta_r = (rad_out - rad_in) / np_r;

        for (int ir = 0; ir <= np_r; ir++) {
            double r = rad_in + ir * delta_r;
            int np_phi = (int)std::round(pi * r / spacing);
            double delta_phi = pi / np_phi;
            for (int ip = 0; ip < np_phi; ip++) {
                double phi = ip * delta_phi;
                double cphi = std::cos(phi);
                double sphi = std::sin(phi);
                double x = r * sphi;
                double y = r * sphi;
                double z = r * cphi;
                int np_th = (int)std::round(2 * pi * r * sphi / spacing);
                double delta_th = (np_th > 0) ? (2 * pi) / np_th : 1;
                for (int it = 0; it < np_th; it++) {
                    double theta = it * delta_th;
                    bce.push_back({x * std::cos(theta), y * std::sin(theta), z});
                }
            }
        }
        return;
    }

    // Use a regular grid and accept/reject points
    int np = (int)std::round(rad / spacing);
    double delta = rad / np;
    if (!solid) {
        np += num_layers;
        rad += num_layers * delta;
    }

    for (int iz = 0; iz <= np; iz++) {
        double z = iz * delta;
        double rz_max = std::sqrt(rad * rad - z * z);
        double rz_min = std::max(rz_max - num_layers * delta, 0.0);
        if (iz >= np - num_layers)
            rz_min = 0;
        double rz_min2 = rz_min * rz_min;
        double rz_max2 = rz_max * rz_max;
        int nq = (int)std::round(rz_max / spacing);
        for (int ix = -nq; ix <= nq; ix++) {
            double x = ix * delta;
            for (int iy = -nq; iy <= nq; iy++) {
                double y = iy * delta;
                double r2 = x * x + y * y;
                if (r2 >= rz_min2 && r2 <= rz_max2) {
                    bce.push_back({x, y, +z});
                    bce.push_back({x, y, -z});
                }
            }
        }
    }
}

void ChFsiSystemSPH::CreateBCE_cylinder(double rad,
                                        double height,
                                        bool solid,
                                        bool capped,
                                        bool polar,
                                        std::vector<ChVector3d>& bce) {
    double spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BCE_LAYERS;

    // Calculate actual spacing
    double hheight = height / 2;
    int np_h = (int)std::round(hheight / spacing);
    double delta_h = hheight / np_h;

    // Inflate cylinder if boundary
    if (!solid && capped) {
        np_h += num_layers;
        hheight += num_layers * delta_h;
    }

    // Use polar coordinates
    if (polar) {
        double rad_max = solid ? rad : rad + num_layers * spacing;
        double rad_min = rad_max - num_layers * spacing;
        int np_r = (int)std::round((rad_max - rad_min) / spacing);
        double delta_r = (rad_max - rad_min) / np_r;

        for (int ir = 0; ir <= np_r; ir++) {
            double r = rad_min + ir * delta_r;
            int np_th = (int)std::round(2 * pi * r / spacing);
            double delta_th = (np_th > 0) ? (2 * pi) / np_th : 1;
            for (int it = 0; it < np_th; it++) {
                double theta = it * delta_th;
                double x = r * cos(theta);
                double y = r * sin(theta);
                for (int iz = -np_h; iz <= np_h; iz++) {
                    double z = iz * delta_h;
                    bce.push_back({x, y, z});
                }
            }
        }

        if (capped) {
            np_r = (int)std::round(rad_max / spacing);
            delta_r = rad_max / np_r;

            for (int ir = 0; ir <= np_r; ir++) {
                double r = rad_max - ir * delta_r;
                int np_th = std::max((int)std::round(2 * pi * r / spacing), 1);
                double delta_th = (2 * pi) / np_th;
                for (int it = 0; it < np_th; it++) {
                    double theta = it * delta_th;
                    double x = r * cos(theta);
                    double y = r * sin(theta);
                    for (int iz = 0; iz <= num_layers; iz++) {
                        double z = hheight - iz * delta_h;
                        bce.push_back({x, y, -z});
                        bce.push_back({x, y, +z});
                    }
                }
            }
        }

        return;
    }

    // Use a regular grid and accept/reject points
    int np_r = (int)std::round(rad / spacing);
    double delta_r = rad / np_r;
    if (!solid) {
        np_r += num_layers;
        rad += num_layers * delta_r;
    }

    double rad_max = rad;
    double rad_min = std::max(rad - num_layers * delta_r, 0.0);
    double r_max2 = rad_max * rad_max;
    double r_min2 = rad_min * rad_min;
    for (int ix = -np_r; ix <= np_r; ix++) {
        double x = ix * delta_r;
        for (int iy = -np_r; iy <= np_r; iy++) {
            double y = iy * delta_r;
            double r2 = x * x + y * y;
            if (r2 >= r_min2 && r2 <= r_max2) {
                for (int iz = -np_h; iz <= np_h; iz++) {
                    double z = iz * delta_h;
                    bce.push_back({x, y, z});
                }
            }
            if (capped && r2 < r_min2) {
                for (int iz = 0; iz <= num_layers; iz++) {
                    double z = hheight - iz * delta_h;
                    bce.push_back({x, y, -z});
                    bce.push_back({x, y, +z});
                }
            }
        }
    }
}

void ChFsiSystemSPH::CreateBCE_cone(double rad,
                                    double height,
                                    bool solid,
                                    bool capped,
                                    bool polar,
                                    std::vector<ChVector3d>& bce) {
    double spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BCE_LAYERS;

    // Calculate actual spacing
    int np_h = (int)std::round(height / spacing);
    double delta_h = height / np_h;

    // Inflate cone if boundary
    if (!solid) {
        np_h += num_layers;
        height += num_layers * delta_h;
        if (capped) {
            np_h += num_layers;
            height += num_layers * delta_h;
        }
    }

    // Use polar coordinates
    if (polar) {
        for (int iz = 0; iz < np_h; iz++) {
            double z = iz * delta_h;
            double rz = rad * (height - z) / height;
            double rad_out = solid ? rz : rz + num_layers * spacing;
            double rad_in = std::max(rad_out - num_layers * spacing, 0.0);
            if (iz >= np_h - num_layers)
                rad_in = 0;
            int np_r = (int)std::round((rad_out - rad_in) / spacing);
            double delta_r = (rad_out - rad_in) / np_r;
            for (int ir = 0; ir <= np_r; ir++) {
                double r = rad_in + ir * delta_r;
                int np_th = (int)std::round(2 * pi * r / spacing);
                double delta_th = (2 * pi) / np_th;
                for (int it = 0; it < np_th; it++) {
                    double theta = it * delta_th;
                    double x = r * cos(theta);
                    double y = r * sin(theta);
                    bce.push_back({x, y, z});
                }
            }
        }

        bce.push_back({0.0, 0.0, height});

        if (capped) {
            //// RADU TODO
        }

        return;
    }

    // Use a regular grid and accept/reject points
    int np_r = (int)std::round(rad / spacing);
    double delta_r = rad / np_r;

    for (int iz = 0; iz <= np_h; iz++) {
        double z = iz * delta_h;
        double rz = rad * (height - z) / height;
        double rad_out = solid ? rz : rz + num_layers * spacing;
        double rad_in = std::max(rad_out - num_layers * spacing, 0.0);
        double r_out2 = rad_out * rad_out;
        double r_in2 = rad_in * rad_in;
        for (int ix = -np_r; ix <= np_r; ix++) {
            double x = ix * delta_r;
            for (int iy = -np_r; iy <= np_r; iy++) {
                double y = iy * delta_r;
                double r2 = x * x + y * y;
                if (r2 >= r_in2 && r2 <= r_out2) {
                    bce.push_back({x, y, z});
                }
            }

            if (capped) {
                //// RADU TODO
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

unsigned int ChFsiSystemSPH::AddBCE_mesh1D(unsigned int meshID, const ChFsiInterface::FsiMesh1D& fsi_mesh) {
    const auto& surface = fsi_mesh.contact_surface;

    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BCE_LAYERS;

    // Traverse the contact segments:
    // - calculate their discretization number n
    //   (largest number that results in a discretization no coarser than the initial spacing)
    // - generate segment coordinates for a uniform grid over the segment
    // - generate locations of BCE points on segment
    unsigned int num_seg = (unsigned int)surface->GetSegmentsXYZ().size();
    unsigned int num_bce = 0;
    for (unsigned int segID = 0; segID < num_seg; segID++) {
        const auto& seg = surface->GetSegmentsXYZ()[segID];

        const auto& P0 = seg->GetNode(0)->GetPos();  // vertex 0 position (absolute coordinates)
        const auto& P1 = seg->GetNode(1)->GetPos();  // vertex 1 position (absolute coordinates)

        const auto& V0 = seg->GetNode(0)->GetPosDt();  // vertex 0 velocity (absolute coordinates)
        const auto& V1 = seg->GetNode(1)->GetPosDt();  // vertex 1 velocity (absolute coordinates)

        auto x_dir = P1 - P0;       // segment direction
        auto len = x_dir.Length();  // segment direction
        x_dir /= len;               // normalized direction

        int n = (int)std::ceil(len / spacing);  // required divisions on segment

        // Create two directions orthogonal to 'x_dir'
        ChVector3<> y_dir(-x_dir.y() - x_dir.z(), x_dir.x() - x_dir.z(), x_dir.x() + x_dir.y());
        y_dir.Normalize();
        ChVector3<> z_dir = Vcross(x_dir, y_dir);

        unsigned int n_bce = 0;  // number of BCE markers on segment
        for (int i = 0; i <= n; i++) {
            if (i == 0 && !seg->OwnsNode(0))  // segment does not own vertex 0
                continue;
            if (i == n && !seg->OwnsNode(1))  // segment does not own vertex 1
                continue;

            auto lambda = ChVector2<>(n - i, i) / n;

            auto P = P0 * lambda[0] + P1 * lambda[1];
            auto V = V0 * lambda[0] + V1 * lambda[1];

            for (int j = -num_layers + 1; j <= num_layers - 1; j += 2) {
                for (int k = -num_layers + 1; k <= num_layers - 1; k += 2) {
                    if (m_remove_center1D && j == 0 && k == 0)
                        continue;
                    if (m_pattern1D == BcePatternMesh1D::STAR && std::abs(j) + std::abs(k) > num_layers)
                        continue;
                    double y_val = j * spacing / 2;
                    double z_val = k * spacing / 2;
                    auto Q = P + y_val * y_dir + z_val * z_dir;

                    m_data_mgr->AddBceMarker(MarkerType::BCE_FLEX1D, ToReal3(Q), ToReal3(V));

                    m_data_mgr->flex1D_BCEcoords_H.push_back(ToReal3({lambda[0], y_val, z_val}));
                    m_data_mgr->flex1D_BCEsolids_H.push_back(mU3(meshID, segID, m_num_flex1D_elements + segID));
                    n_bce++;
                }
            }
        }

        // Add the number of BCE markers for this segment
        num_bce += n_bce;
    }

    return num_bce;
}

unsigned int ChFsiSystemSPH::AddBCE_mesh2D(unsigned int meshID, const ChFsiInterface::FsiMesh2D& fsi_mesh) {
    const auto& surface = fsi_mesh.contact_surface;

    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BCE_LAYERS;

    ////std::ofstream ofile("mesh2D.txt");
    ////ofile << mesh->GetNumTriangles() << endl;
    ////ofile << endl;

    // Traverse the contact surface faces:
    // - calculate their discretization number n
    //   (largest number that results in a discretization no coarser than the initial spacing on each edge)
    // - generate barycentric coordinates for a uniform grid over the triangular face
    // - generate locations of BCE points on triangular face
    unsigned int num_tri = (int)surface->GetTrianglesXYZ().size();
    unsigned int num_bce = 0;
    for (unsigned int triID = 0; triID < num_tri; triID++) {
        const auto& tri = surface->GetTrianglesXYZ()[triID];

        const auto& P0 = tri->GetNode(0)->GetPos();  // vertex 0 position (absolute coordinates)
        const auto& P1 = tri->GetNode(1)->GetPos();  // vertex 1 position (absolute coordinates)
        const auto& P2 = tri->GetNode(2)->GetPos();  // vertex 2 position (absolute coordinates)

        const auto& V0 = tri->GetNode(0)->GetPosDt();  // vertex 0 velocity (absolute coordinates)
        const auto& V1 = tri->GetNode(1)->GetPosDt();  // vertex 1 velocity (absolute coordinates)
        const auto& V2 = tri->GetNode(2)->GetPosDt();  // vertex 2 velocity (absolute coordinates)

        auto normal = Vcross(P1 - P0, P2 - P1);  // triangle normal
        normal.Normalize();

        int n0 = (int)std::ceil((P2 - P1).Length() / spacing);  // required divisions on edge 0
        int n1 = (int)std::ceil((P0 - P2).Length() / spacing);  // required divisions on edge 1
        int n2 = (int)std::ceil((P1 - P0).Length() / spacing);  // required divisions on edge 2

        int n_median = max(min(n0, n1), min(max(n0, n1), n2));  // number of divisions on each edge (median)
        ////int n_max = std::max(n0, std::max(n1, n2));             // number of divisions on each edge (max)

        ////cout << "(" << n0 << " " << n1 << " " << n2 << ")";
        ////cout << "  Median : " << n_median << " Max : " << n_max << endl;

        int n = n_median;

        ////ofile << P0 << endl;
        ////ofile << P1 << endl;
        ////ofile << P2 << endl;
        ////ofile << tri->OwnsNode(0) << " " << tri->OwnsNode(1) << " " << tri->OwnsNode(2) << endl;
        ////ofile << tri->OwnsEdge(0) << " " << tri->OwnsEdge(1) << " " << tri->OwnsEdge(2) << endl;
        ////ofile << n << endl;

        bool remove_center = m_remove_center2D;
        int m_start = 0;
        int m_end = 0;
        switch (m_pattern2D) {
            case BcePatternMesh2D::INWARD:
                m_start = -2 * (num_layers - 1);
                m_end = 0;
                remove_center = false;
                break;
            case BcePatternMesh2D::CENTERED:
                m_start = -(num_layers - 1);
                m_end = +(num_layers - 1);
                break;
            case BcePatternMesh2D::OUTWARD:
                m_start = 0;
                m_end = +2 * (num_layers - 1);
                remove_center = false;
                break;
        }

        ////double z_start = centered ? (num_layers - 1) * spacing / 2 : 0;  // start layer z (along normal)

        unsigned int n_bce = 0;  // number of BCE markers on triangle
        for (int i = 0; i <= n; i++) {
            if (i == n && !tri->OwnsNode(0))  // triangle does not own vertex v0
                continue;
            if (i == 0 && !tri->OwnsEdge(1))  // triangle does not own edge v1-v2 = e1
                continue;

            for (int j = 0; j <= n - i; j++) {
                int k = n - i - j;
                auto lambda = ChVector3<>(i, j, k) / n;  // barycentric coordinates of BCE marker

                if (j == n && !tri->OwnsNode(1))  // triangle does not own vertex v1
                    continue;
                if (j == 0 && !tri->OwnsEdge(2))  // triangle does not own edge v2-v0 = e2
                    continue;

                if (k == n && !tri->OwnsNode(2))  // triangle does not own vertex v2
                    continue;
                if (k == 0 && !tri->OwnsEdge(0))  // triangle does not own edge v0-v1 = e0
                    continue;

                auto P = lambda[0] * P0 + lambda[1] * P1 + lambda[2] * P2;  // absolute coordinates of BCE marker
                auto V = lambda[0] * V0 + lambda[1] * V1 + lambda[2] * V2;  // absolute velocity of BCE marker

                // Create layers in normal direction
                for (int m = m_start; m <= m_end; m += 2) {
                    if (remove_center && m == 0)
                        continue;
                    double z_val = m * spacing / 2;
                    auto Q = P + z_val * normal;

                    m_data_mgr->AddBceMarker(MarkerType::BCE_FLEX2D, ToReal3(Q), ToReal3(V));

                    m_data_mgr->flex2D_BCEcoords_H.push_back(ToReal3({lambda[0], lambda[1], z_val}));
                    m_data_mgr->flex2D_BCEsolids_H.push_back(mU3(meshID, triID, m_num_flex2D_elements + triID));
                    n_bce++;

                    ////ofile << Q << endl;
                }
            }
        }

        ////ofile << n_bce << endl;
        ////ofile << endl;

        // Add the number of BCE markers for this triangle
        num_bce += n_bce;
    }

    ////ofile.close();

    return num_bce;
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiSystemSPH::CreateCylinderAnnulusPoints(double rad_inner,
                                                 double rad_outer,
                                                 double height,
                                                 bool polar,
                                                 double delta,
                                                 std::vector<ChVector3d>& points) {
    // Calculate actual spacing
    double hheight = height / 2;
    int np_h = (int)std::round(hheight / delta);
    double delta_h = hheight / np_h;

    // Use polar coordinates
    if (polar) {
        int np_r = (int)std::round((rad_outer - rad_inner) / delta);
        double delta_r = (rad_outer - rad_inner) / np_r;
        for (int ir = 0; ir <= np_r; ir++) {
            double r = rad_inner + ir * delta_r;
            int np_th = (int)std::round(2 * pi * r / delta);
            double delta_th = (2 * pi) / np_th;
            for (int it = 0; it < np_th; it++) {
                double theta = it * delta_th;
                double x = r * cos(theta);
                double y = r * sin(theta);
                for (int iz = -np_h; iz <= np_h; iz++) {
                    double z = iz * delta_h;
                    points.push_back({x, y, z});
                }
            }
        }
        return;
    }

    // Use a regular grid and accept/reject points
    int np_r = (int)std::round(rad_outer / delta);
    double delta_r = rad_outer / np_r;

    double r_in2 = rad_inner * rad_inner;
    double r_out2 = rad_outer * rad_outer;
    for (int ix = -np_r; ix <= np_r; ix++) {
        double x = ix * delta_r;
        for (int iy = -np_r; iy <= np_r; iy++) {
            double y = iy * delta_r;
            double r2 = x * x + y * y;
            if (r2 >= r_in2 && r2 <= r_out2) {
                for (int iz = -np_h; iz <= np_h; iz++) {
                    double z = iz * delta_h;
                    points.push_back({x, y, z});
                }
            }
        }
    }
}

void ChFsiSystemSPH::CreateMeshPoints(ChTriangleMeshConnected& mesh, double delta, std::vector<ChVector3d>& points) {
    mesh.RepairDuplicateVertexes(1e-9);  // if meshes are not watertight
    auto bbox = mesh.GetBoundingBox();

    const double EPSI = 1e-6;

    ChVector3d ray_origin;
    for (double x = bbox.min.x(); x < bbox.max.x(); x += delta) {
        ray_origin.x() = x + 1e-9;
        for (double y = bbox.min.y(); y < bbox.max.y(); y += delta) {
            ray_origin.y() = y + 1e-9;
            for (double z = bbox.min.z(); z < bbox.max.z(); z += delta) {
                ray_origin.z() = z + 1e-9;

                ChVector3d ray_dir[2] = {ChVector3d(5, 0.5, 0.25), ChVector3d(-3, 0.7, 10)};
                int intersectCounter[2] = {0, 0};

                for (unsigned int i = 0; i < mesh.m_face_v_indices.size(); ++i) {
                    auto& t_face = mesh.m_face_v_indices[i];
                    auto& v1 = mesh.m_vertices[t_face.x()];
                    auto& v2 = mesh.m_vertices[t_face.y()];
                    auto& v3 = mesh.m_vertices[t_face.z()];

                    // Find vectors for two edges sharing V1
                    auto edge1 = v2 - v1;
                    auto edge2 = v3 - v1;

                    int t_inter[2] = {0, 0};

                    for (unsigned int j = 0; j < 2; j++) {
                        // Begin calculating determinant - also used to calculate uu parameter
                        auto pvec = Vcross(ray_dir[j], edge2);
                        // if determinant is near zero, ray is parallel to plane of triangle
                        double det = Vdot(edge1, pvec);
                        // NOT CULLING
                        if (det > -EPSI && det < EPSI) {
                            t_inter[j] = 0;
                            continue;
                        }
                        double inv_det = 1.0 / det;

                        // calculate distance from V1 to ray origin
                        auto tvec = ray_origin - v1;

                        /// Calculate uu parameter and test bound
                        double uu = Vdot(tvec, pvec) * inv_det;
                        // The intersection lies outside of the triangle
                        if (uu < 0.0 || uu > 1.0) {
                            t_inter[j] = 0;
                            continue;
                        }

                        // Prepare to test vv parameter
                        auto qvec = Vcross(tvec, edge1);

                        // Calculate vv parameter and test bound
                        double vv = Vdot(ray_dir[j], qvec) * inv_det;
                        // The intersection lies outside of the triangle
                        if (vv < 0.0 || ((uu + vv) > 1.0)) {
                            t_inter[j] = 0;
                            continue;
                        }

                        double tt = Vdot(edge2, qvec) * inv_det;
                        if (tt > EPSI) {  /// ray intersection
                            t_inter[j] = 1;
                            continue;
                        }

                        // No hit, no win
                        t_inter[j] = 0;
                    }

                    intersectCounter[0] += t_inter[0];
                    intersectCounter[1] += t_inter[1];
                }

                if (((intersectCounter[0] % 2) == 1) && ((intersectCounter[1] % 2) == 1))  // inside mesh
                    points.push_back(ChVector3d(x, y, z));
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

double ChFsiSystemSPH::GetKernelLength() const {
    return m_paramsH->HSML;
}

double ChFsiSystemSPH::GetInitialSpacing() const {
    return m_paramsH->INITSPACE;
}

int ChFsiSystemSPH::GetNumBCELayers() const {
    return m_paramsH->NUM_BCE_LAYERS;
}

ChVector3d ChFsiSystemSPH::GetContainerDim() const {
    return ChVector3d(m_paramsH->boxDimX, m_paramsH->boxDimY, m_paramsH->boxDimZ);
}

double ChFsiSystemSPH::GetDensity() const {
    return m_paramsH->rho0;
}

double ChFsiSystemSPH::GetViscosity() const {
    return m_paramsH->mu0;
}

double ChFsiSystemSPH::GetBasePressure() const {
    return m_paramsH->BASEPRES;
}

double ChFsiSystemSPH::GetParticleMass() const {
    return m_paramsH->markerMass;
}

ChVector3d ChFsiSystemSPH::GetGravitationalAcceleration() const {
    return ChVector3d(m_paramsH->gravity.x, m_paramsH->gravity.y, m_paramsH->gravity.z);
}

double ChFsiSystemSPH::GetSoundSpeed() const {
    return m_paramsH->Cs;
}

ChVector3d ChFsiSystemSPH::GetBodyForce() const {
    return ChVector3d(m_paramsH->bodyForce3.x, m_paramsH->bodyForce3.y, m_paramsH->bodyForce3.z);
}

double ChFsiSystemSPH::GetStepSize() const {
    return m_paramsH->dT;
}

double ChFsiSystemSPH::GetMaxStepSize() const {
    return m_paramsH->dT_Max;
}

bool ChFsiSystemSPH::GetAdaptiveTimeStepping() const {
    return m_paramsH->Adaptive_time_stepping;
}

int ChFsiSystemSPH::GetNumProximitySearchSteps() const {
    return m_paramsH->num_proximity_search_steps;
}

size_t ChFsiSystemSPH::GetNumFluidMarkers() const {
    return m_data_mgr->countersH->numFluidMarkers;
}

size_t ChFsiSystemSPH::GetNumRigidBodyMarkers() const {
    return m_data_mgr->countersH->numRigidMarkers;
}

size_t ChFsiSystemSPH::GetNumFlexBodyMarkers() const {
    return m_data_mgr->countersH->numFlexMarkers1D + m_data_mgr->countersH->numFlexMarkers2D;
}

size_t ChFsiSystemSPH::GetNumBoundaryMarkers() const {
    return m_data_mgr->countersH->numBoundaryMarkers;
}

//--------------------------------------------------------------------------------------------------------------------------------

std::vector<ChVector3d> ChFsiSystemSPH::GetParticlePositions() const {
    thrust::host_vector<Real4> posRadH = m_data_mgr->sphMarkers_D->posRadD;
    std::vector<ChVector3d> pos;

    for (size_t i = 0; i < posRadH.size(); i++) {
        pos.push_back(ToChVector(posRadH[i]));
    }
    return pos;
}

std::vector<ChVector3d> ChFsiSystemSPH::GetParticleFluidProperties() const {
    thrust::host_vector<Real4> rhoPresMuH = m_data_mgr->sphMarkers_D->rhoPresMuD;
    std::vector<ChVector3d> props;

    for (size_t i = 0; i < rhoPresMuH.size(); i++) {
        props.push_back(ToChVector(rhoPresMuH[i]));
    }
    return props;
}

std::vector<ChVector3d> ChFsiSystemSPH::GetParticleVelocities() const {
    thrust::host_vector<Real3> velH = m_data_mgr->sphMarkers_D->velMasD;
    std::vector<ChVector3d> vel;

    for (size_t i = 0; i < velH.size(); i++) {
        vel.push_back(ToChVector(velH[i]));
    }
    return vel;
}

std::vector<ChVector3d> ChFsiSystemSPH::GetParticleAccelerations() const {
    thrust::host_vector<Real4> accH = m_data_mgr->GetParticleAccelerations();
    std::vector<ChVector3d> acc;
    for (size_t i = 0; i < accH.size(); i++) {
        acc.push_back(ToChVector(accH[i]));
    }
    return acc;
}

std::vector<ChVector3d> ChFsiSystemSPH::GetParticleForces() const {
    thrust::host_vector<Real4> frcH = m_data_mgr->GetParticleForces();
    std::vector<ChVector3d> frc;
    for (size_t i = 0; i < frcH.size(); i++) {
        frc.push_back(ToChVector(frcH[i]));
    }
    return frc;
}

//--------------------------------------------------------------------------------------------------------------------------------

thrust::device_vector<int> ChFsiSystemSPH::FindParticlesInBox(const ChFrame<>& frame, const ChVector3d& size) {
    const ChVector3d& Pos = frame.GetPos();
    ChVector3d Ax = frame.GetRotMat().GetAxisX();
    ChVector3d Ay = frame.GetRotMat().GetAxisY();
    ChVector3d Az = frame.GetRotMat().GetAxisZ();

    auto hsize = 0.5 * mR3(size.x(), size.y(), size.z());
    auto pos = mR3(Pos.x(), Pos.y(), Pos.z());
    auto ax = mR3(Ax.x(), Ax.y(), Ax.z());
    auto ay = mR3(Ay.x(), Ay.y(), Ay.z());
    auto az = mR3(Az.x(), Az.y(), Az.z());

    return m_data_mgr->FindParticlesInBox(hsize, pos, ax, ay, az);
}

thrust::device_vector<Real4> ChFsiSystemSPH::GetParticlePositions(const thrust::device_vector<int>& indices) {
    return m_data_mgr->GetParticlePositions(indices);
}

thrust::device_vector<Real3> ChFsiSystemSPH::GetParticleVelocities(const thrust::device_vector<int>& indices) {
    return m_data_mgr->GetParticleVelocities(indices);
}

thrust::device_vector<Real4> ChFsiSystemSPH::GetParticleForces(const thrust::device_vector<int>& indices) {
    return m_data_mgr->GetParticleForces(indices);
}

thrust::device_vector<Real4> ChFsiSystemSPH::GetParticleAccelerations(const thrust::device_vector<int>& indices) {
    return m_data_mgr->GetParticleAccelerations(indices);
}

}  // end namespace fsi
}  // end namespace chrono
