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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu
// =============================================================================
//
// Base class for processing boundary condition enforcing (BCE) particles forces
// in FSI system.
//
// =============================================================================

#ifndef CH_BCE_CUH_
#define CH_BCE_CUH_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChFsiBase.h"
#include "chrono_fsi/physics/FsiDataManager.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// @brief  Base class for processing boundary condition enforcing (BCE) particle forces in an FSI system.
///
/// This class handles the Fluid-Solid Interaction by enforcing i) forces from the fluid/granular dynamics
/// system to the MBD system, and ii) displacement from the MBD system to the fluid dynamics system.
class ChBce : public ChFsiBase {
  public:
    /// Modified velocity information for BCE particles.
    thrust::device_vector<Real3> velMas_ModifiedBCE;

    /// Modified density, pressure information for BCE particles.
    thrust::device_vector<Real4> rhoPreMu_ModifiedBCE;

    /// Modified stress tensor for BCE particles, diagonal entries.
    thrust::device_vector<Real3> tauXxYyZz_ModifiedBCE;

    /// Modified stress tensor for BCE particles， non-diagonal entries.
    thrust::device_vector<Real3> tauXyXzYz_ModifiedBCE;

    /// Constructor of the ChBce class
    ChBce(FsiDataManager& data_mgr,                ///< FSI data
          std::shared_ptr<SimParams> paramsH,      ///< simulation parameters
          std::shared_ptr<ChCounters> numObjects,  ///< number of sph particles on each phase
          bool verbose                             ///< verbose terminal output
    );

    /// Destructor of the ChBce class
    ~ChBce();

    /// Updates the position and velocity of the particles on the rigid bodies based on the state of the body.
    void UpdateBodyMarkerState();

    /// Updates the position and velocity of the particles on the flexible solids based on the state of the mesh.
    void UpdateMeshMarker1DState();
    void UpdateMeshMarker2DState();

    /// Updates the position and velocity of the particles on the rigid bodies based on the state of the body.
    void UpdateBodyMarkerStateInitial();

    /// Updates the position and velocity of the particles on the flexible solids based on the state of the mesh.
    void UpdateMeshMarker1DStateInitial();
    void UpdateMeshMarker2DStateInitial();

    /// Calculates the forces from the fluid/granular dynamics system to the FSI system on rigid bodies.
    void Rigid_Forces_Torques();

    /// Calculates the forces from the fluid/granular dynamics system to the FSI system on flexible bodies.
    void Flex1D_Forces();
    void Flex2D_Forces();

    void updateBCEAcc();

    /// Populates the BCE particles on the rigid bodies at the initial configuration of the system.
    /// The local coordinates w.r.t to the coordinate system of the rigid bodies is saved and is used
    /// during the update stage. In such a condition the position and orientation of the body is
    /// enough to update the position of all the particles attached to it.
    void Populate_RigidSPH_MeshPos_LRF(std::vector<int> fsiBodyBceNum);

    /// Complete construction of the BCE at the intial configuration of the system.
    void Initialize(std::vector<int> fsiBodyBceNum);

  private:
    FsiDataManager& m_data_mgr;  ///< FSI data manager

    thrust::device_vector<Real3> m_totalForceRigid;       ///< Total forces from fluid to bodies
    thrust::device_vector<Real3> m_totalTorqueRigid;      ///< Total torques from fluid to bodies

    bool m_verbose;

    // Calculate accelerations of solid BCE markers based on the information of the ChSystem.
    void CalcRigidBceAcceleration();
    void CalcFlex1DBceAcceleration();
    void CalcFlex2DBceAcceleration();
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
