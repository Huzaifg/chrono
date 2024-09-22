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
// Author:Arman Pazouki, Milad Rakhsha, Wei Hu
// =============================================================================
// This file contains miscellaneous macros and utilities used in the SPH code.
// =============================================================================

#ifndef CH_SPH_GENERAL_CUH
#define CH_SPH_GENERAL_CUH

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <device_launch_parameters.h>

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/sph/utils/ChUtilsDevice.cuh"
#include "chrono_fsi/sph/physics/FsiDataManager.cuh"
#include "chrono_fsi/sph/math/ChFsiLinearSolver.h"
#include "chrono_fsi/sph/math/CustomMath.h"

namespace chrono {
namespace fsi {
namespace sph {

//--------------------------------------------------------------------------------------------------------------------------------

// Declared as const variables static in order to be able to use them in a different translation units in the utils
__constant__ static SimParams paramsD;
__constant__ static Counters countersD;

void CopyParametersToDevice(std::shared_ptr<SimParams> paramsH, std::shared_ptr<Counters> countersH);

//--------------------------------------------------------------------------------------------------------------------------------

/// Short define of the kernel function
#define W3h W3h_Spline

/// Short define of the kernel function gradient
#define GradWh GradWh_Spline

//--------------------------------------------------------------------------------------------------------------------------------
// Cubic Spline SPH kernel function
// d > 0 is the distance between 2 particles. h is the sph kernel length

__device__ inline Real W3h_Spline(Real d) {
    Real invh = paramsD.INVHSML;
    Real q = fabs(d) * invh;
    if (q < 1) {
        return (0.25f * (INVPI * cube(invh)) * (cube(2 - q) - 4 * cube(1 - q)));
    }
    if (q < 2) {
        return (0.25f * (INVPI * cube(invh)) * cube(2 - q));
    }
    return 0;
}

__device__ inline Real3 GradWh_Spline(Real3 d) {
    Real invh = paramsD.INVHSML;
    Real q = length(d) * invh;
    if (abs(q) < EPSILON)
        return mR3(0.0);
    bool less1 = (q < 1);
    bool less2 = (q < 2);
    return (less1 * (3 * q - 4.0f) + less2 * (!less1) * (-q + 4.0f - 4.0f / q)) * .75f * INVPI * quintic(invh) * d;
}

//--------------------------------------------------------------------------------------------------------------------------------
// Johnson kernel 1996b
// d > 0 is the distance between 2 particles. h is the sph kernel length

__device__ inline Real W3h_High(Real d) {
    Real invh = paramsD.INVHSML;
    Real q = fabs(d) * invh;
    if (q < 2) {
        return (1.25f * (INVPI * cube(invh)) * (0.1875f * square(q) - 0.75f * q + 0.75f));
    }
    return 0;
}

__device__ inline Real3 GradWh_High(Real3 d, Real h) {
    Real invh = paramsD.INVHSML;
    Real q = length(d) * invh;
    if (abs(q) < EPSILON)
        return mR3(0.0);
    bool less2 = (q < 2);
    return (3.0 / 8.0 * q - 3.0 / 4.0) * 5.0 / 4.0 / q * INVPI * (1.0 / quintic(h)) * d * less2;
}

//--------------------------------------------------------------------------------------------------------------------------------
// Quintic Spline SPH kernel function
// d > 0 is the distance between 2 particles. h is the sph kernel length

__device__ inline Real W3h_Quintic(Real d) {
    Real invh = paramsD.INVHSML;
    Real q = fabs(d) * invh;
    Real coeff = 8.35655e-3;  // 3/359
    if (q < 1) {
        return (coeff * INVPI * cube(invh) * (quintic(3 - q) - 6 * quintic(2 - q) + 15 * quintic(1 - q)));
    }
    if (q < 2) {
        return (coeff * INVPI * cube(invh) * (quintic(3 - q) - 6 * quintic(2 - q)));
    }
    if (q < 3) {
        return (coeff * INVPI * cube(invh) * (quintic(3 - q)));
    }
    return 0;
}

__device__ inline Real3 GradW3h_Quintic(Real3 d) {
    Real invh = paramsD.INVHSML;
    Real q = length(d) * invh;
    if (fabs(q) < 1e-10)
        return mR3(0.0);
    Real coeff = -4.178273e-2;  // -15/359
    if (q < 1) {
        return (coeff * (INVPI * quintic(invh) / q) * d * (quartic(3 - q) - 6 * quartic(2 - q) + 15 * quartic(1 - q)));
    }
    if (q < 2) {
        return (coeff * (INVPI * quintic(invh) / q) * d * (quartic(3 - q) - 6 * quartic(2 - q)));
    }
    if (q < 3) {
        return (coeff * (INVPI * quintic(invh) / q) * d * (quartic(3 - q)));
    }
    return mR3(0);
}

//--------------------------------------------------------------------------------------------------------------------------------

// Fluid equation of state
__device__ inline Real Eos(Real rho, Real type) {
    // if (rho < paramsD.rho0) //
    //     rho = paramsD.rho0; //
    // Real gama = 7;
    // Real B = 100 * paramsD.rho0 * paramsD.v_Max * paramsD.v_Max / gama;
    // return B * (pow(rho / paramsD.rho0, gama) - 1) + paramsD.BASEPRES; //
    return paramsD.Cs * paramsD.Cs * (rho - paramsD.rho0);  //
}

// Inverse of equation of state
__device__ inline Real InvEos(Real pw) {
    Real rho = pw / (paramsD.Cs * paramsD.Cs) + paramsD.rho0;  //
    return rho;
}

//--------------------------------------------------------------------------------------------------------------------------------

// FerrariCi
__device__ inline Real FerrariCi(Real rho) {
    int gama = 7;
    Real B = 100 * paramsD.rho0 * paramsD.v_Max * paramsD.v_Max / gama;
    return sqrt(gama * B / paramsD.rho0) * pow(rho / paramsD.rho0, 0.5 * (gama - 1));
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline Real3 Modify_Local_PosB(Real3& b, Real3 a) {
    Real3 dist3 = a - b;
    b.x += ((dist3.x > 0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x : 0);
    b.x -= ((dist3.x < -0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x : 0);

    b.y += ((dist3.y > 0.5f * paramsD.boxDims.y) ? paramsD.boxDims.y : 0);
    b.y -= ((dist3.y < -0.5f * paramsD.boxDims.y) ? paramsD.boxDims.y : 0);

    b.z += ((dist3.z > 0.5f * paramsD.boxDims.z) ? paramsD.boxDims.z : 0);
    b.z -= ((dist3.z < -0.5f * paramsD.boxDims.z) ? paramsD.boxDims.z : 0);

    dist3 = a - b;
    // modifying the markers perfect overlap
    Real dd = dist3.x * dist3.x + dist3.y * dist3.y + dist3.z * dist3.z;
    Real MinD = paramsD.epsMinMarkersDis * paramsD.HSML;
    Real sq_MinD = MinD * MinD;
    if (dd < sq_MinD) {
        dist3 = mR3(MinD, 0, 0);
    }
    b = a - dist3;
    return (dist3);
}

__device__ inline Real3 Distance(Real3 a, Real3 b) {
    return Modify_Local_PosB(b, a);
}

//--------------------------------------------------------------------------------------------------------------------------------
// first comp of q is rotation, last 3 components are axis of rot

__device__ inline void RotationMatirixFromQuaternion(Real3& AD1, Real3& AD2, Real3& AD3, const Real4& q) {
    AD1 = 2 * mR3(0.5f - q.z * q.z - q.w * q.w, q.y * q.z - q.x * q.w, q.y * q.w + q.x * q.z);
    AD2 = 2 * mR3(q.y * q.z + q.x * q.w, 0.5f - q.y * q.y - q.w * q.w, q.z * q.w - q.x * q.y);
    AD3 = 2 * mR3(q.y * q.w - q.x * q.z, q.z * q.w + q.x * q.y, 0.5f - q.y * q.y - q.z * q.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline Real3 InverseRotate_By_RotationMatrix_DeviceHost(const Real3& A1,
                                                                   const Real3& A2,
                                                                   const Real3& A3,
                                                                   const Real3& r3) {
    return mR3(A1.x * r3.x + A2.x * r3.y + A3.x * r3.z, A1.y * r3.x + A2.y * r3.y + A3.y * r3.z,
               A1.z * r3.x + A2.z * r3.y + A3.z * r3.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline int3 calcGridPos(Real3 p) {
    int3 gridPos;
    if (paramsD.cellSize.x * paramsD.cellSize.y * paramsD.cellSize.z == 0)
        printf("calcGridPos=%f,%f,%f\n", paramsD.cellSize.x, paramsD.cellSize.y, paramsD.cellSize.z);

    gridPos.x = (int)floor((p.x - paramsD.worldOrigin.x) / (paramsD.cellSize.x));
    gridPos.y = (int)floor((p.y - paramsD.worldOrigin.y) / (paramsD.cellSize.y));
    gridPos.z = (int)floor((p.z - paramsD.worldOrigin.z) / (paramsD.cellSize.z));
    return gridPos;
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline uint calcGridHash(int3 gridPos) {
    gridPos.x -= ((gridPos.x >= paramsD.gridSize.x) ? paramsD.gridSize.x : 0);
    gridPos.y -= ((gridPos.y >= paramsD.gridSize.y) ? paramsD.gridSize.y : 0);
    gridPos.z -= ((gridPos.z >= paramsD.gridSize.z) ? paramsD.gridSize.z : 0);

    gridPos.x += ((gridPos.x < 0) ? paramsD.gridSize.x : 0);
    gridPos.y += ((gridPos.y < 0) ? paramsD.gridSize.y : 0);
    gridPos.z += ((gridPos.z < 0) ? paramsD.gridSize.z : 0);

    return gridPos.z * paramsD.gridSize.y * paramsD.gridSize.x + gridPos.y * paramsD.gridSize.x + gridPos.x;
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline uint calcCellID(uint3 cellPos) {
    if (cellPos.x < paramsD.gridSize.x && cellPos.y < paramsD.gridSize.y && cellPos.z < paramsD.gridSize.z) {
        return cellPos.z * paramsD.gridSize.x * paramsD.gridSize.y + cellPos.y * paramsD.gridSize.x + cellPos.x;
    } else {
        printf("shouldn't be here\n");
        return paramsD.gridSize.x * paramsD.gridSize.y * paramsD.gridSize.z;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline uint getCellPos(int trialCellPos, uint ub) {
    if (trialCellPos >= 0 && trialCellPos < ub) {
        return (uint)trialCellPos;
    } else if (trialCellPos < 0) {
        return (uint)(trialCellPos + ub);
    } else {
        return (uint)(trialCellPos - ub);
    }
    return (uint)trialCellPos;
}

//--------------------------------------------------------------------------------------------------------------------------------
inline __device__ uint getCenterCellID(const uint* numPartsInCenterCells, const uint threadID) {
    uint offsets[9] = {0};
    for (int i = 0; i < 8; ++i) {
        offsets[i + 1] = numPartsInCenterCells[i];
    }
    uint left = 0;
    uint right = 8;
    while (left < right) {
        uint mid = (left + right) / 2;
        if (offsets[mid] < threadID) {
            left = mid + 1;
        } else if (offsets[mid] > threadID) {
            right = mid;
        } else {
            return mid;
        }
    }
    return left - 1;
}

// -------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real Strain_Rate(Real3 grad_ux, Real3 grad_uy, Real3 grad_uz) {
    grad_ux.y = (grad_uy.x + grad_ux.y) * 0.5;
    grad_ux.z = (grad_uz.x + grad_ux.z) * 0.5;

    grad_uy.x = grad_ux.y;
    grad_uy.z = (grad_uy.z + grad_uz.y) * 0.5;

    grad_uz.x = grad_ux.z;
    grad_uz.y = grad_uy.z;

    return sqrt(                                    //
        0.5 * (length(grad_ux) * length(grad_ux) +  //
               length(grad_uy) * length(grad_uy) +  //
               length(grad_uz) * length(grad_uz))   //
    );
}

// -------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real Tensor_Norm(Real* T) {
    return sqrt(                                          //
        0.5 * (T[0] * T[0] + T[1] * T[1] + T[2] * T[2] +  //
               T[3] * T[3] + T[4] * T[4] + T[5] * T[5] +  //
               T[6] * T[6] + T[7] * T[7] + T[8] * T[8])   //
    );
}
// -------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real Sym_Tensor_Norm(Real3 xx_yy_zz, Real3 xy_xz_yz) {
    return sqrt(0.5 * (xx_yy_zz.x * xx_yy_zz.x + xx_yy_zz.y * xx_yy_zz.y + xx_yy_zz.z * xx_yy_zz.z +
                       2 * xy_xz_yz.x * xy_xz_yz.x + 2 * xy_xz_yz.y * xy_xz_yz.y + 2 * xy_xz_yz.z * xy_xz_yz.z));
}
// -------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real Inertia_num(Real Strain_rate, Real rho, Real p, Real diam) {
    Real I = Strain_rate * diam * sqrt(rho / rmaxr(p, EPSILON));
    return rminr(1e3, I);
}

// -------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real mu_I(Real Strain_rate, Real I) {
    Real mu = 0;
    if (paramsD.mu_of_I == FrictionLaw::CONSTANT)
        mu = paramsD.mu_fric_s;
    else if (paramsD.mu_of_I == FrictionLaw::NONLINEAR)
        mu = paramsD.mu_fric_s + paramsD.mu_I_b * I;
    else
        mu = paramsD.mu_fric_s + (paramsD.mu_fric_2 - paramsD.mu_fric_s) * (I / (paramsD.mu_I0 + I));

    return mu;
}
// -------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real mu_eff(Real Strain_rate, Real p, Real mu_I) {
    return rmaxr(mu_I * rmaxr(p, 0.0) / Strain_rate, paramsD.mu_max);
}

// -------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real Herschel_Bulkley_stress(Real Strain_rate, Real k, Real n, Real tau0) {
    Real tau = tau0 + k * pow(Strain_rate, n);
    return tau;
}
// -------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real Herschel_Bulkley_mu_eff(Real Strain_rate, Real k, Real n, Real tau0) {
    Real mu_eff = tau0 / Strain_rate + k * pow(Strain_rate, n - 1);
    return rminr(mu_eff, paramsD.mu_max);
}
// -------------------------------------------------------------------------------------------------------------------------------
__global__ void calc_A_tensor(Real* A_tensor,
                              Real* G_tensor,
                              Real4* sortedPosRad,
                              Real4* sortedRhoPreMu,
                              Real* sumWij_inv,
                              uint* cellStart,
                              uint* cellEnd,
                              volatile bool* error_flag);
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calc_L_tensor(Real* A_tensor,
                              Real* L_tensor,
                              Real* G_tensor,
                              Real4* sortedPosRad,
                              Real4* sortedRhoPreMu,
                              Real* sumWij_inv,
                              uint* cellStart,
                              uint* cellEnd,
                              volatile bool* error_flag);

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calcRho_kernel(Real4* sortedPosRad,  // input: sorted positionsmin(
                               Real4* sortedRhoPreMu,
                               Real* sumWij_inv,
                               uint* cellStart,
                               uint* cellEnd,
                               uint* mynumContact,
                               volatile bool* error_flag);

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calcNormalizedRho_kernel(Real4* sortedPosRad,  // input: sorted positions
                                         Real3* sortedVelMas,
                                         Real4* sortedRhoPreMu,
                                         Real* sumWij_inv,
                                         Real* G_i,
                                         Real3* normals,
                                         Real* Color,
                                         uint* cellStart,
                                         uint* cellEnd,
                                         volatile bool* error_flag);
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calcNormalizedRho_Gi_fillInMatrixIndices(Real4* sortedPosRad,  // input: sorted positions
                                                         Real3* sortedVelMas,
                                                         Real4* sortedRhoPreMu,
                                                         Real* sumWij_inv,
                                                         Real* G_i,
                                                         Real3* normals,
                                                         uint* csrColInd,
                                                         uint* numContacts,
                                                         uint* cellStart,
                                                         uint* cellEnd,
                                                         volatile bool* error_flag);
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Function_Gradient_Laplacian_Operator(Real4* sortedPosRad,  // input: sorted positions
                                                     Real3* sortedVelMas,
                                                     Real4* sortedRhoPreMu,
                                                     Real* sumWij_inv,
                                                     Real* G_tensor,
                                                     Real* L_tensor,
                                                     Real* A_L,   // velocity Laplacian matrix;
                                                     Real3* A_G,  // This is a matrix in a way that A*p gives the gradp
                                                     Real* A_f,
                                                     uint* csrColInd,
                                                     uint* numContacts,
                                                     volatile bool* error_flag);
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Jacobi_SOR_Iter(Real4* sortedRhoPreMu,
                                Real* A_Matrix,
                                Real3* V_old,
                                Real3* V_new,
                                Real3* b3vec,
                                Real* q_old,  // q=p^(n+1)-p^n
                                Real* q_new,  // q=p^(n+1)-p^n
                                Real* b1vec,
                                const uint* csrColInd,
                                const uint* numContacts,
                                bool _3dvector,
                                volatile bool* error_flag);
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Update_AND_Calc_Res(Real4* sortedRhoPreMu,
                                    Real3* V_old,
                                    Real3* V_new,
                                    Real* q_old,
                                    Real* q_new,
                                    Real* Residuals,
                                    bool _3dvector,
                                    volatile bool* error_flag);
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Initialize_Variables(Real4* sortedRhoPreMu,
                                     Real* p_old,
                                     Real3* sortedVelMas,
                                     Real3* V_new,
                                     volatile bool* error_flag);

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void UpdateDensity(Real3* vis_vel,
                              Real3* XSPH_Vel,
                              Real3* new_vel,       // Write
                              Real4* sortedPosRad,  // Read
                              Real4* sortedRhoPreMu,
                              Real* sumWij_inv,
                              uint* cellStart,
                              uint* cellEnd,
                              volatile bool* error_flag);

__global__ void neighborSearchNum(const Real4* sortedPosRad,
                                  const Real4* sortedRhoPreMu,
                                  const uint* cellStart,
                                  const uint* cellEnd,
                                  const uint* activityIdentifierD,
                                  uint* numNeighborsPerPart,
                                  volatile bool* error_flag);

__global__ void neighborSearchID(const Real4* sortedPosRad,
                                 const Real4* sortedRhoPreMu,
                                 const uint* cellStart,
                                 const uint* cellEnd,
                                 const uint* activityIdentifierD,
                                 const uint* numNeighborsPerPart,
                                 uint* neighborList,
                                 volatile bool* error_flag);

}  // namespace sph
}  // namespace fsi
}  // namespace chrono

#endif