#ifndef ICP_HPP
#define ICP_HPP

#include <Eigen/Dense>
#include "pcr/prelude.hpp"
#include "pcr/core/point_cloud.hpp"
#include "pcr/spatial/kd_tree.hpp"

/**
 * @namespace pcr::registration
 * @brief Interface for registering point clouds
 */
namespace pcr::registration {

/**
 * @brief Parameters for the ICP algorithm
 */
struct IcpParams {
  std::size_t max_iterations = 50; ///< Max iterations to before stopping fit
  pcr::coord_t tolerance = 1e-4; ///< Tolerance for convergence
  std::size_t k = 1;
  ///< Number of nearest neighbors to use when finding correspondences
};

/**
 * @brief Result of the ICP algorithm
 */
struct IcpResult {
  pcr::coord_t final_error = 0; ///< Final error of the registration
  std::size_t iterations = 0; ///< Number of iterations executed
  bool converged = false; ///< Whether the algorithm converged
  pcr::transform_t transform;
  ///< Final transformation matrix for the Rigid Body Transform
};

/**
 * @brief Iterative Closest Point (ICP) algorithm for registering point clouds
 */
class ICP {

  /**
   * @brief Constructor for the ICP class
   */
  ICP();

  /**
   * @brief Aligns two point clouds using the ICP algorithm
   * @param source The source point cloud
   * @param target The target point cloud
   * @param icp_params The ICP parameters
   * @return The result of the ICP algorithm
   */
  IcpResult align(const pcr::core::PointCloud &source,
                  const pcr::core::PointCloud &target,
                  const IcpParams &icp_params);
};

}


#endif
