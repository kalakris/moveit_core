/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Sachin Chitta
*********************************************************************/

#ifndef KINEMATICS_CACHE_H_
#define KINEMATICS_CACHE_H_

#include <kinematics_base/kinematics_base.h>
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>

#include <geometry_msgs/Point.h>
#include <boost/shared_ptr.hpp>

namespace kinematics_cache
{
  std::vector<double> kinematics_cache_map_;

  class KinematicsCache 
  {
    public:

    struct Options
    {
      unsigned int num_prefix;
      geometry_msgs::Point origin, workspace_size;
      double resolution;
      unsigned int max_solutions_per_grid_location;      
    };
               
    /** @class
     *  @brief An implementation of a cache for fast kinematics lookups. initialize needs to be called on this class before it can be used.
     */
    KinematicsCache();

    /** @brief Generate the cache map spending timeout (seconds) on the generation process)
     *  @param timeout Time (in seconds) to be spent on generating the cache
     *  @return True if cache map generation was successful
     */
    bool generateCacheMap(double timeout);

    /** @brief Get a candidate solution for a particular pose. Note that the pose will be projected onto the grid used in the caching process.
     *  @param pose The desired pose
     *  @param solution_index The solution index
     *  @param solution The actual solution
     *  @return False if desired pose lies outside the grid
     */
    bool getSolution(const geometry_msgs::Pose &pose, unsigned int solution_index, std::vector<double> &solution) const;
    
    /** @brief Get all candidate solutions for a particular pose. Note that the pose will be projected onto the grid used in the caching process. The size of the solutions vector MUST be pre-allocated to num_solutions (see getNumSolutions)
     *  @param pose The desired pose
     *  @param solution_index The solution index
     *  @param solutions The actual set of solutions
     *  @return False if desired pose lies outside the grid or if size(solutions) is not set to num_solutions (see getNumSolutions) 
     */
    bool getSolutions(const geometry_msgs::Pose &pose, std::vector<std::vector<double> > &solutions) const;

    /** @brief Get number of candidate solutions for a particular pose.
     *  @param pose The desired pose
     *  @param solution_index The solution index
     *  @param solution The actual solution
     *  @return False if desired pose lies outside the grid
     */
    bool getNumSolutions(const geometry_msgs::Pose &pose, unsigned int &num_solutions) const;
   
    /** @brief Initialize the cache class
     *  @param solver An instance of a kinematics solver
     *  @param kinematic_model An instance of a kinematic model
     *  @param opt Parameters needed for defining the cache workspace
     *  @return False if any error occured during initialization
     */
    bool initialize(kinematics::KinematicsBaseConstPtr &solver,
                    planning_models::KinematicModelConstPtr &kinematic_model,
                    const KinematicsCache::Options &opt);    

    /** @brief Return the instance of the kinematics solver */
    kinematics::KinematicsBaseConstPtr& getSolverInstance(void)
    {
      return kinematics_solver_;
    }
    
    /** @brief Return the instance of the kinematics model */
    planning_models::KinematicModelConstPtr& getModelInstance(void)
    {
      return kinematic_model_;
    }
    
    /** @brief Return the cache parameters used for cache construction */
    const Options& getCacheParameters(void)
    {
      return options_;
    }

  private:    

    /** @brief Get the location of a solution given the grid index and solution index */
    unsigned int getSolutionLocation(unsigned int &grid_index, unsigned int &solution_index) const;
    
    /** @brief Get a solution for the grid index given the solution index */
    std::vector<double>& getSolution(unsigned int &grid_index, unsigned int solution_index) const;
    
    /** @brief Add a new solution to the cache at the given location */
    bool addToCache(const geometry_msgs::Pose &pose, const std::vector<double> &joint_values);    
        
    /** @brief Get the grid index for a given pose */
    bool getGridIndex(const geometry_msgs::Pose &pose, unsigned int &grid_index) const;    
    
    KinematicsCache::Options options_; /** Internal copy of cache parameters */
    geometry_msgs::Point cache_origin_;/** Origin for cache workspace */
    double cache_resolution_; /** Resolution for grid (in m) */
    unsigned int cache_size_x_, cache_size_y_, cache_size_z_; /** Number of elements in XYZ grid */
    unsigned int max_solutions_per_grid_location_; /** Max number of solutions per grid location */
    unsigned int solution_dimension_; /** Dimension of each solution */
    unsigned int size_grid_node_; /** Size of storage location (in number of doubles) for each grid node */
    unsigned int kinematics_cache_points_with_solution_; /** Number of grid points that have solutions */
    unsigned int kinematics_cache_size_; /** Total number of grid points */

    std::vector<double> kinematics_cache_vector_; /** Storage for the solutions */
    std::vector<unsigned int> num_solutions_vector_; /** Storage for number of solutions for each grid location */
    
    kinematics::KinematicsBaseConstPtr kinematics_solver_;/** An instance of the kinematics solver */
        
    planning_models::KinematicModelConstPtr kinematic_model_; /** An instance of the kinematic model */
    planning_models::KinematicStatePtr kinematic_state_; /** An instance of the kinematic state */
    
    const planning_models::KinematicModel::JointModelGroup* joint_model_group_; /** Joint model group associated with this cache */
    boost::shared_ptr<planning_models::KinematicState::JointStateGroup> joint_state_group_; /** Joint state corresponding to cache */
    
    mutable std::vector<double> solution_local_; /** Local pre-allocated storage */
  };
    
}

#endif