/*********************************************************************
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

/* Author: E. Gil Jones */

#ifndef MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_WORLD_DISTANCE_FIELD_
#define MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_WORLD_DISTANCE_FIELD_

#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_distance_field/collision_distance_field_types.h>
#include <moveit/collision_distance_field/collision_robot_distance_field.h>

namespace collision_detection {

class CollisionWorldDistanceField : public CollisionWorld
{
public:

  struct DistanceFieldCacheEntry {
    std::map<std::string, 
             std::vector<PosedBodyPointDecompositionPtr> > posed_body_point_decompositions_;
    boost::shared_ptr<distance_field::DistanceField> distance_field_;
  };
  
  CollisionWorldDistanceField(double size_x = DEFAULT_SIZE_X, 
                              double size_y = DEFAULT_SIZE_Y,
                              double size_z = DEFAULT_SIZE_Z,
                              bool use_signed_distance_field = DEFAULT_USE_SIGNED_DISTANCE_FIELD,
                              double resolution = DEFAULT_RESOLUTION,
                              double collision_tolerance = DEFAULT_COLLISION_TOLERANCE,
                              double max_propagation_distance = DEFAULT_MAX_PROPAGATION_DISTANCE);

  CollisionWorldDistanceField(const CollisionWorldDistanceField &other);

  virtual ~CollisionWorldDistanceField(void){}

  virtual void checkCollision(const CollisionRequest &req,
                              CollisionResult &res,
                              const CollisionRobot &robot,
                              const kinematic_state::KinematicState &state) const;

  virtual void checkCollision(const CollisionRequest &req,
                              CollisionResult &res,
                              const CollisionRobot &robot,
                              const kinematic_state::KinematicState &state,
                              boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  virtual void checkCollision(const CollisionRequest &req,
                              CollisionResult &res,
                              const CollisionRobot &robot,
                              const kinematic_state::KinematicState &state,
                              const AllowedCollisionMatrix &acm) const;

  virtual void checkCollision(const CollisionRequest &req,
                              CollisionResult &res,
                              const CollisionRobot &robot,
                              const kinematic_state::KinematicState &state,
                              const AllowedCollisionMatrix &acm,
                              boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  virtual void checkRobotCollision(const CollisionRequest &req, 
                                   CollisionResult &res, 
                                   const CollisionRobot &robot, 
                                   const kinematic_state::KinematicState &state) const;
  
  virtual void checkRobotCollision(const CollisionRequest &req, 
                                   CollisionResult &res, 
                                   const CollisionRobot &robot, 
                                   const kinematic_state::KinematicState &state,
                                   boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  virtual void checkRobotCollision(const CollisionRequest &req, 
                                   CollisionResult &res, 
                                   const CollisionRobot &robot, 
                                   const kinematic_state::KinematicState &state, 
                                   const AllowedCollisionMatrix &acm) const;

  virtual void checkRobotCollision(const CollisionRequest &req, 
                                   CollisionResult &res, 
                                   const CollisionRobot &robot, 
                                   const kinematic_state::KinematicState &state, 
                                   const AllowedCollisionMatrix &acm,
                                   boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  virtual void checkRobotCollision(const CollisionRequest &req, 
                                   CollisionResult &res, 
                                   const CollisionRobot &robot, 
                                   const kinematic_state::KinematicState &state1, 
                                   const kinematic_state::KinematicState &state2) const {}
  virtual void checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const kinematic_state::KinematicState &state1, const kinematic_state::KinematicState &state2, const AllowedCollisionMatrix &acm) const {}
  virtual void checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world) const {}
  virtual void checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world, const AllowedCollisionMatrix &acm) const {}
  
  virtual double distanceRobot(const CollisionRobot &robot, const kinematic_state::KinematicState &state) const {return 0.0;}
  virtual double distanceRobot(const CollisionRobot &robot, const kinematic_state::KinematicState &state, const AllowedCollisionMatrix &acm) const {return 0.0;}
  virtual double distanceWorld(const CollisionWorld &world) const {return 0.0;}
  virtual double distanceWorld(const CollisionWorld &world, const AllowedCollisionMatrix &acm) const {return 0.0;}
  
  virtual void addToObject(const std::string &id, 
                           const std::vector<shapes::ShapeConstPtr> &shapes, 
                           const EigenSTL::vector_Affine3d &poses);
  
  virtual void addToObject(const std::string &id, 
                           const shapes::ShapeConstPtr &shape, 
                           const Eigen::Affine3d &pose);
  virtual bool moveShapeInObject(const std::string &id, 
                                 const shapes::ShapeConstPtr &shape, 
                                 const Eigen::Affine3d &pose);
  virtual bool removeShapeFromObject(const std::string &id, const shapes::ShapeConstPtr &shape);
  virtual void removeObject(const std::string &id);
  virtual void clearObjects(void);

  void generateEnvironmentDistanceField(bool redo = true);

  boost::shared_ptr<const distance_field::DistanceField> getDistanceField() const {
    return distance_field_cache_entry_->distance_field_;
  }

  // boost::shared_ptr<const CollisionRobotDistanceField::GroupStateRepresentation> getLastGroupStateRepresentation() const {
  //   return last_gsr_;
  // }

  void getCollisionGradients(const CollisionRequest &req, 
                             CollisionResult &res, 
                             const CollisionRobot &robot, 
                             const kinematic_state::KinematicState &state, 
                             const AllowedCollisionMatrix* acm,
                             boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  void getAllCollisions(const CollisionRequest &req, 
                        CollisionResult &res, 
                        const CollisionRobot &robot, 
                        const kinematic_state::KinematicState &state, 
                        const AllowedCollisionMatrix* acm,
                        boost::shared_ptr<GroupStateRepresentation>& gsr) const;

protected:

  boost::shared_ptr<DistanceFieldCacheEntry> generateDistanceFieldCacheEntry();


  void updateDistanceObject(const std::string& id,
                            boost::shared_ptr<CollisionWorldDistanceField::DistanceFieldCacheEntry>& dfce,
                            EigenSTL::vector_Vector3d& add_points,
                            EigenSTL::vector_Vector3d& subtract_points);

  bool getEnvironmentCollisions(const CollisionRequest& req,
                                CollisionResult& res,
                                const boost::shared_ptr<const distance_field::DistanceField>& env_distance_field,
                                boost::shared_ptr<GroupStateRepresentation>& gsr) const;
  
  
  
  bool getEnvironmentProximityGradients(const boost::shared_ptr<const distance_field::DistanceField>& env_distance_field,
                                        boost::shared_ptr<GroupStateRepresentation>& gsr) const;
                                        
  double size_x_;
  double size_y_;
  double size_z_;
  bool use_signed_distance_field_;
  double resolution_;
  double collision_tolerance_;
  double max_propagation_distance_;

  mutable boost::mutex update_cache_lock_;
  boost::shared_ptr<DistanceFieldCacheEntry> distance_field_cache_entry_;
  //boost::shared_ptr<CollisionRobotDistanceField::GroupStateRepresentation> last_gsr_;  
};

}

#endif
