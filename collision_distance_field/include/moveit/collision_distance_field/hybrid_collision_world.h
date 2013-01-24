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

#ifndef MOVEIT_COLLISION_DISTANCE_FIELD_HYBRID_COLLISION_WORLD_
#define MOVEIT_COLLISION_DISTANCE_FIELD_HYBRID_COLLISION_WORLD_

#include <moveit/collision_detection_fcl/collision_world.h>
#include <moveit/collision_distance_field/collision_distance_field_types.h>
#include <moveit/collision_distance_field/collision_world_distance_field.h>

namespace collision_detection {

class CollisionWorldHybrid : public CollisionWorldFCL
{
public:

  CollisionWorldHybrid(double size_x = 3.0, 
                       double size_y = 3.0,
                       double size_z = 4.0,
                       bool use_signed_distance_field = false,
                       double resolution = .02,
                       double collision_tolerance = 0.0,
                       double max_propagation_distance = .25);

  CollisionWorldHybrid(const CollisionWorldHybrid &other);

  virtual ~CollisionWorldHybrid(void){}

  void checkCollisionDistanceField(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const CollisionRobot &robot,
                                   const kinematic_state::KinematicState &state) const;

  void checkCollisionDistanceField(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const CollisionRobot &robot,
                                   const kinematic_state::KinematicState &state,
                                   boost::shared_ptr<GroupStateRepresentation>& gsr) const;
    
  void checkCollisionDistanceField(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const CollisionRobot &robot,
                                   const kinematic_state::KinematicState &state,
                                   const AllowedCollisionMatrix &acm) const;

  void checkCollisionDistanceField(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const CollisionRobot &robot,
                                   const kinematic_state::KinematicState &state,
                                   const AllowedCollisionMatrix &acm,
                                   boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  void checkRobotCollisionDistanceField(const CollisionRequest &req, 
                                        CollisionResult &res, 
                                        const CollisionRobot &robot, 
                                        const kinematic_state::KinematicState &state) const;

  void checkRobotCollisionDistanceField(const CollisionRequest &req, 
                                        CollisionResult &res, 
                                        const CollisionRobot &robot, 
                                        const kinematic_state::KinematicState &state,
                                        boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  void checkRobotCollisionDistanceField(const CollisionRequest &req, 
                                        CollisionResult &res, 
                                        const CollisionRobot &robot, 
                                        const kinematic_state::KinematicState &state, 
                                        const AllowedCollisionMatrix &acm) const;

  void checkRobotCollisionDistanceField(const CollisionRequest &req, 
                                        CollisionResult &res, 
                                        const CollisionRobot &robot, 
                                        const kinematic_state::KinematicState &state, 
                                        const AllowedCollisionMatrix &acm,
                                        boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  virtual void addToObject(const std::string &id, const std::vector<shapes::ShapeConstPtr> &shapes, const EigenSTL::vector_Affine3d &poses);
  
  virtual void addToObject(const std::string &id, 
                           const shapes::ShapeConstPtr &shape, 
                           const Eigen::Affine3d &pose);
  virtual bool moveShapeInObject(const std::string &id, 
                                 const shapes::ShapeConstPtr &shape, 
                                 const Eigen::Affine3d &pose);
  virtual bool removeShapeFromObject(const std::string &id, const shapes::ShapeConstPtr &shape);
  virtual void removeObject(const std::string &id);
  virtual void clearObjects(void);

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

  const boost::shared_ptr<const collision_detection::CollisionWorldDistanceField> getCollisionWorldDistanceField() const {
    return cworld_distance_;
  }

protected:
  boost::shared_ptr<collision_detection::CollisionWorldDistanceField> cworld_distance_;
};

}

#endif
