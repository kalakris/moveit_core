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

#include <moveit/collision_distance_field/collision_common_distance_field.h>
#include <boost/thread/mutex.hpp>
#include <cassert>

namespace collision_detection
{

struct BodyDecompositionCache
{
  BodyDecompositionCache(void) : clean_count_(0) {}
  static const unsigned int MAX_CLEAN_COUNT = 100;
  std::map<boost::weak_ptr<const shapes::Shape>, BodyDecompositionConstPtr> map_;
  unsigned int clean_count_;
  boost::mutex lock_;
};

BodyDecompositionCache& getBodyDecompositionCache(void) 
{
  static BodyDecompositionCache cache;
  return cache;
}

BodyDecompositionConstPtr getBodyDecompositionCacheEntry(const shapes::ShapeConstPtr& shape,
                                                         double resolution)
{
  //TODO - deal with changing resolution?
  BodyDecompositionCache& cache = getBodyDecompositionCache();
  boost::weak_ptr<const shapes::Shape> wptr(shape);
  {
    boost::mutex::scoped_lock slock(cache.lock_);
    std::map<boost::weak_ptr<const shapes::Shape>, BodyDecompositionConstPtr>::const_iterator cache_it = cache.map_.find(wptr);
    if(cache_it != cache.map_.end()) {
      return cache_it->second;
    }
  } 
  BodyDecompositionConstPtr bdcp(new BodyDecomposition(shape, resolution));
  {
    boost::mutex::scoped_lock slock(cache.lock_);
    cache.map_[wptr] = bdcp;
    cache.clean_count_++;
    return bdcp;
  }
  //TODO - clean cache
}

PosedBodyPointDecompositionVectorPtr getCollisionObjectPointDecomposition(const collision_detection::CollisionWorld::Object& obj,
                                                                     double resolution)
{
  PosedBodyPointDecompositionVectorPtr ret(new PosedBodyPointDecompositionVector());
  for(unsigned int i = 0; i < obj.shapes_.size(); i++) {
    PosedBodyPointDecompositionPtr pbd(new PosedBodyPointDecomposition(getBodyDecompositionCacheEntry(obj.shapes_[i], resolution)));
    ret->addToVector(pbd);
    ret->updatePose(ret->getSize()-1, obj.shape_poses_[i]);
  }
  return ret;
}

PosedBodySphereDecompositionVectorPtr getAttachedBodySphereDecomposition(const kinematic_state::AttachedBody* att,
                                                                         double resolution)
{
  PosedBodySphereDecompositionVectorPtr ret(new PosedBodySphereDecompositionVector());
  for(unsigned int i = 0; i < att->getShapes().size(); i++) {
    PosedBodySphereDecompositionPtr pbd(new PosedBodySphereDecomposition(getBodyDecompositionCacheEntry(att->getShapes()[i], resolution)));
    pbd->updatePose(att->getGlobalCollisionBodyTransforms()[i]);
    ret->addToVector(pbd);
  }
  return ret;
}

PosedBodyPointDecompositionVectorPtr getAttachedBodyPointDecomposition(const kinematic_state::AttachedBody* att,
                                                                       double resolution)
{
  PosedBodyPointDecompositionVectorPtr ret(new PosedBodyPointDecompositionVector());
  for(unsigned int i = 0; i < att->getShapes().size(); i++) {
    PosedBodyPointDecompositionPtr pbd(new PosedBodyPointDecomposition(getBodyDecompositionCacheEntry(att->getShapes()[i], resolution)));
    ret->addToVector(pbd);
    ret->updatePose(ret->getSize()-1, att->getGlobalCollisionBodyTransforms()[i]);
  }
  return ret;
}

void getBodySphereVisualizationMarkers(boost::shared_ptr<const GroupStateRepresentation> gsr,
                                       const std::string& frame_id,
                                       const ros::Time& time_stamp,
                                       visualization_msgs::MarkerArray& marker_array)
{
  marker_array.markers.clear();
  if (!gsr)
    return;
  for (size_t i=0; i<gsr->link_body_decompositions_.size(); ++i)
  {
    if (!gsr->link_body_decompositions_[i])
      continue;
    const EigenSTL::vector_Vector3d& sphere_centers = gsr->link_body_decompositions_[i]->getSphereCenters();
    const std::vector<double>& sphere_radii = gsr->link_body_decompositions_[i]->getSphereRadii();
    assert(sphere_centers.size() == sphere_radii.size());
    for (size_t j=0; j<sphere_centers.size(); ++j)
    {
      visualization_msgs::Marker m;
      m.header.frame_id = frame_id;
      m.header.stamp = time_stamp;
      m.ns = gsr->dfce_->link_names_[i];
      m.id = j;
      m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::ADD;
      m.color.a = 0.5;
      m.color.r = 0.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      m.pose.orientation.w = 1.0;
      m.pose.orientation.x = 0.0;
      m.pose.orientation.y = 0.0;
      m.pose.orientation.z = 0.0;
      m.pose.position.x = sphere_centers[j](0);
      m.pose.position.y = sphere_centers[j](1);
      m.pose.position.z = sphere_centers[j](2);
      m.scale.x = 2 * sphere_radii[j];
      m.scale.y = 2 * sphere_radii[j];
      m.scale.z = 2 * sphere_radii[j];
      marker_array.markers.push_back(m);
    }
  }
}

}

