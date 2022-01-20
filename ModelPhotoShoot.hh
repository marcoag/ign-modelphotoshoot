/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef SYSTEM_PLUGIN_SAMPLESYSTEM_HH_
#define SYSTEM_PLUGIN_SAMPLESYSTEM_HH_

#include <sdf/sdf.hh>

//! [header]
#include <ignition/common/Image.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/sensors.hh>
#include <ignition/transport/Node.hh>

namespace model_photo_shoot {
/// \brief System that takes snapshots of an sdf model
class ModelPhotoShoot : public ignition::gazebo::System,
                        public ignition::gazebo::ISystemConfigure {
public:
  ModelPhotoShoot();

public:
  ~ModelPhotoShoot();

public:
  void Configure(const ignition::gazebo::Entity &_id,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) final;

  void PerformPostRenderingOperations();

  void LoadModel(const ignition::gazebo::Entity &_entity,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 const std::shared_ptr<const sdf::Element> &_sdf);

  void SavePicture(const ignition::rendering::CameraPtr cam, const ignition::math::Pose3d pose,const std::string name);

  /// \brief Ignition publisher used to stop the server.
private:
  ignition::transport::Node::Publisher pubIgn;

  /// \brief Ignition publisher used to spawn the model.
private:
  ignition::transport::Node::Publisher factoryPubIgn;

  /// \brief Pointer to the sdf document.
private:
  sdf::SDFPtr sdf;

  /// \brief Name of the model.
private:
  std::string modelName;

  /// \brief Name of the world.
private:
  std::string worldName;

  /// \brief world instance.
private:
  std::shared_ptr<ignition::gazebo::Model> world;

private:
  std::shared_ptr<ignition::transport::Node> factoryPub;

  /// \brief Pointer to the scene.
private:
  ignition::rendering::ScenePtr scene;

  /// \brief Pointer to the camera.
private:
  ignition::rendering::CameraPtr camera;

private:
  ignition::sensors::Manager mgr;

  /// \brief Pointer to the light.
private:
  ignition::rendering::LightPtr light;

  /// \brief Connection to pre-render event callback
private:
  ignition::common::ConnectionPtr connection{nullptr};

  /// \brief Boolean to control we only take the picture once
private:
  bool take_picture;
};
} // namespace model_photo_shoot
//! [header]

#endif
