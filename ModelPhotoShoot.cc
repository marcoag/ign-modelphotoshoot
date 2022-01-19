#include "ModelPhotoShoot.hh"

#include <boost/program_options.hpp>

#include <ignition/common/MeshManager.hh>
#include <ignition/gazebo/rendering/Events.hh>
#include <ignition/msgs.hh>
#include <ignition/plugin/Register.hh>

// Include a line in your source file for each interface implemented.
IGNITION_ADD_PLUGIN(model_photo_shoot::ModelPhotoShoot,
                    ignition::gazebo::System,
                    model_photo_shoot::ModelPhotoShoot::ISystemConfigure)

using namespace model_photo_shoot;
namespace po = boost::program_options;

ModelPhotoShoot::ModelPhotoShoot()
    : factoryPub(std::make_shared<ignition::transport::Node>()),
      take_picture(true) {}

ModelPhotoShoot::~ModelPhotoShoot() {}

void ModelPhotoShoot::Configure(const ignition::gazebo::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                ignition::gazebo::EntityComponentManager &_ecm,
                                ignition::gazebo::EventManager &_eventMgr) {

  this->connection = _eventMgr.Connect<ignition::gazebo::events::PostRender>(
      std::bind(&ModelPhotoShoot::PerformPostRenderingOperations, this));

  this->LoadModel(_entity, _ecm, _sdf);
}

void ModelPhotoShoot::LoadModel(
    const ignition::gazebo::Entity &_entity,
    ignition::gazebo::EntityComponentManager &_ecm,
    const std::shared_ptr<const sdf::Element> &_sdf) {
  this->world = std::make_shared<ignition::gazebo::Model>(_entity);
  this->worldName = this->world->Name(_ecm);

  this->sdf.reset(new sdf::SDF());

  std::string model_location = _sdf->Get<std::string>("model_uri");

  if (!sdf::init(this->sdf)) {
    ignerr << "ERROR: SDF parsing the xml failed\n";
    return;
  }

  if (!sdf::readFile(model_location, this->sdf)) {
    ignerr << "Error: SDF parsing the xml failed\n";
    return;
  }

  sdf::ElementPtr modelElem = this->sdf->Root()->GetElement("model");
  this->modelName = modelElem->Get<std::string>("name");

  // Create entity
  std::string service = "/world/" + this->worldName + "/create";
  ignition::msgs::EntityFactory request;
  request.set_sdf(this->sdf->ToString());
  request.set_name(this->modelName);

  ignition::msgs::Boolean response;
  bool result;
  uint32_t timeout = 5000;
  bool executed =
      this->factoryPub->Request(service, request, timeout, response, result);
  if (executed) {
    if (result && response.data()) {
      igndbg << "Requested creation of entity: " << this->modelName.c_str()
             << std::endl;
    } else {
      ignerr << "Failed request to create entity.\n"
             << request.DebugString().c_str();
    }
  } else {
    ignerr << "Request to create entity from service "
           << request.DebugString().c_str() << "timer out ...\n";
  }
}

void ModelPhotoShoot::PerformPostRenderingOperations() {
  igndbg << "PerformPostRenderingOperations\n";

  this->scene = ignition::rendering::sceneFromFirstRenderEngine();
  ignition::rendering::v6::VisualPtr vis =
      this->scene->VisualByName(this->modelName);

  this->scene->SetAmbientLight(0.1, 0.1, 0.1);
  this->scene->SetBackgroundColor(ignition::math::Color(0.8, 0.898039f, 1));
  // this->scene->PreRender();
  ignition::rendering::VisualPtr root = this->scene->RootVisual();

  // create directional light
  ignition::rendering::DirectionalLightPtr light0 =
      this->scene->CreateDirectionalLight();
  light0->SetDirection(-0.5, 0.5, -1);
  light0->SetDiffuseColor(0.8, 0.8, 0.8);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  // create point light
  ignition::rendering::PointLightPtr light2 = this->scene->CreatePointLight();
  light2->SetDiffuseColor(0.5, 0.5, 0.5);
  light2->SetSpecularColor(0.5, 0.5, 0.5);
  light2->SetLocalPosition(3, 5, 5);
  root->AddChild(light2);

  if (vis && take_picture) {
    for (unsigned int i = 0; i < this->scene->NodeCount(); ++i) {
      auto cam = std::dynamic_pointer_cast<ignition::rendering::Camera>(
          this->scene->NodeByIndex(i));
      if (nullptr != cam) {

        unsigned int width = cam->ImageWidth();
        unsigned int height = cam->ImageHeight();

        cam->Update();

        auto cameraImage = cam->CreateImage();
        cam->Copy(cameraImage);

        auto formatStr =
            ignition::rendering::PixelUtil::Name(cam->ImageFormat());
        auto format = ignition::common::Image::ConvertPixelFormat(formatStr);
        std::string name = ignition::common::systemTimeISO() + ".png";

        ignition::common::Image image;
        image.SetFromData(cameraImage.Data<unsigned char>(), width, height,
                          format);
        image.SavePNG(name);
        igndbg << "Saved image to [" << name << "]" << std::endl;

        take_picture = false;
      }
    }
  }
}
