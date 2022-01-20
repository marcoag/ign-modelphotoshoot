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

  //Configuration values
  std::string model_location = _sdf->Get<std::string>("model_uri");
  if (model_location.empty()) {
    ignerr << "Please specify the model location through the <model_location>"
        "tag in the sdf file.\n";
    return;
  }
  std::string save_data_location = _sdf->Get<std::string>("translation_data_file");
  if (save_data_location.empty()) {
    igndbg << "No data location specified, skipping translaiton data"
        "saving.\n";
  }
  else {
    igndbg << "Saving translation data to: " << save_data_location << std::endl;
    this->savingFile.open(save_data_location);
  }

  this->connection = _eventMgr.Connect<ignition::gazebo::events::PostRender>(
      std::bind(&ModelPhotoShoot::PerformPostRenderingOperations, this));

  this->LoadModel(model_location, _entity, _ecm, _sdf);
}

void ModelPhotoShoot::LoadModel(
    const std::string model_location,
    const ignition::gazebo::Entity &_entity,
    ignition::gazebo::EntityComponentManager &_ecm,
    const std::shared_ptr<const sdf::Element> &_sdf) {
  this->world = std::make_shared<ignition::gazebo::Model>(_entity);
  this->worldName = this->world->Name(_ecm);

  this->sdf.reset(new sdf::SDF());

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

  ignition::rendering::VisualPtr root = this->scene->RootVisual();

  if (vis && take_picture) {

    this->scene->SetAmbientLight(0.3, 0.3, 0.3);
    this->scene->SetBackgroundColor(0.3, 0.3, 0.3);

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

    for (unsigned int i = 0; i < this->scene->NodeCount(); ++i) {
      auto cam = std::dynamic_pointer_cast<ignition::rendering::Camera>(
          this->scene->NodeByIndex(i));
      if (nullptr != cam) {

        //Set the model pose
        ignition::math::AxisAlignedBox bbox = vis->LocalBoundingBox();
        ignition::rendering::WireBoxPtr wireBox = this->scene->CreateWireBox();

        double scaling = 1.0 / bbox.Size().Max();

        // Compute the model translation.
        ignition::math::Vector3d trans = bbox.Center();
        trans *= -scaling;

        if (savingFile.is_open())
          savingFile << "Translation: " << trans << std::endl;

        // Normalize the size of the visual
        vis->SetLocalScale(ignition::math::Vector3d(scaling, scaling, scaling));
        vis->SetWorldPose(
            ignition::math::Pose3d(trans.X(), trans.Y(), trans.Z(), 0, 0, 0));

        ignition::math::Pose3d pose;

        //Perspective view
        pose.Pos().Set(1.6, -1.6, 1.2);
        pose.Rot().Euler(0, IGN_DTOR(30), IGN_DTOR(-225));
        SavePicture(cam, pose, "1");

        //Top view
        pose.Pos().Set(0, 0, 2.2);
        pose.Rot().Euler(0, IGN_DTOR(90), 0);
        SavePicture(cam, pose, "2");

        // Front view
        pose.Pos().Set(2.2, 0, 0);
        pose.Rot().Euler(0, 0, IGN_DTOR(-180));
        SavePicture(cam, pose, "3");

        // Side view
        pose.Pos().Set(0, 2.2, 0);
        pose.Rot().Euler(0, 0, IGN_DTOR(-90));
        SavePicture(cam, pose, "4");

        // Back view
        pose.Pos().Set(-2.2, 0, 0);
        pose.Rot().Euler(0, 0, 0);
        SavePicture(cam, pose, "5");

        take_picture = false;
      }
    }
  }
}

void ModelPhotoShoot::SavePicture(const ignition::rendering::CameraPtr cam,
                                  const ignition::math::Pose3d pose,
                                  const std::string name)
{
  unsigned int width = cam->ImageWidth();
  unsigned int height = cam->ImageHeight();

  ignition::common::Image image;

  cam->SetWorldPose(pose);
  auto cameraImage = cam->CreateImage();
  cam->Capture(cameraImage);
  auto formatStr =
      ignition::rendering::PixelUtil::Name(cam->ImageFormat());
  auto format = ignition::common::Image::ConvertPixelFormat(formatStr);
  image.SetFromData(cameraImage.Data<unsigned char>(), width, height,
                        format);
  std::string fullname = name + ".png";
  image.SavePNG(fullname);

  igndbg << "Saved image to [" << fullname << "]" << std::endl;
}
