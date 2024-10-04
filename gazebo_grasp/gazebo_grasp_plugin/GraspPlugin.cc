#include "GraspPlugin.hh"

#include <gz/msgs/contact.pb.h>
#include <gz/msgs/contacts.pb.h>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <sdf/Element.hh>

#include <gz/transport/Node.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/Conversions.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/components/Link.hh"
#include <gz/sim/Joint.hh>
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class ContactSensor
{
  /// \brief Load the GraspPlugin sensor from an sdf element
  /// \param[in] _sdf SDF element describing the GraspPlugin sensor
  /// \param[in] _topic string with topic name
  /// \param[in] _collisionEntities A list of entities that act as contact
  /// sensors
  public: void Load(const sdf::ElementPtr &_sdf, const std::string &_topic,
                    const std::vector<Entity> &_collisionEntities);

  /// \brief Add contacts to the list to be published
  /// \param[in] _stamp Time stamp of the sensor measurement
  /// \param[in] _contacts A contact message to be added to the list
  public: void AddContacts(const std::chrono::steady_clock::duration &_stamp,
                           const msgs::Contacts &_contacts);

  /// \brief Publish sensor data over gz transport
  public: void Publish();

  /// \brief Topic to publish data to
  public: std::string topic;

  /// \brief Message to publish
  public: msgs::Contacts contactsMsg;

  /// \brief Gazebo transport node
  public: transport::Node node;

  /// \brief Gazebo transport publisher
  public: transport::Node::Publisher pub;

  /// \brief Entities for which this sensor publishes data
  public: std::vector<Entity> collisionEntities;
};

class gz::sim::systems::GraspPluginPrivate
{
  /// \brief Create sensors that correspond to entities in the simulation
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateSensors(EntityComponentManager &_ecm);

  /// \brief Update and publish sensor data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateSensors(const UpdateInfo &_info,
                             const EntityComponentManager &_ecm);

  /// \brief Remove sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveSensors(const EntityComponentManager &_ecm);

	/// \brief Attach the parent and child link in contact.
  /// \param[in] _ecm Mutable reference to ECM.
  public: void AttachLinks(EntityComponentManager &_ecm);

	/// \brief Detach the parent and child link in contact.
  /// \param[in] _ecm Mutable reference to ECM.
  public: void DetachLinks(EntityComponentManager &_ecm);

  /// \brief A map of GraspPlugin entity to its GraspPlugin sensor.
  public: std::unordered_map<Entity,
      std::unique_ptr<ContactSensor>> entitySensorMap;

  /// \brief Model interface
  public: Model model{kNullEntity};
	/// \brief Entity of attachment link in the parent model
  public: Entity parentLinkEntity{kNullEntity};
  /// \brief Entity of attachment link in the child model
  private: Entity childLinkEntity{kNullEntity};
  /// \brief Entity of the detachable joint created by this system
  private: Entity detachableJointEntity{kNullEntity};
  /// \brief Whether detachment has been requested
  private: std::atomic<bool> detachRequested{false};
  /// \brief Whether attachment has been requested
  private: std::atomic<bool> attachRequested{true};
  /// \brief Whether child entity is attached
  private: std::atomic<bool> isAttached{false};
  ///  \brief Joint position for the gripper
  private: double jointPosition;
  ///  \brief Joint position for the gripper at first contact
  private: double initJointPosition;
  ///  \brief Joint position for the gripper
  private: double prevJointPosition{-1.0};
  ///  \brief Joint position for the gripper
  private: double jointDiff{9999.0};
  ///  \brief Joint controlling the contact link
  private: Entity desiredJointEntity{kNullEntity};
  ///  \brief Joint controlling the contact link
  private: Joint desiredJoint{kNullEntity};
  ///  \brief initialize parameters
  private: std::atomic<bool> initialziedParams{false};
};

//////////////////////////////////////////////////
void ContactSensor::Load(const sdf::ElementPtr &_sdf, const std::string &_topic,
                         const std::vector<Entity> &_collisionEntities)
{
  this->collisionEntities = _collisionEntities;

  auto contactElem = _sdf->GetElement("contact");
  auto tmpTopic =
      contactElem->Get<std::string>("topic", "__default_topic__").first;

  if (tmpTopic == "__default_topic__")
  {
    // use default topic for sensor
    this->topic = _topic;
  }
  else
  {
    this->topic = tmpTopic;
  }

  gzmsg << "GraspPlugin system publishing on " << this->topic << std::endl;
  this->pub = this->node.Advertise<msgs::Contacts>(this->topic);
}

//////////////////////////////////////////////////
void ContactSensor::AddContacts(
    const std::chrono::steady_clock::duration &_stamp,
    const msgs::Contacts &_contacts)
{
  auto stamp = convert<msgs::Time>(_stamp);
  for (const auto &contact : _contacts.contact())
  {
    auto *newContact = this->contactsMsg.add_contact();
    newContact->CopyFrom(contact);
    newContact->mutable_header()->mutable_stamp()->CopyFrom(stamp);
  }

  this->contactsMsg.mutable_header()->mutable_stamp()->CopyFrom(stamp);
}

//////////////////////////////////////////////////
void ContactSensor::Publish()
{
  // Only publish if there are contacts
  if (this->contactsMsg.contact_size() > 0)
  {
    this->pub.Publish(this->contactsMsg);
    this->contactsMsg.Clear();
  }
}

//////////////////////////////////////////////////
void GraspPluginPrivate::CreateSensors(EntityComponentManager &_ecm)
{
  GZ_PROFILE("GraspPluginPrivate::CreateSensors");
  _ecm.EachNew<components::ContactSensor>(
      [&](const Entity &_entity,
          const components::ContactSensor *_contact) -> bool
      {
        // Check if the parent entity is a link
        auto *parentEntity = _ecm.Component<components::ParentEntity>(_entity);
        if (nullptr == parentEntity)
          return true;

        auto *linkComp = _ecm.Component<components::Link>(parentEntity->Data());
        if (nullptr == linkComp)
        {
          // GraspPlugin sensors should only be attached to links
          return true;
        }

        auto *parentEntitym = _ecm.Component<components::ParentEntity>(parentEntity->Data());
        if (nullptr == parentEntitym)
          return true;  

        if (!this->initialziedParams)
        {
          // Get the robot arm model
          this->model = Model(parentEntitym->Data());

          // Get the palm link
          this->parentLinkEntity = this->model.LinkByName(_ecm, "wrist_3_link");
          this->desiredJointEntity = this->model.JointByName(_ecm, "robotiq_85_left_knuckle_joint");
          this->desiredJoint = Joint(this->desiredJointEntity);

          this->initialziedParams = true;
        }

        auto collisionElem =
            _contact->Data()->GetElement("contact")->GetElement("collision");

        std::vector<Entity> collisionEntities;
        // Get all the collision elements
        for (; collisionElem;
             collisionElem = collisionElem->GetNextElement("collision"))
        {
          auto collisionName = collisionElem->Get<std::string>();
          // Get collision entity that matches the name given by the sensor's
          // configuration.
          auto childEntities = _ecm.ChildrenByComponents(
              parentEntity->Data(), components::Collision(),
              components::Name(collisionName));

          if (!childEntities.empty())
          {
            // We assume that if childEntities is not empty, it only has one
            // element.
            collisionEntities.push_back(childEntities.front());

            // Create component to be filled by physics.
            _ecm.CreateComponent(childEntities.front(),
                                 components::ContactSensorData());
          }
        }

        std::string defaultTopic = scopedName(_entity, _ecm, "/") + "/contact";

        auto sensor = std::make_unique<ContactSensor>();
        sensor->Load(_contact->Data(), defaultTopic, collisionEntities);
        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void GraspPluginPrivate::UpdateSensors(const UpdateInfo &_info,
                                   const EntityComponentManager &_ecm)
{
  GZ_PROFILE("GraspPluginPrivate::UpdateSensors");
  for (const auto &item : this->entitySensorMap)
  {
    for (const Entity &entity : item.second->collisionEntities)
    {
      auto contacts = _ecm.Component<components::ContactSensorData>(entity);
      // We will assume that the ContactData component will have been created if
      // this entity is in the collisionEntities list
      if (contacts->Data().contact_size() > 0)
      {
        item.second->AddContacts(_info.simTime, contacts->Data());
      }
    }
  }
}

//////////////////////////////////////////////////
void GraspPluginPrivate::RemoveSensors(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("GraspPluginPrivate::RemoveSensors");
  _ecm.EachRemoved<components::ContactSensor>(
    [&](const Entity &_entity,
        const components::ContactSensor *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing GraspPlugin sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

void GraspPluginPrivate::AttachLinks(
    EntityComponentManager &_ecm)
{
	GZ_PROFILE("GraspPluginPrivate::AttachLinks");
  auto item = this->entitySensorMap.begin();
  auto contacts = _ecm.Component<components::ContactSensorData>(item->second->collisionEntities.front());

  auto force = _ecm.Component<components::JointForceCmd>(this->desiredJointEntity);

  // Let the gripper position come to halt before attaching the object
  if (contacts->Data().contact_size() > 0)
  {
    auto jointPositionVec = this->desiredJoint.Position(_ecm);
    this->jointPosition = jointPositionVec->front();
    this->jointDiff = this->jointPosition - this->prevJointPosition;
    this->prevJointPosition = this->jointPosition;
  }

  if (abs(this->jointDiff) < 1e-7 && this->isAttached == false)
  {
	  this->detachableJointEntity = _ecm.CreateEntity();
    auto childLinkId = contacts->Data().contact().begin()->collision2().id();
    auto childLinkCollision = Entity(childLinkId);
    auto *parentEntity = _ecm.Component<components::ParentEntity>(childLinkCollision);
    this->childLinkEntity = parentEntity->Data();
    force->Data()[0] += -1.0;
	  _ecm.CreateComponent(
              this->detachableJointEntity,
              components::DetachableJoint({this->parentLinkEntity,
                                           this->childLinkEntity, "fixed"}));
  
	  this->isAttached = true;
    this->jointDiff = 999.0;
    this->prevJointPosition = -1.0;
	  gzerr << "Attaching entity: " << this->detachableJointEntity
                 << std::endl;
  }
}

void GraspPluginPrivate::DetachLinks(
    EntityComponentManager &_ecm)
{
	GZ_PROFILE("GraspPluginPrivate::DetachLinks");

  // The joint has value between 0.0 and 0.8
  auto detachJointValue = this->jointPosition - this->jointPosition*0.1; 
	if (this->isAttached && this->desiredJoint.Position(_ecm)->front() < detachJointValue)
	{
		gzerr << "Removing entity: " << this->detachableJointEntity << std::endl;
  	_ecm.RequestRemoveEntity(this->detachableJointEntity);
  	this->detachableJointEntity = kNullEntity;
  	this->isAttached = false;
	}

}

//////////////////////////////////////////////////
GraspPlugin::GraspPlugin() : System(), dataPtr(std::make_unique<GraspPluginPrivate>())
{
}

//////////////////////////////////////////////////
void GraspPlugin::PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm)
{
  GZ_PROFILE("GraspPlugin::PreUpdate");
  this->dataPtr->CreateSensors(_ecm);
  if (!this->dataPtr->entitySensorMap.empty())
  {
	  this->dataPtr->AttachLinks(_ecm);
	  this->dataPtr->DetachLinks(_ecm);
  }
}


//////////////////////////////////////////////////
void GraspPlugin::PostUpdate(const UpdateInfo &_info,
                         const EntityComponentManager &_ecm)
{
  GZ_PROFILE("GraspPlugin::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  if (!_info.paused)
  {
    this->dataPtr->UpdateSensors(_info, _ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Publish sensor data
      it.second->Publish();
    }
  }

  this->dataPtr->RemoveSensors(_ecm);
}

GZ_ADD_PLUGIN(GraspPlugin, System,
  GraspPlugin::ISystemPreUpdate,
  GraspPlugin::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(GraspPlugin, "gz::sim::systems::GraspPlugin")