#ifndef CMGPLUGIN_HH_
#define CMGPLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

/// \brief Single gimballed Control Moment Gyro plugin. Multiple can be used to make a full 3-axis CMG system. 
///
/// ## System Parameters:
///
///   `<parent_link>` The link in the target model to attach the CMG.
///   Required.
///
///   `<child_model>` The name of the parachute model.
///   Required.
///
///   `<child_link>` The base link of the parachute model (bridle point).
///   Required.
///
///   `<child_pose>` The relative pose of parent link to the child link.
///   The default value is: `0, 0, 0, 0, 0, 0`.
///
///   `<cmd_topic>` The topic to receive  the CMG command.
///   The default value is: `/model/<model_name>/cmg/cmd`.
///
class CMGPlugin :
    public System,
    public ISystemConfigureParameters,
    public ISystemPreUpdate,
    public ISystemConfigure
{
  /// \brief Destructor
  public: virtual ~CMGPlugin();

  /// \brief Constructor
  public: CMGPlugin();

  // Documentation inherited
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         EntityComponentManager &_ecm) final;

  // Documentation inherited
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &) final;
  
  public: void ConfigureParameters(
      gz::transport::parameters::ParametersRegistry &_registry,
      gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Load control channels
  private: void LoadControlChannels(
    sdf::ElementPtr _sdf,
    gz::sim::EntityComponentManager &_ecm);
    
  /// \internal
  /// \brief Private implementation
  private: class Impl;
  private: std::unique_ptr<Impl> impl;
};

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // PARACHUTEPLUGIN_HH_