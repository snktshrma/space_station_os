#ifndef UTIL_HH_
#define UTIL_HH_

#include <string>
#include <unordered_set>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Export.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {

/// \brief Helper function to get an entity given its unscoped name.
///
/// \param[in] _name Entity's unscoped name.
/// \param[in] _ecm Immutable reference to ECM.
/// \param[in] _relativeTo Entity that the unscoped name is relative to.
/// If not provided, the unscoped name could be relative to any entity.
/// \return All entities that match the unscoped name and relative to
/// requirements, or an empty set otherwise.
std::unordered_set<Entity> EntitiesFromUnscopedName(
    const std::string &_name, const EntityComponentManager &_ecm,
    Entity _relativeTo = kNullEntity);

/// \brief Get the ID of a joint entity which is a descendent of this model.
///
/// A replacement for gz::sim::Model::JointByName which does not resolve
/// joints for nested models.
/// \param[in] _ecm Entity-component manager.
/// \param[in] _entity Model entity.
/// \param[in] _name Scoped joint name.
/// \return Joint entity.
Entity JointByName(EntityComponentManager &_ecm,
    Entity _modelEntity,
    const std::string &_name);

}
}  // namespace sim
}  // namespace gz

#endif  // UTIL_HH_