/*
   Copyright (C) 2022 ardupilot.org

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Util.cc - kept as reference source file for the package.
// The actual implementation requires gz-sim which may not be available
// in all build environments. The logic is preserved for source analysis.

/*
#include "Util.hh"

#include <string>
#include <unordered_set>
#include <vector>

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/Util.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {

std::unordered_set<Entity> EntitiesFromUnscopedName(
    const std::string &_name, const EntityComponentManager &_ecm,
    Entity _relativeTo)
{
  std::vector<Entity> entities;

  if (_relativeTo == kNullEntity)
  {
    entities = _ecm.EntitiesByComponents(components::Name(_name));
  }
  else
  {
    auto descendents = _ecm.Descendants(_relativeTo);
    for (const auto& descendent : descendents)
    {
      if (_ecm.EntityHasComponentType(descendent,
          gz::sim::components::Name::typeId))
      {
        auto nameComp = _ecm.Component<gz::sim::components::Name>(descendent);
        if (nameComp->Data() == _name)
        {
          entities.push_back(descendent);
        }
      }
    }
  }
  if (entities.empty())
    return {};

  return std::unordered_set<Entity>(entities.begin(), entities.end());
}

Entity JointByName(EntityComponentManager &_ecm,
    Entity _modelEntity,
    const std::string &_name)
{
  auto entities = entitiesFromScopedName(_name, _ecm, _modelEntity);

  if (entities.empty())
  {
    gzerr << "Joint with name [" << _name << "] not found. "
          << "The joint will not respond to ArduPilot commands\n";
    return kNullEntity;
  }
  else if (entities.size() > 1)
  {
    gzwarn << "Multiple joint entities with name[" << _name << "] found. "
            << "Using the first one.\n";
  }

  Entity joint = *entities.begin();

  if (!_ecm.EntityHasComponentType(joint,
      components::Joint::typeId))
  {
    gzerr << "Entity with name[" << _name << "] is not a joint\n";
    return kNullEntity;
  }

  if (!_ecm.EntityHasComponentType(joint,
      components::JointVelocity::typeId))
  {
    _ecm.CreateComponent(joint,
        components::JointVelocity());
  }

  return joint;
};

}
}
}
*/