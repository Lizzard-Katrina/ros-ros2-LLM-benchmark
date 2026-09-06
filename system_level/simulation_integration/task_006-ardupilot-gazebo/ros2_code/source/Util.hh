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

#pragma once

#include <string>
#include <unordered_set>

// Minimal forward declarations - this header is only needed for compilation
// of ArduPilotPlugin.cc but we don't link against gz-sim in our test build.

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {

using Entity = uint64_t;
class EntityComponentManager;

std::unordered_set<Entity> EntitiesFromUnscopedName(
    const std::string &_name, const EntityComponentManager &_ecm,
    Entity _relativeTo);

Entity JointByName(EntityComponentManager &_ecm,
    Entity _modelEntity,
    const std::string &_name);

}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz