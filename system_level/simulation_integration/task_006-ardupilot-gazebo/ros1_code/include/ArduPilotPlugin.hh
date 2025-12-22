#ifndef ARDUPILOTPLUGIN_HH_
#define ARDUPILOTPLUGIN_HH_

#include <array>
#include <memory>
#include <gz/sim/System.hh>
#include <sdf/sdf.hh>

namespace gz
{
namespace sim
{
namespace systems
{

struct servo_packet_16 { uint16_t magic; uint16_t frame_rate; uint32_t frame_count; uint16_t pwm[16]; };
struct servo_packet_32 { uint16_t magic; uint16_t frame_rate; uint32_t frame_count; uint16_t pwm[32]; };

class ArduPilotPluginPrivate;

class GZ_SIM_VISIBLE ArduPilotPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{
  public: ArduPilotPlugin();
  public: ~ArduPilotPlugin();

  public: void Reset(const UpdateInfo &_info, EntityComponentManager &_ecm) final;
  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr) final;
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) final;
  // TODO: implement closed-loop update
  // END
  private: bool ReceiveServoPacket();
  private: void UpdateMotorCommands(const std::array<uint16_t, 32> &_pwm);
  private: void CreateStateJSON(double _simTime, const gz::sim::EntityComponentManager &_ecm) const;
  private: void SendState() const;
  private: bool InitSockets(sdf::ElementPtr _sdf) const;

  private: std::unique_ptr<ArduPilotPluginPrivate> dataPtr;
};

}  // namespace systems
}  // namespace sim
}  // namespace gz

#endif  // ARDUPILOTPLUGIN_HH_
