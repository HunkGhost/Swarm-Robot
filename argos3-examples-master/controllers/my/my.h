
#ifndef MY_H
#define MY_H
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
using namespace argos;

class CFootBotDiffusion : public CCI_Controller {
 
public:

   CFootBotDiffusion();

   virtual ~CFootBotDiffusion() {}

   virtual void Init(TConfigurationNode& t_node);

   virtual void ControlStep();

   virtual void Reset() {}
 
   virtual void Destroy() {}
 
private:

   CCI_DifferentialSteeringActuator* m_pcWheels;

   CCI_FootBotProximitySensor* m_pcProximity;

   CDegrees m_cAlpha;

   Real m_fDelta;
   /* Wheel speed. */
   Real m_fWheelVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> m_cGoStraightAngleRange;
 
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
   
   CCI_LEDsActuator* m_pcLEDs;
   
   // set 180 degree threshold to know the closest bot is on righ or left
   CRadians AngleThreshold;
   
   Real TargetDistance;
};
 
#endif
