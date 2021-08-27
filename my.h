
#ifndef MY_H
#define MY_H
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the foot-bot motor ground sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>

using namespace argos;

class CFootBotMy : public CCI_Controller {

public:

   CFootBotMy();

   virtual ~CFootBotMy() {}

   virtual void Init(TConfigurationNode& t_node);

   virtual void ControlStep();
   virtual int level();

   virtual void Reset() {}
 // 
   virtual void Destroy() {}

   // scan the target
   virtual bool Find_Target();
   
   // scan the link to the target
   virtual void Join_link();

   // the tail will decide which bot is the next tail
   virtual int NextTail();

   // send messages about the current state
   // 0: current_Target_link
   // 1: total_Target_link
   // 2: current_character
   // 3: link_finish
   // 4: current bots id
   // 5: the id of bot become next tail
   // 6: the check signal
   virtual void Broadcast();

   virtual void ReceiveMessage();

   virtual void TailMovement();

   virtual void linkMovement();

   virtual void runnerMovement();

   virtual void maintainerMovement();

   virtual void repairMovement();

protected:
   /* Wheel speed. */
   Real m_fWheelVelocity;
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
   // set 180 degree threshold to know the closest bot is on righ or left
   CRadians AngleThreshold;

   CRadians cAngle;

   CRadians closest_ang;
   CRadians averageAngle;

   /* The random number generator */
   CRandom::CRNG* m_pcRNG;

   // the counter for runner walk randomly
   UInt32 m_unCounter;

   // the counter for malfunction
   UInt32 func_Counter;

   /* The counter range */
   CRange<UInt32> m_cCountRange;

   /* 4 characters: 0=runner, 1=maintainer, 2=link, 3=tail*/
   UInt32 character;
   UInt32 id;

   // the statement of maintainer, repairing or relaxing
   bool repairing;
   // the bot maintainer is surrounding, number is the order in the link
   UInt32 repair_bot;


private:
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_FootBotProximitySensor* m_pcProximity;
   CDegrees m_cAlpha;
   Real m_fDelta;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> m_cGoStraightAngleRange;
   CCI_LEDsActuator* m_pcLEDs;
   Real TargetDistance;

   /* Pointer to the foot-bot motor ground sensor */
   CCI_FootBotMotorGroundSensor* m_pcGround;

   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor* m_pcLight;

   // check whether the bot is link with target or base
   bool link_target;
   bool link_base;
   bool link_finish;
   bool check_signal;


   // the total length of link and current number of this bot
   // 0 means not in link,
   UInt32 current_Target_link;
   UInt32 total_Target_link;
   UInt32 current_Base_link;
   UInt32 total_Base_link;
};

class Behavior: public CFootBotMy{
public:
   void move();
};

class Obstacle : public Behavior {
public:
   void move(CCI_DifferentialSteeringActuator* m_pcWheels, CRadians cAngle){
      //std::cout << "level 1" << std::endl;
      //std::cout << cAngle.GetValue() << std::endl;
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
         //std::cout << "turn right" << std::endl;
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
         //std::cout << "turn left" << std::endl;
      }
   }
};

class Sparse : public Behavior {
public:
   void move(CCI_DifferentialSteeringActuator* m_pcWheels, CRadians averageAngle) {
      if(averageAngle > AngleThreshold){
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
         //std::cout << "level 2: Moving away other bots" << std::endl;
         //std::cout << "turn right" << std::endl;
      }else{
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
         //std::cout << "level 2: Moving away other bots" << std::endl;
         //std::cout << "turn left" << std::endl;
      }
   }
};

class Random : public Behavior {
public:
   void move(CCI_DifferentialSteeringActuator* m_pcWheels, Real random){
      /* Go straight */
      //std::cout << random << std::endl;
      if(random > 2){
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
         //std::cout << "level 3: Go Stright Search" << std::endl;
      }else if(random > 0){
         //std::cout << "level 3: left" << std::endl;
         m_pcWheels->SetLinearVelocity(2*m_fWheelVelocity, -2*m_fWheelVelocity);
      }else{
         //td::cout << "level 3: right" << std::endl;
         m_pcWheels->SetLinearVelocity(-2*m_fWheelVelocity, 2*m_fWheelVelocity);
      }
   }
};



 
#endif