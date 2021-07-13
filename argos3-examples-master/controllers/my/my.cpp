/* Include the controller definition */
#include "my.h"
#include "iostream"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/vector2.h>

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(25.0f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)),
   m_pcCamera(NULL) {}


void CFootBotDiffusion::Init(TConfigurationNode& t_node) {

   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );

   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
   
   m_pcLEDs   = GetActuator<CCI_LEDsActuator>("leds");
   m_pcCamera = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
   m_pcCamera->Enable();
   
   CDegrees cAngle;
   GetNodeAttribute(t_node, "threshold", cAngle);
   AngleThreshold = ToRadians(cAngle);
   
   GetNodeAttribute(t_node, "distanceThreshold", TargetDistance);
}

void CFootBotDiffusion::ControlStep() {

   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetSingleColor(12, CColor::RED);
   
   bool obstacle = false;
   bool away = false;

//----------------first level for deceting obstacle---------------------------
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   CRadians cAngle = cAccumulator.Angle();
   if(!( m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta) ) {
      // detected obstacle, the first level
      obstacle = true;
      std::cout << "level 1: Avoid Obstacle" << std::endl;
      /* Turn, depending on the sign of the angle */
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      }
   }
   if(obstacle){
      return ;
   }
   
//----------------second level for moving away closest bot---------------------------
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
   if(! sReadings.BlobList.empty()) {
       
       double closest_dis = sReadings.BlobList[0]->Distance;
       CRadians closest_ang = sReadings.BlobList[0]->Angle;
       
       // find the closest bot
       for(size_t i = 1; i < sReadings.BlobList.size(); ++i) {
           if(sReadings.BlobList[i]->Distance < closest_dis){
	       closest_dis = sReadings.BlobList[i]->Distance;
	       closest_ang = sReadings.BlobList[i]->Angle;
	   }
       }
       
       if(closest_dis < TargetDistance && closest_ang > AngleThreshold){
          m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
          std::cout << "level 2: Moving away other bots" << std::endl;
          std::cout << "turn right" << std::endl;
          away = true;
       }else if(closest_dis < TargetDistance){
          m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
          std::cout << "level 2: Moving away other bots" << std::endl;
          std::cout << "turn left" << std::endl;
          away = true;
       }       
   }
   if(away){
      return ;
   }
   
//----------------third level for going straight---------------------------
   /* Go straight */
   std::cout << "level 3: Go Stright Search" << std::endl;
   m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
}




REGISTER_CONTROLLER(CFootBotDiffusion, "my_controller")
