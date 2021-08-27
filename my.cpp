/* Include the controller definition */
#include "my.h"
#include "iostream"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/vector2.h>
#include<string>
#include<cstring> 
CFootBotMy::CFootBotMy() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(25.0f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)),
   m_pcGround(NULL),
   m_pcCamera(NULL),
   m_unCounter(0),
   m_cCountRange(0, 2),
   m_pcLight(NULL),
   character(0),
   func_Counter(0),
   check_signal(false),
   repairing(false),
   repair_bot(0) {}


void CFootBotMy::Init(TConfigurationNode& t_node) {

   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
   m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
   m_pcGround    = GetSensor  <CCI_FootBotMotorGroundSensor    >("footbot_motor_ground" );
   m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );

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

   m_pcRNG = CRandom::CreateRNG("argos");
   m_unCounter = 0;

   link_target = false;
   link_base = false;
   link_finish = false;

   total_Target_link = 0;
   total_Base_link = 0;
   current_Target_link = 0;
   current_Base_link = 0;
   character = 0;
   std::string c_id = CCI_Controller::GetId().c_str();
   id = std::stoi(c_id.substr(2,c_id.length()));
   check_signal = false;
   repairing = false;
   repair_bot = 0;
}

int CFootBotMy::level() {
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
   cAngle = cAccumulator.Angle();
   //std::cout << cAccumulator.Length() << std::endl;
   if(!( m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta) ) {
      // detected obstacle, the first level
      return 1;
   }

   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
 

   // second level
   if(! sReadings.BlobList.empty()) {
         
      double closest_dis = sReadings.BlobList[0]->Distance;
      closest_ang = sReadings.BlobList[0]->Angle;
      averageAngle = sReadings.BlobList[0]->Angle;

      // find the closest bot
      for(size_t i = 1; i < sReadings.BlobList.size(); ++i) {
         if(sReadings.BlobList[i]->Distance < closest_dis){
            closest_dis = sReadings.BlobList[i]->Distance;
            closest_ang = sReadings.BlobList[i]->Angle;
         }
         averageAngle += sReadings.BlobList[i]->Angle;
      }
      
      if(closest_dis < TargetDistance){
         return 2;
      }       
   }


   return 3;
}

void CFootBotMy::ControlStep() {

   func_Counter++;

   if(func_Counter >= 1800 && id == 17){
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
      m_pcRABA->SetData(0, 0);
      m_pcRABA->SetData(1, 0);
      m_pcRABA->SetData(2, 0);
      m_pcRABA->SetData(3, false);
      m_pcRABA->SetData(4, 0);
      m_pcRABA->SetData(5, 0);
      m_pcRABA->SetData(6, false);
      m_pcLEDs->SetSingleColor(12, CColor::RED);
      return;
   }



   // if(character == 3){
   //    std::cout << id  << " is tail" << std::endl;
   // }

   Broadcast();
   ReceiveMessage();
   switch(character){
      case 0:
         runnerMovement();
         break;
      case 1:
         if(repairing){
            repairMovement();
            return;
         }else{
            maintainerMovement();
            return;
         }
      case 2:
         linkMovement();
         return;
      case 3:
         //std::cout << id  << " is tail" << std::endl;
         if(!(current_Target_link == 1)){
            TailMovement();
            //std::cout << id  << " is moving to light" << std::endl;
         }
         return;
   }

   Find_Target();

   if(link_target || link_finish){
      return;
   }


}


bool CFootBotMy::Find_Target() {

   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();

   if(tGroundReads[0].Value < 0.25f ||
      tGroundReads[1].Value < 0.25f ||
      tGroundReads[2].Value < 0.25f ||
      tGroundReads[3].Value < 0.25f) {   
      // when the bot finds target, it becomes guardian
      character = 3;
      link_target = true;
      current_Target_link = 1;
      total_Target_link =1;

      // found the target to change the color and stop moving
      m_pcLEDs->SetSingleColor(12, CColor::GREEN);
      m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
   }

   return link_target;
}

// choose the bots with smallest id be the next tail
int CFootBotMy::NextTail() {
   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
   UInt32 next_id = -1;

   for(size_t i = 0; i < tPackets.size(); ++i) {
      if((next_id > tPackets[i].Data[4]) && (tPackets[i].Data[2] == 0) ){
         next_id = tPackets[i].Data[4];
      }
   }

   //std::cout << "next tail" << next_id << std::endl;
   return next_id;
}

void CFootBotMy::Broadcast() {

   m_pcRABA->SetData(0, current_Target_link);
   m_pcRABA->SetData(1, total_Target_link);
   m_pcRABA->SetData(2, character);
   m_pcRABA->SetData(3, link_finish);
   m_pcRABA->SetData(4, id);
   if (character == 3 && (!link_finish)){
      int ni = NextTail();
      m_pcRABA->SetData(5, ni);
   }else{
      m_pcRABA->SetData(5, -1);
   }

   // if the bot is the first in line, send check signal
   if(character == 2 && current_Target_link == 1){
      check_signal = true;
      m_pcRABA->SetData(6, check_signal);
   }

   // if the bot is in line but not the first, only send true when previous has sent true
   // then it becomes false and waiting for next check signal from previous
   if(character == 2 && check_signal){
      m_pcRABA->SetData(6, check_signal);
      m_pcLEDs->SetSingleColor(12, CColor::GREEN);
      check_signal = false;
   }else{
      m_pcRABA->SetData(6, check_signal);
   }
}

// 1. Found link and join the link at tail
// 2. Update total number of link bots
void CFootBotMy::ReceiveMessage() {

   // if the current bot's number is smaller than link length, changes the character into  link
   if(character == 3 && (total_Target_link > current_Target_link)) {
      character = 2;
   }

   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();

   // update the link length
   for(size_t i = 0; i < tPackets.size(); ++i) {
      if(tPackets[i].Data[1] > total_Target_link){
         total_Target_link = tPackets[i].Data[1];
         //std::cout << total_Target_link << tPackets[i].Data[1] << std::endl;
         break;
      }
   }

}

// join the link, if the bot also in base, finish the search and stop 
void CFootBotMy::Join_link(){
   link_target = true;
   m_pcLEDs->SetSingleColor(12, CColor::GREEN);

   m_pcWheels->SetLinearVelocity(0.0f, 0.0f);

   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();

   // anyone ground sensor detected grey floor means search finished
   if((tGroundReads[0].Value > 0.25f && tGroundReads[0].Value < 0.75f) ||
      (tGroundReads[1].Value > 0.25f && tGroundReads[1].Value < 0.75f) ||
      (tGroundReads[2].Value > 0.25f && tGroundReads[2].Value < 0.75f) ||
      (tGroundReads[3].Value > 0.25f && tGroundReads[3].Value < 0.75f)){
      link_finish = true;
   }
}

void CFootBotMy::TailMovement(){
   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();

   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
   
   if(! tPackets.empty()) {
      for(size_t i = 0; i < tPackets.size(); ++i) {
         // look for the previous bot
         // the previous bot sent check signal as true
         if(tPackets[i].Data[6] && (tPackets[i].Data[0] == (current_Target_link - 1))){
            // std::cout << tPackets[i].Data[4] << std::endl;
            std::cout << "connection good" << std::endl;
            break;
         }else if(!tPackets[i].Data[6] && (tPackets[i].Data[0] == (current_Target_link - 1))){
            // std::cout << tPackets[i].Data[6] << "  " << tPackets[i].Data[0] << std::endl;
            std::cout << "connection bad" << std::endl;
            break;
         }
         
      }
   }

   // anyone ground sensor detected grey floor means search finished
   if((tGroundReads[0].Value > 0.25f && tGroundReads[0].Value < 0.75f) ||
      (tGroundReads[1].Value > 0.25f && tGroundReads[1].Value < 0.75f) ||
      (tGroundReads[2].Value > 0.25f && tGroundReads[2].Value < 0.75f) ||
      (tGroundReads[3].Value > 0.25f && tGroundReads[3].Value < 0.75f)){
      //std::cout << "link_finish" << std::endl;
      link_finish = true;
      m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
      return;
   }

   // get the distance from previous bot, if it's too far, stop moving

   if(! tPackets.empty()) {
      for(size_t i = 0; i < tPackets.size(); ++i) {
         // look for the previous bot
         if(tPackets[i].Data[0] == (current_Target_link - 1)){
            Real pre_distance = tPackets[i].Range;
            // if the distance between bots is 200 cm or further, stop moving
            if(pre_distance >= 200){
               //std::cout << pre_distance << std::endl;
               m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
               return;
            }
         }
      }
   }

   //Move to light if the distance to previous bots isn't too far
   CVector2 Temp;
   const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tLightReads.size(); ++i) {
      cAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
   }
   /* If the light was perceived, return the vector */
   if(cAccumulator.Length() > 0.0f) {
      Temp = CVector2(1.0f, cAccumulator.Angle());
      //std::cout << Temp << std::endl;
      if( !( m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(Temp.Angle().SignedNormalize()))) {
         if(Temp.Angle().GetValue() < 0.0f) {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
            //std::cout << "turn right" << std::endl;
         }
         else {
            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
            //std::cout << "turn left" << std::endl;
         }
      }else{
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
      }
   }

}

void CFootBotMy::linkMovement(){

   if(current_Target_link == 1){
      m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
      return;
   }
   
   // if bot is not too far from previous, it will move toward next bot
   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();

   int previous_i = -1;
   int next_i = -1;

   if(! tPackets.empty()) {
      for(size_t i = 0; i < tPackets.size(); ++i) {
         // look for the previous bot
         if(tPackets[i].Data[0] == (current_Target_link - 1)){
            previous_i = i;
         }
         if(tPackets[i].Data[0] == (current_Target_link + 1)){
            next_i = i;
         }

         // the previous bot sent check signal as true
         if(tPackets[i].Data[6] && (tPackets[i].Data[0] == (current_Target_link - 1))){
            check_signal = true;
            m_pcLEDs->SetSingleColor(12, CColor::BLUE);
         }else if(!tPackets[i].Data[6] && (tPackets[i].Data[0] == (current_Target_link - 1))){
            check_signal = false;
         }
      }
   }

   if((previous_i <= 0 )||( next_i <= 0)) {
      m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
      return;
   }

   Real pre_distance = tPackets[previous_i].Range;
   // if the distance between bots is 200 cm or closer, move toward next bot
   if(pre_distance <= 130){
      //std::cout << pre_distance << std::endl;
      //std::cout << id << "moving to next" << std::endl;

      CRadians Angle = tPackets[next_i].HorizontalBearing;
      if( !( m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(Angle.SignedNormalize()))) {
         if(Angle.GetValue() < 0.0f) {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
            //std::cout << "turn right" << std::endl;
         }
         else {
            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
            //std::cout << "turn left" << std::endl;
         }
      }else{
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
      }

   }else{
      m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
   }
}

void CFootBotMy::runnerMovement(){

   m_pcLEDs->SetSingleColor(12, CColor::RED);

   Obstacle obs;
   Sparse spa;
   Random ran;

   switch(CFootBotMy::level()){
      case 1:
         obs.move(m_pcWheels, cAngle);
         break;
      case 2:
         spa.move(m_pcWheels, averageAngle);
         break;
      default:
         if(m_unCounter%100 == 0){
            ran.move(m_pcWheels, m_pcRNG->Uniform(m_cCountRange));
            m_unCounter = 0;
         }else{
            ran.move(m_pcWheels, 3);
         }
   }
   m_unCounter ++;

   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();

   for(size_t i = 0; i < tPackets.size(); ++i) {
      // if the link has been built, runner stop wondering
      if(tPackets[i].Data[3]){
         character = 1;
         m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
         return;
      }

      // if founds the tail, join the link
      if((tPackets[i].Data[2] == 3)&&(tPackets[i].Data[5] == id)){
         if(tPackets[i].Range <= 130){
            Join_link();
            // change the character to tail
            character = 3;
            current_Target_link = tPackets[i].Data[0] + 1;
            total_Target_link = current_Target_link;
            break;
         }else{
            //std::cout << tPackets[i].Range << std::endl;
            //std::cout << id << "Moving closer" << std::endl;
            // move closer if they are too far
            CRadians Angle = tPackets[i].HorizontalBearing;
            if( !( m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(Angle.SignedNormalize()))){
               if(Angle.GetValue() < 0.0f) {
                  m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                  //std::cout << "turn right" << std::endl;
               }
               else {
                  m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                  //std::cout << "turn left" << std::endl;
               }
            }else{
               m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
            }
            break;
         }
      }
   }
}

void CFootBotMy::maintainerMovement(){

   m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
   m_pcLEDs->SetSingleColor(12, CColor::WHITE);

   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();


   if(! tPackets.empty()) {

      // the maintainer will become runner if it doesn't get messages from tail
      bool tail_sig = false;

      for(size_t i = 0; i < tPackets.size(); ++i) {
         // the maintainer will keep near the tail to get newest message        
         if(tPackets[i].Data[2] == 3){
            tail_sig = true;
            if(tPackets[i].Range >= 60){
               CRadians Angle = tPackets[i].HorizontalBearing;
               if( !( m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(Angle.SignedNormalize()))) {
                  if(Angle.GetValue() < 0.0f) {
                     m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
                     //std::cout << "turn right" << std::endl;
                  }
                  else {
                     m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
                     //std::cout << "turn left" << std::endl;
                  }
               }else{
                  m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
               }

            }else{
               m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
            }

            if(!tPackets[i].Data[6] && tPackets[i].Range <= 100){
               repairing = true;
               m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
            }
         }
      }

      if(!tail_sig){
         character = 0;
      }
   }
   
}

void CFootBotMy::repairMovement(){
   
   m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();

   m_pcLEDs->SetSingleColor(12, CColor::YELLOW);


   // the rebuilding connection part
   if(! tPackets.empty()) {
      for(size_t i = 0; i < tPackets.size(); ++i) {
         // if another bot is sending check signal and further up the order
         // maintainer join the link
         if(tPackets[i].Data[6] && (tPackets[i].Data[0] < repair_bot) && (!tPackets[repair_bot].Data[6])){
            Join_link();
            // change the character to tail
            character = 2;
            current_Target_link = tPackets[i].Data[0] + 1;
            return;
         }

      }

   }



   // the searching and following part
   if(! tPackets.empty()) {

      UInt32 most_pre = tPackets.size() + 1;

      // find if there is a robot in link around
      for(size_t i = 0; i < tPackets.size(); ++i) {

         // the maintainer will stop repair if they can get check signal      
         if(tPackets[i].Data[6]){
            //std::cout << tPackets[i].Data[4] << std::endl;
            repairing = false;
         }

         if(tPackets[i].Data[2] == 2){
            most_pre = i;
            break;
         }
      }

      if(most_pre > tPackets.size()){
         return;
      }


      for(size_t i = 0; i < tPackets.size(); ++i) {


         if((tPackets[i].Data[2] == 2) && (tPackets[i].Data[0] < tPackets[most_pre].Data[0]) && (tPackets[i].Data[0]>0)){
            most_pre = i;
         }

      }

      repair_bot = tPackets[most_pre].Data[0];

      if((tPackets[most_pre].Data[2] == 2) && (most_pre < tPackets.size()) && (tPackets[most_pre].Range <= 200)){
         //std::cout << id << "  "<< tPackets[most_pre].Data[2] << "  " << tPackets[most_pre].Data[4] << std::endl;
         CRadians Angle = tPackets[most_pre].HorizontalBearing;
         if(( m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(Angle.SignedNormalize()))) {
            if(Angle.GetValue() > 0.0f) {
               m_pcWheels->SetLinearVelocity(m_fWheelVelocity/2, 0.0f);
               //std::cout << "turn right" << std::endl;
            }
            else {
               m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity/2);
               //std::cout << "turn left" << std::endl;
            }
         }else{
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
         }

      }else{
         m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
         CRadians Angle = tPackets[most_pre].HorizontalBearing;
         if(!( m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(Angle.SignedNormalize()))) {
            if(Angle.GetValue() < 0.0f) {
               m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
               //std::cout << "turn right" << std::endl;
            }
            else {
               m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
               //std::cout << "turn left" << std::endl;
            }
         }else{
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
         }
      }
   }


}

REGISTER_CONTROLLER(CFootBotMy, "my_controller")
