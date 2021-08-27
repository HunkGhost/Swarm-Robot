#include "foraging_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_foraging/footbot_foraging.h>
 
/****************************************/
/****************************************/
 
CForagingLoopFunctions::CForagingLoopFunctions() :
   m_cForagingArenaSideX(6.0f, 7.0f),
   m_cForagingArenaSideY(-5.0f, 7.0f),
   m_pcFloor(NULL),
   m_pcRNG(NULL){
}
 
/****************************************/
/****************************************/
 
void CForagingLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tForaging = GetNode(t_node, "foraging");

      UInt32 unFoodItems;
      GetNodeAttribute(tForaging, "items", unFoodItems);
      /* Get the number of food items we want to be scattered from XML */
      GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
      m_fFoodSquareRadius *= m_fFoodSquareRadius;
      /* Create a new RNG */
      m_pcRNG = CRandom::CreateRNG("argos");
      /* Distribute uniformly the items in the environment */
      for(UInt32 i = 0; i < unFoodItems; ++i) {
         m_cFoodPos.push_back(
            CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                     m_pcRNG->Uniform(m_cForagingArenaSideY)));
      }
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
}
 
/****************************************/
/****************************************/
 
void CForagingLoopFunctions::Reset() {

}
 
/****************************************/
/****************************************/
 
void CForagingLoopFunctions::Destroy() {

}
 
/****************************************/
/****************************************/
 
CColor CForagingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   if(c_position_on_plane.GetX() < -4.0f) {
      return CColor::GRAY50;
   }
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      if((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
         return CColor::BLACK;
      }
   }
   return CColor::WHITE;
}
 
/****************************************/
/****************************************/
 
void CForagingLoopFunctions::PreStep() {
}
 
/****************************************/
/****************************************/
 
REGISTER_LOOP_FUNCTIONS(CForagingLoopFunctions, "foraging_loop_functions")
