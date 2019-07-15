/**
* Copyright (C) 2017 Chalmers Revere
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
* USA.
*/

#include "collector.hpp"
/*-------------------------------------------------------*/
Collector::Collector(PathPlanner &planner, std::map<std::string, std::string> commandlineArguments) :
    m_planner(planner),
    m_currentConeFrame{},
    
    uncompleteFrameMutex(),
    uncompleteFrame{},
    currentUncompleteFrameId(),

    completeFrameMutex(),
    completeFrame{},
    currentCompleteFrameId(),

    frameStart(),
    frameEnd(),
    
    currentAim(),
    m_verbose(),
    m_debug()
    
{
  m_verbose = static_cast<bool>(commandlineArguments.count("verbose") != 0);
  m_debug = static_cast<bool>(commandlineArguments.count("debug") != 0);
}

void Collector::GetCompleteFrameCFSD19(){
  //Copy cones to m_currentConeFrame for processing
  std::lock_guard<std::mutex> lock(completeFrameMutex);
  if(completeFrame.size())
  {
    for(auto elem : completeFrame)
	    m_planner.m_currentConeFrame.push(elem.second);
  }
}

void Collector::getObjectFrameStart(cluon::data::Envelope envelope){
        opendlv::logic::perception::ObjectFrameStart msg = 
          cluon::extractMessage<opendlv::logic::perception::ObjectFrameStart>(
              std::move(envelope));

        uint32_t objectFrameId = msg.objectFrameId();
        {
          std::lock_guard<std::mutex> lock(uncompleteFrameMutex);
          uncompleteFrame = std::map<uint32_t, Cone>();
          currentUncompleteFrameId = objectFrameId;
          frameStart = envelope.sent();
        }

        if (m_verbose) {
          std::cout << "Got frame START with id=" << objectFrameId
            << std::endl;
        }
}

void Collector::getObjectFrameEnd(cluon::data::Envelope envelope){
  opendlv::logic::perception::ObjectFrameEnd msg = 
          cluon::extractMessage<opendlv::logic::perception::ObjectFrameEnd>(
              std::move(envelope));

        uint32_t objectFrameId = msg.objectFrameId();
        {
          std::lock_guard<std::mutex> lock1(completeFrameMutex);
          std::lock_guard<std::mutex> lock2(uncompleteFrameMutex);
          completeFrame = uncompleteFrame;
          
          currentCompleteFrameId = currentUncompleteFrameId;
          frameEnd = envelope.sent();
        }

        if (m_verbose) {
          std::cout << "Got frame END with id=" << objectFrameId 
            << std::endl;
        }
}

void Collector::getObject(cluon::data::Envelope envelope){
  opendlv::logic::perception::Object msg = 
        cluon::extractMessage<opendlv::logic::perception::Object>(
            std::move(envelope));

      uint32_t objectId = msg.objectId();
      {
        std::lock_guard<std::mutex> lock(uncompleteFrameMutex);

      }
      if (m_verbose) {
        
      }
}

void Collector::ProcessFrameCFSD19(){
  m_planner.ProcessFrameCFSD19();
}

void Collector::getObjectType(cluon::data::Envelope envelope){
  opendlv::logic::perception::ObjectType msg = 
        cluon::extractMessage<opendlv::logic::perception::ObjectType>(
            std::move(envelope));

      uint32_t objectId = msg.objectId();
      {
        std::lock_guard<std::mutex> lock(uncompleteFrameMutex);
        Cone newCone(objectId, 0, 0, 0);
        uncompleteFrame[objectId] = newCone;
        std::cout << "Got NEW OBJECT with id=" << objectId << std::endl;
        
        if (uncompleteFrame.count(objectId)) {
          uncompleteFrame[objectId].m_color = msg.type();
        }
      }
      if (m_verbose) {
        std::cout << "Got OBJECT TYPE for object with id=" << objectId 
          << " and type=" << msg.type() << std::endl;
      }
}

void Collector::getObjectPosition(cluon::data::Envelope envelope){
  opendlv::logic::perception::ObjectPosition msg = 
        cluon::extractMessage<opendlv::logic::perception::ObjectPosition>(
            std::move(envelope));

      uint32_t objectId = msg.objectId();
      {
        std::lock_guard<std::mutex> lock(uncompleteFrameMutex);
        if (uncompleteFrame.count(objectId)) {
        // remap the coordiate so that the coordinate is as follow in path planner
        // from perceptiom:
        // x: positive upward vertically
        // y: positive to left horizontally
        
        // In path planner
        // x: postive to right horizontally
        // y: positive upward vertically
          uncompleteFrame[objectId].m_x = -msg.y();
          uncompleteFrame[objectId].m_y = msg.x();
        }
      }
      if (m_verbose) {
        std::cout << "Got OBJECT POSITION for object with id=" << objectId 
          << " and x=" << msg.x() << " y=" << msg.y() << std::endl;
      }
}

void Collector::getEquilibrioception(cluon::data::Envelope envelope){
  opendlv::logic::sensation::Equilibrioception msg = 
        cluon::extractMessage<opendlv::logic::sensation::Equilibrioception>(
            std::move(envelope));

      float vx = msg.vx();
      float yawRate = msg.yawRate();
      if (m_verbose) {
        std::cout << "Got EQUILIBRIOCEPTION vx=" << vx << " and yawRate=" 
          << yawRate << std::endl;
      }
}

void Collector::getAimpoint(cluon::data::Envelope envelope){
  opendlv::logic::action::AimPoint msg = 
        cluon::extractMessage<opendlv::logic::action::AimPoint>(
            std::move(envelope));

      float angle = msg.azimuthAngle();
      float distance = msg.distance();
      m_planner.currentAim.x = distance*cos(angle);
      m_planner.currentAim.y = distance*sin(angle);
      if (m_verbose) {
        std::cout << "Got AIMPOINT for object with id=" << 0 
          << " and x=" << m_planner.currentAim.x << " y=" << m_planner.currentAim.y << std::endl;
      }
}
