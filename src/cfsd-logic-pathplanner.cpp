/*
 * Copyright (C) 2019 Nam Vu
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <cstdint>
#include <iostream>
#include <memory>
#include <iterator>

struct Cone {
  uint32_t objectId;
  uint32_t type;
  float i;
  float j;
  float w;
  float h;
  float x;
  float y;
  uint32_t collectedComponents;
};

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ( (0 == commandlineArguments.count("cid")) ||
      (0 == commandlineArguments.count("freq")) )  {
    std::cerr << argv[0] << " creates a path based on objec data input." 
      << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> "
      << "--freq=<Frequency> [--verbose]" << std::endl;
  } else {
    bool const verbose{commandlineArguments.count("verbose") != 0};
    //uint32_t localPathSenderId = 2601;

    cluon::OD4Session od4{
      static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    std::mutex uncompleteFrameMutex;
    std::map<uint32_t, Cone> uncompleteFrame;
    uint32_t currentUncompleteFrameId;
    
    std::mutex completeFrameMutex;
    std::map<uint32_t, Cone> completeFrame;
    uint32_t currentCompleteFrameId;
    
    cluon::data::TimeStamp frameStart;
    cluon::data::TimeStamp frameEnd;

    auto onObjectFrameStart{[&uncompleteFrameMutex, &uncompleteFrame,
      &currentUncompleteFrameId, &frameStart](
          cluon::data::Envelope &&envelope)
    {
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
    }};

    auto onObjectFrameEnd{[&uncompleteFrameMutex, &uncompleteFrame,
      &currentUncompleteFrameId, &completeFrameMutex, &completeFrame,
      &currentCompleteFrameId, &frameEnd](cluon::data::Envelope &&envelope)
    {
      opendlv::logic::perception::ObjectFrameEnd msg = 
        cluon::extractMessage<opendlv::logic::perception::ObjectFrameEnd>(
            std::move(envelope));

      uint32_t objectFrameId = msg.objectFrameId();
      if (objectFrameId == currentUncompleteFrameId) {
        std::lock_guard<std::mutex> lock1(completeFrameMutex);
        std::lock_guard<std::mutex> lock2(uncompleteFrameMutex);

        completeFrame = std::map<uint32_t, Cone>();
        for (auto obj : uncompleteFrame) {
          if (obj.second.collectedComponents == 4) {
            completeFrame[obj.first] = obj.second;
          } else {
            std::cout << "Warning: Expected 4 collected components for object "
              << "(id=" << obj.first << ") but found " 
              << obj.second.collectedComponents << " (discarded)." << std::endl;
          }
        }
        
        currentCompleteFrameId = currentUncompleteFrameId;
        frameEnd = envelope.sent();
      }
        
      uncompleteFrame = std::map<uint32_t, Cone>();
    }};

    auto onObjectType{[&uncompleteFrameMutex, &uncompleteFrame](
        cluon::data::Envelope &&envelope)
    {
      opendlv::logic::perception::ObjectType msg = 
        cluon::extractMessage<opendlv::logic::perception::ObjectType>(
            std::move(envelope));

      uint32_t objectId = msg.objectId();
      {
        std::lock_guard<std::mutex> lock(uncompleteFrameMutex);

        if (uncompleteFrame.count(objectId)) {
          uncompleteFrame[objectId].type = msg.type();
          uncompleteFrame[objectId].collectedComponents++;
        } else {
          Cone cone;
          cone.objectId = objectId;
          cone.type = msg.type();
          cone.collectedComponents = 1;
          uncompleteFrame[objectId] = cone;
        }
      }
    }};

    auto onObjectPosition{[&uncompleteFrameMutex, &uncompleteFrame](
        cluon::data::Envelope &&envelope)
    {
      opendlv::logic::perception::ObjectPosition msg = 
        cluon::extractMessage<opendlv::logic::perception::ObjectPosition>(
            std::move(envelope));

      uint32_t objectId = msg.objectId();
      {
        std::lock_guard<std::mutex> lock(uncompleteFrameMutex);

        if (uncompleteFrame.count(objectId)) {
          uncompleteFrame[objectId].x = msg.x();
          uncompleteFrame[objectId].y = msg.y();
          uncompleteFrame[objectId].collectedComponents++;
        } else {
          Cone cone;
          cone.objectId = objectId;
          cone.x = msg.x();
          cone.y = msg.y();
          cone.collectedComponents = 1;
          uncompleteFrame[objectId] = cone;
        }
      }
    }};
    
    auto onObjectDirection{[&uncompleteFrameMutex, &uncompleteFrame](
        cluon::data::Envelope &&envelope)
    {
      opendlv::logic::perception::ObjectDirection msg = 
        cluon::extractMessage<opendlv::logic::perception::ObjectDirection>(
            std::move(envelope));

      uint32_t objectId = msg.objectId();
      {
        std::lock_guard<std::mutex> lock(uncompleteFrameMutex);

        if (uncompleteFrame.count(objectId)) {
          uncompleteFrame[objectId].i = msg.azimuthAngle();
          uncompleteFrame[objectId].j = msg.zenithAngle();
          uncompleteFrame[objectId].collectedComponents++;
        } else {
          Cone cone;
          cone.objectId = objectId;
          cone.i = msg.azimuthAngle();
          cone.j = msg.zenithAngle();
          cone.collectedComponents = 1;
          uncompleteFrame[objectId] = cone;
        }
      }
    }};
    
    auto onObjectAngularBlob{[&uncompleteFrameMutex, &uncompleteFrame](
        cluon::data::Envelope &&envelope)
    {
      opendlv::logic::perception::ObjectAngularBlob msg = 
        cluon::extractMessage<opendlv::logic::perception::ObjectAngularBlob>(
            std::move(envelope));

      uint32_t objectId = msg.objectId();
      {
        std::lock_guard<std::mutex> lock(uncompleteFrameMutex);

        if (uncompleteFrame.count(objectId)) {
          uncompleteFrame[objectId].w = msg.width();
          uncompleteFrame[objectId].h = msg.height();
          uncompleteFrame[objectId].collectedComponents++;
        } else {
          Cone cone;
          cone.objectId = objectId;
          cone.w = msg.width();
          cone.h = msg.height();
          cone.collectedComponents = 1;
          uncompleteFrame[objectId] = cone;
        }
      }
    }};

    auto onEquilibrioception{[](cluon::data::Envelope &&)
    {
    }};
    
    auto atFrequency{[&od4, &frameStart, &frameEnd, &completeFrameMutex,
      &completeFrame, &currentCompleteFrameId, &verbose]() -> bool
      {
        {
          std::lock_guard<std::mutex> lock(completeFrameMutex);

          // Do stuff...

          if (verbose) {
            uint64_t frameDuration = cluon::time::toMicroseconds(frameEnd)
              - cluon::time::toMicroseconds(frameStart);
            std::cout << "Using frame (id=" << currentCompleteFrameId 
              << ") to find aim point, frame duration=" << frameDuration 
              << ", numer of objects=" << completeFrame.size() <<  std::endl;
          }
        }

        /*
        uint32_t length = collector.m_planner.middlePath.size();
        std::string data;
        if(collector.m_planner.middlePath.size()){
        for (auto c : collector.m_planner.middlePath) {
          float x = (float)c.x;
          float y = (float)c.y;
          float z = 0.0f;

          char r[12];
          memcpy(r, &x, 4); 
          memcpy(r + 4, &y, 4); 
          memcpy(r + 8, &z, 4); 

          data += std::string(r, 12);
        }
        }
        cluon::data::TimeStamp ts = cluon::time::now();

        opendlv::logic::action::LocalPath localPath;
        localPath.length(length);
        localPath.data(data);
        od4.send(localPath, ts, localPathSenderId);
        */

        return true;
    }};


    od4.dataTrigger(opendlv::logic::perception::ObjectFrameStart::ID(),
        onObjectFrameStart);
    od4.dataTrigger(opendlv::logic::perception::ObjectFrameEnd::ID(),
        onObjectFrameEnd);
    od4.dataTrigger(opendlv::logic::perception::ObjectType::ID(),
        onObjectType);
    od4.dataTrigger(opendlv::logic::perception::ObjectPosition::ID(),
        onObjectPosition);
    od4.dataTrigger(opendlv::logic::perception::ObjectDirection::ID(),
        onObjectDirection);
    od4.dataTrigger(opendlv::logic::perception::ObjectAngularBlob::ID(),
        onObjectAngularBlob);
    od4.dataTrigger(opendlv::logic::sensation::Equilibrioception::ID(),
        onEquilibrioception);

    od4.timeTrigger(std::stoi(commandlineArguments["freq"]), atFrequency);
    
    retCode = 0;
  }
  return retCode;
}
