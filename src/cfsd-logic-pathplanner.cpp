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

#include <Eigen/Core>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <iterator>

#include "collector.hpp"
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
    uint32_t localPathSenderId = 2601;

    Collector collector(commandlineArguments);

    cluon::OD4Session od4{
      static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    auto onObjectFrameStart{[&planner = collector](
          cluon::data::Envelope &&envelope)
    {
      // TODO: for now assume senderStamp 0 for perception (add SLAM later)
      if (envelope.senderStamp() == 0) {
        planner.getObjectFrameStart(envelope);
      }
    }};

    auto onObjectFrameEnd{[&planner = collector](
        cluon::data::Envelope &&envelope)
    {
      // TODO: for now assume senderStamp 0 for perception (add SLAM later)
      if (envelope.senderStamp() == 0) {
        planner.getObjectFrameEnd(envelope);
      }
    }};

    auto onObject{[&planner = collector](
        cluon::data::Envelope &&envelope)
    {
      planner.getObject(envelope);
    }};

    auto onObjectType{[&planner = collector](
        cluon::data::Envelope &&envelope)
    {
      planner.getObjectType(envelope);
    }};

    auto onObjectPosition{[&planner = collector](
        cluon::data::Envelope &&envelope)
    {
      planner.getObjectPosition(envelope);
    }};

    auto onEquilibrioception{[&planner = collector](
        cluon::data::Envelope &&envelope)
    {
      planner.getEquilibrioception(envelope);
    }};
    
    auto atFrequency{[&od4, &verbose, &collector, &localPathSenderId]() -> bool
      {
        {
          collector.GetCompleteFrameCFSD19();
          

          if (verbose) {
            uint64_t frameDuration = cluon::time::toMicroseconds(collector.frameEnd)
              - cluon::time::toMicroseconds(collector.frameStart);
            std::cout << "Using frame to find path, frame duration="
              << frameDuration << ", number of cones=" << collector.m_currentConeFrame.size()
              << std::endl;
          }
          
          collector.ProcessFrameCFSD19();
        }

        uint32_t length = collector.middlePath.size();
        std::string data;
        if(collector.middlePath.size()){
        for (auto c : collector.middlePath) {
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

        return true;
    }};


    od4.dataTrigger(opendlv::logic::perception::ObjectFrameStart::ID(),
        onObjectFrameStart);
    od4.dataTrigger(opendlv::logic::perception::ObjectFrameEnd::ID(),
        onObjectFrameEnd);
    od4.dataTrigger(opendlv::logic::perception::Object::ID(),
        onObject);
    od4.dataTrigger(opendlv::logic::perception::ObjectType::ID(),
        onObjectType);
    od4.dataTrigger(opendlv::logic::perception::ObjectPosition::ID(),
        onObjectPosition);
    od4.dataTrigger(opendlv::logic::sensation::Equilibrioception::ID(),
        onEquilibrioception);
    od4.timeTrigger(std::stoi(commandlineArguments["freq"]), atFrequency);
    
    retCode = 0;
  }
  return retCode;
}
