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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

    struct ConeObject {
      float x;
      float y;
      uint32_t type;
    };

    std::mutex uncompleteFrameMutex;
    std::vector<ConeObject> uncompleteFrame;

    std::mutex completeFrameMutex;
    std::vector<ConeObject> completeFrame;

    cluon::OD4Session od4{
      static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    auto onObjectFrameStart{[&uncompleteFrameMutex, &uncompleteFrame, &verbose](
        cluon::data::Envelope &&envelope)
    {
      // TODO: for now assume senderStamp 0 for perception (add SLAM later)
      if (envelope.senderStamp() == 0) {
        auto msg = 
          cluon::extractMessage<opendlv::logic::perception::ObjectFrameStart>(
              std::move(envelope));

        uint32_t objectFrameId = msg.objectFrameId();
        if (verbose) {
          std::cout << "Got frame START with id=" << objectFrameId
            << std::endl;
        }
      }
    }};

    auto onObjectFrameEnd{[&uncompleteFrameMutex, &uncompleteFrame, &verbose](
        cluon::data::Envelope &&envelope)
    {
      // TODO: for now assume senderStamp 0 for perception (add SLAM later)
      if (envelope.senderStamp() == 0) {
        auto msg = 
          cluon::extractMessage<opendlv::logic::perception::ObjectFrameEnd>(
              std::move(envelope));

        uint32_t objectFrameId = msg.objectFrameId();
        if (verbose) {
          std::cout << "Got frame END with id=" << objectFrameId 
            << std::endl;
        }
      }
    }};

    auto onObject{[&uncompleteFrameMutex, &uncompleteFrame, &verbose](
        cluon::data::Envelope &&envelope)
    {
      // TODO: for now assume senderStamp 0 for perception (add SLAM later)
      if (envelope.senderStamp() == 0) {
        auto msg = 
          cluon::extractMessage<opendlv::logic::perception::Object>(
              std::move(envelope));

        uint32_t objectId = msg.objectId();
        if (verbose) {
          std::cout << "Got NEW OBJECT with id=" << objectId << std::endl;
        }
      }
    }};

    auto onObjectType{[&uncompleteFrameMutex, &uncompleteFrame, &verbose](
        cluon::data::Envelope &&envelope)
    {
      // TODO: for now assume senderStamp 0 for perception (add SLAM later)
      if (envelope.senderStamp() == 0) {
        auto msg = 
          cluon::extractMessage<opendlv::logic::perception::ObjectType>(
              std::move(envelope));

        uint32_t objectId = msg.objectId();
        uint32_t type = msg.type();
        if (verbose) {
          std::cout << "Got OBJECT TYPE for object with id=" << objectId 
            << " and type=" << type << std::endl;
        }
      }
    }};

    auto onObjectPosition{[&uncompleteFrameMutex, &uncompleteFrame, &verbose](
        cluon::data::Envelope &&envelope)
    {
      // TODO: for now assume senderStamp 0 for perception (add SLAM later)
      if (envelope.senderStamp() == 0) {
        auto msg = 
          cluon::extractMessage<opendlv::logic::perception::ObjectPosition>(
              std::move(envelope));

        uint32_t objectId = msg.objectId();
        float x = msg.x();
        float y = msg.y();
        if (verbose) {
          std::cout << "Got OBJECT POSITION for object with id=" << objectId 
            << " and x=" << x << " y=" << y << std::endl;
        }
      }
    }};

    auto onEquilibrioception{[&uncompleteFrameMutex, &uncompleteFrame, &verbose](
        cluon::data::Envelope &&envelope)
    {
      // TODO: for now assume senderStamp 0 for perception (add SLAM later)
      if (envelope.senderStamp() == 0) {
        auto msg = 
          cluon::extractMessage<opendlv::logic::sensation::Equilibrioception>(
              std::move(envelope));

        float vx = msg.vx();
        float yawRate = msg.yawRate();
        if (verbose) {
          std::cout << "Got EQUILIBRIOCEPTION vx=" << vx << " and yawRate=" 
            << yawRate << std::endl;
        }
      }
    }};

    auto atFrequency{[&od4, &completeFrameMutex, &localPathSenderId, 
      &verbose]() -> bool
      {
        std::lock_guard<std::mutex> lock(completeFrameMutex);

        float path[] = {1.23f, 2.34f, 3.45f, 4.56f};

        std::string data(reinterpret_cast<char const *>(std::begin(path)),
            reinterpret_cast<char const *>(std::end(path)));

        cluon::data::TimeStamp ts = cluon::time::now();

        opendlv::logic::action::LocalPath localPath;
        localPath.length(2);
        localPath.data(data);
        od4.send(localPath, ts, localPathSenderId);

        if (verbose) {
          cv::Mat img(320, 240, CV_8UC3, cv::Scalar(0,0,0));
          cv::imshow("Visualization", img);
          cv::waitKey(1);
        }

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
