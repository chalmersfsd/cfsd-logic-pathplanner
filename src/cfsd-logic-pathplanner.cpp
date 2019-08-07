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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <cstdint>
#include <cmath>
#include <iostream>
#include <memory>
#include <iterator>

struct Object {
  uint32_t objectId;
  uint32_t type;
  float azimuthAngle;
  float zenithAngle;
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

    cluon::OD4Session od4{
      static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    std::mutex uncompleteFrameMutex;
    std::map<uint32_t, Object> uncompleteFrame;
    cluon::data::TimeStamp frameTime;
    uint32_t currentUncompleteFrameId;
    
    std::mutex completeFrameMutex;
    std::vector<Object> completeFrame;
    
    auto onObjectFrameStart{[&uncompleteFrameMutex, &uncompleteFrame,
      &frameTime, &currentUncompleteFrameId](
          cluon::data::Envelope &&envelope)
    {
      opendlv::logic::perception::ObjectFrameStart msg = 
        cluon::extractMessage<opendlv::logic::perception::ObjectFrameStart>(
            std::move(envelope));

      uint32_t objectFrameId = msg.objectFrameId();
      {
        std::lock_guard<std::mutex> lock(uncompleteFrameMutex);
        uncompleteFrame = std::map<uint32_t, Object>();
        currentUncompleteFrameId = objectFrameId;
        frameTime = envelope.sampleTimeStamp();
      }
    }};

    auto onObjectFrameEnd{[&uncompleteFrameMutex, &uncompleteFrame, &frameTime,
      &currentUncompleteFrameId, &completeFrameMutex, &completeFrame, 
      &verbose](cluon::data::Envelope &&envelope)
    {
      opendlv::logic::perception::ObjectFrameEnd msg = 
        cluon::extractMessage<opendlv::logic::perception::ObjectFrameEnd>(
            std::move(envelope));

      uint32_t objectFrameId = msg.objectFrameId();
      if (objectFrameId == currentUncompleteFrameId) {
        std::lock_guard<std::mutex> lock1(completeFrameMutex);
        std::lock_guard<std::mutex> lock2(uncompleteFrameMutex);
      
        completeFrame = std::vector<Object>();
        
        for (auto obj : uncompleteFrame) {
          if (obj.second.collectedComponents != 4) {
            std::cout << "Warning: Expected 4 collected components for object "
              << "(id=" << obj.first << ") but found " 
              << obj.second.collectedComponents << " (discarded)." << std::endl;
          } else {
            completeFrame.push_back(obj.second);
          }
        }

        if (verbose) {
          std::cout << "Frame collected (id=" << objectFrameId 
            << "), numer of objects=" 
            << completeFrame.size() <<  std::endl;
        }
      }
        
      uncompleteFrame = std::map<uint32_t, Object>();
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
          Object object;
          object.objectId = objectId;
          object.type = msg.type();
          object.collectedComponents = 1;
          uncompleteFrame[objectId] = object;
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
          Object object;
          object.objectId = objectId;
          object.x = msg.x();
          object.y = msg.y();
          object.collectedComponents = 1;
          uncompleteFrame[objectId] = object;
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
          uncompleteFrame[objectId].azimuthAngle = msg.azimuthAngle();
          uncompleteFrame[objectId].zenithAngle = msg.zenithAngle();
          uncompleteFrame[objectId].collectedComponents++;
        } else {
          Object object;
          object.objectId = objectId;
          object.azimuthAngle = msg.azimuthAngle();
          object.zenithAngle = msg.zenithAngle();
          object.collectedComponents = 1;
          uncompleteFrame[objectId] = object;
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
          Object object;
          object.objectId = objectId;
          object.w = msg.width();
          object.h = msg.height();
          object.collectedComponents = 1;
          uncompleteFrame[objectId] = object;
        }
      }
    }};

    auto atFrequency{[&od4, &completeFrameMutex, &completeFrame, &verbose]() 
      -> bool
      {
        {
          std::lock_guard<std::mutex> lock(completeFrameMutex);
          
          if (verbose) {
            int32_t width{1280};
            int32_t height{720};
            float halfWidth{width / 2.0f};
            cv::Mat outImg(height, width, CV_8UC3, cv::Scalar(0,0,0));
            for (auto &obj : completeFrame) {

              int32_t i = static_cast<int32_t>(
                  halfWidth - obj.azimuthAngle);
              int32_t j = static_cast<int32_t>(
                  height - obj.zenithAngle);
              int32_t w = 10;
              int32_t h = 10;

              cv::Scalar color(255, 255, 255);
              switch (obj.type) {
                case 0:
                  color = cv::Scalar(0, 255, 255);
                  break;
                case 1:
                  color = cv::Scalar(255, 0, 0);
                  break;
                case 2:
                  color = cv::Scalar(0, 100, 255);
                  break;
                default:
                  break;
              }
              cv::ellipse(outImg, cv::Point(i, j), cv::Size(w, h), 0.0, 0.0, 
                  360.0, color);
            }
            cv::imshow("Path planner (local data only)", outImg);
            cv::waitKey(1);
          }
        }

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

    od4.timeTrigger(std::stoi(commandlineArguments["freq"]), atFrequency);
    
    retCode = 0;
  }
  return retCode;
}
