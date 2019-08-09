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
      (0 == commandlineArguments.count("freq")) ||
      (0 == commandlineArguments.count("speed-min")) ||
      (0 == commandlineArguments.count("speed-max")) ||
      (0 == commandlineArguments.count("steer-max")))  {
    std::cerr << argv[0] << " creates a path based on objec data input." 
      << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> "
      << "--freq=<Frequency> [--verbose]" << std::endl;
  } else {
    bool const verbose{commandlineArguments.count("verbose") != 0};

    float const carHalfWidth{500.0f};

    float const maxAimPointAzimuthAngleSpeed{500.0f};
    float const maxAimPointZenithAngleSpeed{500.0f};

    int32_t const freq{std::stoi(commandlineArguments["freq"])};
    float const dt{1.0f / freq};
    
    float const speedMin{std::stof(commandlineArguments["speed-min"])};
    float const speedMax{std::stof(commandlineArguments["speed-max"])};
    float const steerMax{std::stof(commandlineArguments["steer-max"])};

    cluon::OD4Session od4{
      static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    std::mutex uncompleteFrameMutex;
    std::map<uint32_t, Object> uncompleteFrame;
    cluon::data::TimeStamp frameTime;
    uint32_t currentUncompleteFrameId;
    
    std::mutex completeFrameMutex;
    std::vector<Object> completeFrame;

    std::mutex modeMutex;
    uint32_t mode{0};  // 0: auto, 1: blue, 2: yellow, 3: parking

    std::pair<float, float> aimPoint{0.0f, 0.0f};
    bool isParked{false};
    
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

        /*
        std::sort(completeFrame.begin(), completeFrame.end(),
            [](Object const &a, Object const &b) -> bool
            {
              return a.zenithAngle > b.zenithAngle;
            });
            */

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
    
    auto onSwitchStateRequest{[&modeMutex, &mode, &verbose](
        cluon::data::Envelope &&envelope)
    {
      if (envelope.senderStamp() == 2901) {
        opendlv::proxy::SwitchStateRequest msg = 
          cluon::extractMessage<opendlv::proxy::SwitchStateRequest>(
              std::move(envelope));

        std::lock_guard<std::mutex> lock(modeMutex);
        if (mode != static_cast<uint32_t>(msg.state())) {
          mode = msg.state();
          if (verbose) {
            std::cout << "Behaviour mode changed to " << mode << std::endl;
          }
        }
      }
    }};

    auto atFrequency{[&od4, &completeFrameMutex, &completeFrame, &modeMutex,
      &mode, &aimPoint, &isParked, &dt, &carHalfWidth,
      &maxAimPointAzimuthAngleSpeed, &maxAimPointZenithAngleSpeed, &speedMin, 
      &speedMax, &steerMax, &verbose]() 
      -> bool
      {
        {

          auto hasObjectInside{[](std::pair<float, float> p0, 
              std::pair<float, float> p1, std::pair<float, float> p2, 
              std::vector<Object> const &objects) 
            -> bool
            {
              float const p0x{p0.first};
              float const p0y{p0.second};
              float const p1x{p1.first};
              float const p1y{p1.second};
              float const p2x{p2.first};
              float const p2y{p2.second};

              float const areaTimesTwo{-p1y * p2x + p0y * (-p1x + p2x) 
                  + p0x * (p1y - p2y) + p1x * p2y};

              for (auto &obj : objects) {
                float const px{obj.azimuthAngle};
                float const py{obj.zenithAngle};

                float const s{(p0y * p2x - p0x * p2y + (p2y - p0y) * px 
                  + (p0x - p2x) * py) / areaTimesTwo};
                float const t{(p0x * p1y - p0y * p1x + (p0y - p1y) * px 
                  + (p1x - p0x) * py) / areaTimesTwo};

                if (s > 0.0f && t > 0.0f && (1.0f - s - t) > 0.0f) {
                  return true;
                }
              }
              return false;
            }};

          auto findAimForward{[&hasObjectInside](std::pair<float, float> p0, 
              std::pair<float, float> p1, std::vector<Object> const &objects) 
            -> std::pair<bool, std::pair<float, float>>
            {
              float const angleStep{30.0};
              float const horizonZenithAngle{360.0f};
              float const nearAngleLimit{100.0f};

              bool foundAim{false};
              std::pair<float, float> bestValidAim{0.0f, 0.0f};

              for (float j = horizonZenithAngle; j > nearAngleLimit; 
                  j -= angleStep) {
                for (float i = p1.first; i < p0.first; i += angleStep) {
                  if (foundAim && std::abs(bestValidAim.first) < std::abs(i)) {
                    continue;
                  }
                  
                  std::pair<float, float> p2{i, j};

                  if (!hasObjectInside(p0, p1, p2, objects)) {
                    foundAim = true;
                    bestValidAim = p2;
                  }
                }
                if (foundAim) {
                  break;
                }
              }
              return std::pair<bool, std::pair<float, float>>(
                  foundAim, bestValidAim);
            }};


          auto findAimLeft{[&hasObjectInside](std::pair<float, float> p0, 
              std::pair<float, float> p1, std::vector<Object> const &objects) 
            -> std::pair<bool, std::pair<float, float>>
            {
              bool foundAim{false};
              std::pair<float, float> bestValidAim{0.0f, 0.0f};

              for (auto &obj : objects) {
                if (obj.type != 1 || obj.azimuthAngle < 0.0f) {
                  continue;
                }
                if (bestValidAim.second > obj.zenithAngle) {
                  continue;
                }

                std::pair<float, float> p2{obj.azimuthAngle - 1.5f * obj.w,
                  obj.zenithAngle + obj.h};

                if (!hasObjectInside(p0, p1, p2, objects)) {
                  foundAim = true;
                  bestValidAim = p2;
                }
              }
              return std::pair<bool, std::pair<float, float>>(
                  foundAim, bestValidAim);
            }};
          
          auto findAimRight{[&hasObjectInside](std::pair<float, float> p0, 
              std::pair<float, float> p1, std::vector<Object> const &objects) 
            -> std::pair<bool, std::pair<float, float>>
            {
              bool foundAim{false};
              std::pair<float, float> bestValidAim{0.0f, 0.0f};

              for (auto &obj : objects) {
                if (obj.type != 0 || obj.azimuthAngle > 0.0f) {
                  continue;
                }
                if (bestValidAim.second > obj.zenithAngle) {
                  continue;
                }

                std::pair<float, float> p2{obj.azimuthAngle + 1.5f * obj.w,
                  obj.zenithAngle + obj.h};

                if (!hasObjectInside(p0, p1, p2, objects)) {
                  foundAim = true;
                  bestValidAim = p2;
                }
              }
              return std::pair<bool, std::pair<float, float>>(
                  foundAim, bestValidAim);
            }};


          std::lock_guard<std::mutex> lock1(completeFrameMutex);
          std::lock_guard<std::mutex> lock2(modeMutex);

          std::pair<float, float> const p0{carHalfWidth, 0.0f};
          std::pair<float, float> const p1{-carHalfWidth, 0.0f};

          uint32_t subMode{0};
          bool foundAim{true};
          std::pair<float, float> aim;
          switch (mode) {
            case 0:
            case 3:
              {
                auto res = findAimForward(p0, p1, completeFrame);
                if (res.first) {
                  aim = res.second;
                  if (aim.first < -carHalfWidth / 2.0f) {
                    res = findAimLeft(p0, p1, completeFrame);
                    if (res.first) {
                      subMode = 1;
                      aim = res.second;
                    }
                  } else if (aim.first > carHalfWidth / 2.0f) {
                    res = findAimRight(p0, p1, completeFrame);
                    if (res.first) {
                      subMode = 2;
                      aim = res.second;
                    }
                  }
                } else {
                  foundAim = false;
                }
                break;
              }
            case 1:
              {
                auto res = findAimLeft(p0, p1, completeFrame);
                if (res.first) {
                  aim = res.second;
                } else {
                  res = findAimRight(p0, p1, completeFrame);
                  if (res.first) {
                    aim = res.second;
                  } else {
                    foundAim = false;
                  }
                }
                break;
              }
            case 2:
              {
                auto res = findAimRight(p0, p1, completeFrame);
                if (res.first) {
                  aim = res.second;
                } else {
                  res = findAimLeft(p0, p1, completeFrame);
                  if (res.first) {
                    aim = res.second;
                  } else {
                    foundAim = false;
                  }
                }
                break;
              }
            default:
              break;
          } 
          

          if (!isParked && mode == 3) {
            float closeZenithAngle{150.0f};
            bool foundOrangeToLeft{false};
            bool foundOrangeToRight{false};
              
            for (auto &obj : completeFrame) {
              if ((obj.type == 2 || obj.type == 3) 
                  && obj.zenithAngle < closeZenithAngle) {
                if (obj.azimuthAngle > 0.0f) {
                  foundOrangeToLeft = true;
                  if (verbose) {
                    std::cout << "[Parking]: Found cone to the left." << std::endl;
                  }
                } else {
                  foundOrangeToRight = true;
                  if (verbose) {
                    std::cout << "[Parking]: Found cone to the right." << std::endl;
                  }
                }
              }
            }
            isParked = (foundOrangeToLeft && foundOrangeToRight);
          }

          float aimPointAzimuthAngleSpeed{(aim.first - aimPoint.first) / dt};
          if (aimPointAzimuthAngleSpeed < -maxAimPointAzimuthAngleSpeed) {
            aimPointAzimuthAngleSpeed = -maxAimPointAzimuthAngleSpeed;
          }
          if (aimPointAzimuthAngleSpeed > maxAimPointAzimuthAngleSpeed) {
            aimPointAzimuthAngleSpeed = maxAimPointAzimuthAngleSpeed;
          }
          aimPoint.first = aimPoint.first + aimPointAzimuthAngleSpeed * dt;

          float aimPointZenithAngleSpeed{(aim.second - aimPoint.second) / dt};
          if (aimPointZenithAngleSpeed < -maxAimPointZenithAngleSpeed) {
            aimPointZenithAngleSpeed = -maxAimPointZenithAngleSpeed;
          }
          if (aimPointZenithAngleSpeed > maxAimPointZenithAngleSpeed) {
            aimPointZenithAngleSpeed = maxAimPointZenithAngleSpeed;
          }
          aimPoint.second = aimPoint.second + aimPointZenithAngleSpeed * dt;
           

          float groundSteeringAngleDeg{
            aimPoint.first / carHalfWidth * steerMax};

          {
            opendlv::proxy::GroundSteeringRequest groundSteeringRequest;
            groundSteeringRequest.groundSteering(groundSteeringAngleDeg);
            od4.send(groundSteeringRequest, cluon::time::now(), 2801);
          }
          
          float const horizonZenithAngle{360.0f}; // TODO: make parameter (also used above)
          float groundSpeed{speedMin 
              + aimPoint.second / horizonZenithAngle * (speedMax - speedMin)};
          if (isParked) {
            groundSpeed = -1.0f;
          }
          
          {
            opendlv::proxy::GroundSpeedRequest groundSpeedRequest;
            groundSpeedRequest.groundSpeed(groundSpeed);
            od4.send(groundSpeedRequest, cluon::time::now(), 2201);
          }
          
          if (verbose) {
            int32_t width{1280};
            int32_t height{720};
            float halfWidth{width / 2.0f};
            cv::Mat outImg(height, width, CV_8UC3, cv::Scalar(0,0,0));
            for (auto &obj : completeFrame) {

              int32_t w = static_cast<int32_t>(obj.w);
              int32_t h = static_cast<int32_t>(obj.h);
              int32_t i = static_cast<int32_t>(
                  halfWidth - obj.azimuthAngle - obj.w / 2.0f);
              int32_t j = static_cast<int32_t>(
                  height - obj.zenithAngle - obj.h);


              cv::Scalar color(255, 255, 255);
              switch (obj.type) {
                case 0:
                  color = cv::Scalar(0, 255, 255);
                  break;
                case 1:
                  color = cv::Scalar(255, 0, 0);
                  break;
                case 2:
                  color = cv::Scalar(0, 150, 255);
                  break;
                case 3:
                  color = cv::Scalar(0, 0, 255);
                  break;
                default:
                  break;
              }
              cv::rectangle(outImg, cv::Point(i, j), cv::Point(i + w, j + h), 
                  color);
            }

            if (foundAim) {
              int32_t p0i = static_cast<int32_t>(
                  halfWidth - p0.first);
              int32_t p0j = static_cast<int32_t>(
                  height - p0.second);
              int32_t p1i = static_cast<int32_t>(
                  halfWidth - p1.first);
              int32_t p1j = static_cast<int32_t>(
                  height - p1.second);
              int32_t p2i = static_cast<int32_t>(
                  halfWidth - aim.first);
              int32_t p2j = static_cast<int32_t>(
                  height - aim.second);
              cv::line(outImg, cv::Point(p0i, p0j), cv::Point(p2i, p2j),
                  cv::Scalar(0, 0, 255));
              cv::line(outImg, cv::Point(p1i, p1j), cv::Point(p2i, p2j),
                  cv::Scalar(0, 0, 255));
            }
              

            int32_t api = static_cast<int32_t>(
                halfWidth - aimPoint.first);
            int32_t apj = static_cast<int32_t>(
                height - aimPoint.second);
            cv::ellipse(outImg, cv::Point(api, apj), cv::Size(10, 10), 0.0, 0.0, 
                360.0, cv::Scalar(100, 50, 255));

            std::string modeText = 
              (mode == 0 && subMode == 0) ? "auto (forwards)" 
              : (mode == 0 && subMode == 1) ? "auto (blue)" 
              : (mode == 0 && subMode == 2) ? "auto (yellow)" 
              : (mode == 1) ? "blue"
              : (mode == 2) ? "yellow"
              : (mode == 3 && subMode == 0) ? "parking (forwards)" 
              : (mode == 3 && subMode == 1) ? "parking (blue)" 
              : (mode == 3 && subMode == 2) ? "parking (yellow)"
              : (mode == 3 && subMode == 2) ? "parking (stopped)"
              : "";
            cv::putText(outImg, "Mode: " + modeText, cv::Point(5, 20), 0, 0.5, 
                cv::Scalar(255, 255, 255));
            cv::putText(outImg, "Speed: " + std::to_string(groundSpeed), 
                cv::Point(5, 40), 0, 0.5, cv::Scalar(255, 255, 255));
            cv::putText(outImg, 
                "Steering: " + std::to_string(groundSteeringAngleDeg), 
                cv::Point(5, 60), 0, 0.5, cv::Scalar(255, 255, 255));
            if (isParked) {
              cv::putText(outImg, "Parked!", cv::Point(5, 80), 0, 0.5, cv::Scalar(0, 100, 255));
            }

            cv::imshow("Path planner (local data only)", outImg);
            char key = cv::waitKey(1);
            if (key != -1) {
              if (key == 'a') {
                mode = 0;
              } else if (key == 'b') {
                mode = 1;
              } else if (key == 'y') {
                mode = 2;
              } else if (key == 'p') {
                mode = 3;
              }
            }
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
    od4.dataTrigger(opendlv::proxy::SwitchStateRequest::ID(),
        onSwitchStateRequest);

    od4.timeTrigger(freq, atFrequency);
    
    retCode = 0;
  }
  return retCode;
}
