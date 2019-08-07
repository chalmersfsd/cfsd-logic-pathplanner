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

  Object(): objectId(), type(), azimuthAngle(), zenithAngle(), w(), h(), x(), 
    y(), collectedComponents() {}
};

class Cone {
  public:
    Cone(uint32_t, std::pair<cluon::data::TimeStamp, std::pair<float, float>>)
      noexcept;
    ~Cone() noexcept;

    void addPosition(std::pair<cluon::data::TimeStamp, std::pair<float, float>>)
      noexcept;
    void cleanPath(cluon::data::TimeStamp, float) noexcept;
    float getAge(cluon::data::TimeStamp) noexcept;
    std::pair<float, float> getPosition() noexcept;
    std::pair<float, float> getPositionPredicted(cluon::data::TimeStamp) 
      noexcept;
    std::pair<float, float> getVelocity() noexcept;
    uint32_t getType() noexcept;
    void print() noexcept;

  private:
    uint32_t m_type;
    std::vector<std::pair<cluon::data::TimeStamp, std::pair<float, float>>>
      m_motion;
};

inline Cone::Cone(uint32_t type,
    std::pair<cluon::data::TimeStamp, std::pair<float, float>> position)
  noexcept:
  m_type(type),
  m_motion()
{
  m_motion.push_back(position);
}

inline Cone::~Cone() noexcept
{
}
    
inline void Cone::addPosition(
    std::pair<cluon::data::TimeStamp, std::pair<float, float>> position)
      noexcept
{
  m_motion.push_back(position);
}

inline void Cone::cleanPath(cluon::data::TimeStamp now, float duration)
  noexcept
{
  if (m_motion.size() < 2) {
    return;
  }

  std::vector<std::pair<cluon::data::TimeStamp, std::pair<float, float>>>
    validMotion;
  for (size_t i{m_motion.size() - 2}; i >= 0; --i) {
    cluon::data::TimeStamp lastSighting{m_motion[i].first};
    float age{static_cast<float>(cluon::time::toMicroseconds(now)
         - cluon::time::toMicroseconds(lastSighting)) / 1000000.0f};
    if (age < duration) {
      validMotion.insert(validMotion.begin(), m_motion[i]);
    } else {
      break;
    }
  }

  m_motion = validMotion;
}

inline float Cone::getAge(cluon::data::TimeStamp now) noexcept
{
  cluon::data::TimeStamp lastSighting{m_motion.back().first};
  float age{static_cast<float>(cluon::time::toMicroseconds(now)
       - cluon::time::toMicroseconds(lastSighting)) / 1000000.0f};
  return age;
}

inline std::pair<float, float> Cone::getPosition() noexcept
{
  return m_motion.back().second;
}

inline std::pair<float, float> Cone::getPositionPredicted(
    cluon::data::TimeStamp now) noexcept
{
  auto position = getPosition();
  if (m_motion.size() >= 2) {
    cluon::data::TimeStamp lastSighting{m_motion.back().first};
    float dt{static_cast<float>(cluon::time::toMicroseconds(now)
         - cluon::time::toMicroseconds(lastSighting)) / 1000000.0f};
    auto velocity = getVelocity();
    
    std::pair<float, float> prediction{
      position.first + velocity.first * dt,
      position.second + velocity.second * dt
    };
    return prediction;
  }
  return position;
}

inline std::pair<float, float> Cone::getVelocity() noexcept
{
  if (m_motion.size() < 2) {
    return std::pair<float, float>{0.0f, 0.0f};
  }

  cluon::data::TimeStamp t1{m_motion[m_motion.size() - 1].first};
  cluon::data::TimeStamp t0{m_motion[m_motion.size() - 2].first};
  float dt{static_cast<float>(cluon::time::toMicroseconds(t1) 
        - cluon::time::toMicroseconds(t0)) / 1000000.0f};

  float a1{m_motion[m_motion.size() - 1].second.first};
  float a0{m_motion[m_motion.size() - 2].second.first};
  float aDiff{a1 - a0};
  
  float z1{m_motion[m_motion.size() - 1].second.second};
  float z0{m_motion[m_motion.size() - 2].second.second};
  float zDiff{z1 - z0};

  return std::pair<float, float>{aDiff / dt, zDiff / dt};
}

inline uint32_t Cone::getType() noexcept
{
  return m_type;
}

inline void Cone::print() noexcept
{
  std::cout << " type: " << m_type << std::endl;
  std::cout << " motion (" << m_motion.size() << " entries)"
    << std::endl;
  for (auto pos : m_motion) {
    std::cout << cluon::time::toMicroseconds(pos.first) 
      << ": " << pos.second.first << ", " << pos.second.second 
      << std::endl;
  }
}

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

    float const sameConeDistance = 25.0f;


    cluon::OD4Session od4{
      static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    std::mutex uncompleteFrameMutex;
    std::map<uint32_t, Object> uncompleteFrame;
    cluon::data::TimeStamp frameTime;
    uint32_t currentUncompleteFrameId;
    
    std::mutex conesMutex;
    std::vector<Cone> cones;
    cluon::data::TimeStamp conesTime;
    
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
      &currentUncompleteFrameId, &conesMutex, &cones, &conesTime, 
      &sameConeDistance, &verbose](cluon::data::Envelope &&envelope)
    {
      opendlv::logic::perception::ObjectFrameEnd msg = 
        cluon::extractMessage<opendlv::logic::perception::ObjectFrameEnd>(
            std::move(envelope));

      uint32_t objectFrameId = msg.objectFrameId();
      if (objectFrameId == currentUncompleteFrameId) {
        std::lock_guard<std::mutex> lock1(conesMutex);
        std::lock_guard<std::mutex> lock2(uncompleteFrameMutex);
        
        if (verbose) {
          std::cout << "Merging frame (id=" << objectFrameId 
            << ") with cone map, numer of objects=" 
            << uncompleteFrame.size() <<  std::endl;
        }

        conesTime = frameTime;

        std::vector<Cone> conesToAdd;
        for (auto obj : uncompleteFrame) {
          if (obj.second.collectedComponents != 4) {
            std::cout << "Warning: Expected 4 collected components for object "
              << "(id=" << obj.first << ") but found " 
              << obj.second.collectedComponents << " (discarded)." << std::endl;
            continue;
          }

          uint32_t type{obj.second.type};
          float azimuth{obj.second.azimuthAngle};
          float zenith{obj.second.zenithAngle};
              
          std::pair<cluon::data::TimeStamp, std::pair<float, float>> pos{
            frameTime, {azimuth, zenith}};

          bool isInList{false};
          for (auto &cone : cones) {
            if (cone.getType() != type) {
              continue;
            }

            auto conePosition{cone.getPositionPredicted(frameTime)};
            float dAzimuth{azimuth - conePosition.first};
            float dZenith{zenith - conePosition.second};
            double distance{sqrt(dAzimuth * dAzimuth + dZenith * dZenith)};

            if (distance < sameConeDistance) {
              isInList = true;
              cone.addPosition(pos);
              std::cout << "Merge cones since " << distance << " < " 
                << sameConeDistance << std::endl;
              break;
            }
          }
          if (!isInList) {
            Cone cone(type, pos);
            conesToAdd.push_back(cone);
            std::cout << "Add new cone" << std::endl;
          }

        }
        cones.insert(std::end(cones), std::begin(conesToAdd),
            std::end(conesToAdd));
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

    auto onEquilibrioception{[](cluon::data::Envelope &&)
    {
    }};
    
    auto atFrequency{[&od4, &conesMutex, &cones, &conesTime, &verbose]() 
      -> bool
      {
        {
          std::lock_guard<std::mutex> lock(conesMutex);
          
          float const maxAge{1.0f};
          float const maxPathDuration{5.0f};

          // Clean
          std::vector<Cone> validCones;
          for (auto &cone : cones) {
            if (cone.getAge(conesTime) < maxAge) {
              cone.cleanPath(conesTime, maxPathDuration);
              validCones.push_back(cone);
            }
          }
          cones = validCones;






          if (verbose) {

            int32_t width{1280};
            int32_t height{720};
            float halfWidth{width / 2.0f};
            cv::Mat outImg(height, width, CV_8UC3, cv::Scalar(0,0,0));
            std::cout << "--- Cones" << std::endl;
            uint32_t n{0};
            for (auto &cone : cones) {

              std::cout << "cone " << ++n << std::endl;
              cone.print();

              auto position = cone.getPosition();
              int32_t x = static_cast<int32_t>(
                  halfWidth - position.first);
              int32_t y = static_cast<int32_t>(
                  height - position.second);
              int32_t w = 10;
              int32_t h = 10;

              cv::Scalar color(255, 255, 255);
              switch (cone.getType()) {
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
              cv::ellipse(outImg, cv::Point(x, y), cv::Size(w, h), 0.0, 0.0, 
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
    od4.dataTrigger(opendlv::logic::sensation::Equilibrioception::ID(),
        onEquilibrioception);

    od4.timeTrigger(std::stoi(commandlineArguments["freq"]), atFrequency);
    
    retCode = 0;
  }
  return retCode;
}
