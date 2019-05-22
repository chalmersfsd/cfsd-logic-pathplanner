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

#ifndef COLLECTOR_HPP
#define COLLECTOR_HPP

#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <Eigen/Core>
#include "opendlv-standard-message-set.hpp"
#include "cluon-complete.hpp"
// CFSD19 modification
#include <algorithm>
#include "cone.hpp"
#include <queue>
#define PI 3.14159265
enum Status { isMakingFrame, isProcessingFrame};
struct Point2D {
    double x;
    double y;
};

class Collector{
  public:
    Collector();
    ~Collector() = default;

    // CFSD19 modification
    void CollectConesCFSD19();
    void GetCompleteFrameCFSD19();
    void ProcessFrameCFSD19();
    std::vector<Cone> SortConesCFSD19(std::vector<Cone>);
    void GuessMissingCones(std::vector<Cone>* blues, std::vector<Cone>* yellows);
    
    std::vector<Point2D> PredictConePositions(std::vector<Cone> observedCone, std::vector<Point2D> *pastCone, Eigen::ArrayXXf displaceVector);
    std::vector<Point2D> MatchConePair(std::vector<Cone> observedCone, std::vector<Point2D> pastCone);
    
    Eigen::ArrayXXf MakeSidePoints(std::vector<Cone> cones, uint32_t numberOfPoints);

    // CFSD19 modification
  public:
    std::vector<Point2D> m_pastBlue;
    std::vector<Point2D> m_pastYellow;
    std::queue<Cone*> m_Cones;
    std::queue<Cone*> m_currentConeFrame;
    uint32_t m_collectorFrameCounter;
    uint32_t m_currentFrameCounter;
    Status m_status;
    bool m_isSkidpad;
};

#endif
