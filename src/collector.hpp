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
#include <mutex>
#include <iterator>
#include "pathplanner.hpp"

#define PI 3.14159265

class Collector{
  public:
    Collector(PathPlanner &planner, std::map<std::string, std::string>);
    ~Collector() = default;

    // CFSD19 modification
    void CollectConesCFSD19();
    void GetCompleteFrameCFSD19();
    void ProcessFrameCFSD19();
    
  public:
    PathPlanner &m_planner;
    std::queue<Cone> m_currentConeFrame;
    
    //Variables for creating cone frames
    std::mutex uncompleteFrameMutex;
    std::map<uint32_t, Cone> uncompleteFrame;
    uint32_t currentUncompleteFrameId;

    std::mutex completeFrameMutex;
    std::map<uint32_t, Cone> completeFrame;
    uint32_t currentCompleteFrameId;
    
    cluon::data::TimeStamp frameStart;
    cluon::data::TimeStamp frameEnd;
    Point2D currentAim;
    bool m_verbose;
    bool m_debug;

    //Function for creating cone frames
    void getObjectFrameStart(cluon::data::Envelope envelope);
    void getObjectFrameEnd(cluon::data::Envelope envelope);
    void getObject(cluon::data::Envelope envelope);
    void getObjectType(cluon::data::Envelope envelope);
    void getObjectPosition(cluon::data::Envelope envelope);
    void getEquilibrioception(cluon::data::Envelope envelope);
    
    //Visualiztion of aimpoints fro debugging
    void getAimpoint(cluon::data::Envelope envelope);
    
};

#endif
