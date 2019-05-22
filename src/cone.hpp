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

#ifndef CONE_HPP
#define CONE_HPP

#include <cmath>
#include <chrono>
#include "opendlv-standard-message-set.hpp"
class Cone{
  public:
    Cone(uint32_t objectId, double x, double y, double z, float azimuthAngle, float zenithAngle, float distance, uint8_t color, 
  uint32_t frame, uint32_t conesPerFrame);
    ~Cone() = default;
    uint32_t m_objectId;
    double m_x;
    double m_y;
    double m_z;
    float m_azimuthAngle;
    float m_zenithAngle;
    float m_distance;
    uint8_t m_color;
    uint32_t m_frame;
    uint32_t m_conesPerFrame;
};

#endif
