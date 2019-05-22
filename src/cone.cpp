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

#include "cone.hpp"


Cone::Cone(uint32_t objectId, double x, double y, double z, float azimuthAngle, float zenithAngle, float distance, uint8_t color, uint32_t frame, uint32_t conesPerFrame):
  m_objectId(objectId)
, m_x(x)
, m_y(y)
, m_z(z)
, m_azimuthAngle(azimuthAngle)
, m_zenithAngle(zenithAngle)
, m_distance(distance)
, m_color(color)
, m_frame(frame)
, m_conesPerFrame(conesPerFrame)
{
}
