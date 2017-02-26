/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 Da Hong
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Da Hong <bh.dong@foxmail.com>
*/

#include "sdn-port-position-match.h"

//#define ControlArea std::pair<Vector2D, Vector2D>

namespace ns3{
namespace sdn{

Port_Position_Match::Port_Position_Match ()
{
	m_controlAreaMap.clear();
	ControlAreaList templist;
	ControlArea temp;
	temp.first=Vector2D (0,990),temp.second=Vector2D (1000,1000);
	templist.push_back(temp);
	temp.first=Vector2D (990,990),temp.second=Vector2D (1000,0);
	templist.push_back(temp);
	m_controlAreaMap[65419] = templist;
	templist.clear();

	temp.first=Vector2D (0,990),temp.second=Vector2D (1000,1000);
	templist.push_back(temp);
	temp.first=Vector2D (1000,990),temp.second=Vector2D (2000,1000);
	templist.push_back(temp);
	m_controlAreaMap[65420] = templist;
	templist.clear();

	temp.first=Vector2D (0,990),temp.second=Vector2D (1000,1000);
	templist.push_back(temp);
	temp.first=Vector2D (1000,1000),temp.second=Vector2D (1010,2000);
	templist.push_back(temp);
	m_controlAreaMap[65421] = templist;
	templist.clear();
};

bool
Port_Position_Match::CheckThis (uint16_t port, Vector position)
{
	if(m_controlAreaMap.find(port) != m_controlAreaMap.end())
	{
		ControlAreaList templist;
		templist = m_controlAreaMap[port];
		for(std::list<std::pair<Vector2D, Vector2D>>::iterator it = templist.begin();it != templist.end();++it)
		{
			if(IsInTheArea(position,*it))
			{
				return true;
			}
		}
		return false;
	}
	return true;
}

bool
Port_Position_Match::IsInTheArea(Vector position, ControlArea area)
{
	double lx = (area.first.x < area.second.x) ? area.first.x : area.second.x;
	double rx = (area.first.x < area.second.x) ? area.second.x : area.first.x;
	double ly = (area.first.y < area.second.y) ? area.first.y : area.second.y;
	double ry = (area.first.y < area.second.y) ? area.second.y : area.first.y;

	if(position.x >= lx && position.x <= rx
			&& position.y >= ly && position.y <= ry)
	{
		return true;
	}
	else
		return false;
}

}
}

