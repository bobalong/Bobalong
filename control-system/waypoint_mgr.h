/*
	waypoint_mgr.h 

	Contains a list of all the waypoints the boat will attempt to travel to.

	This code is released under the terms of the LGPLv3 licence.
 */

#pragma once

#include "type_defs.h"

namespace WaypointMgr
{
	struct Waypoint {
		GPSPosition position;
		Waypoint* next;
	};

	static uint waypoint_count;
	static Waypoint* link_list;

	/**********************************************************************************
	 * Returns the current waypoint.
	 *********************************************************************************/
	GPSPosition GetCurrentWaypoint();

	/**********************************************************************************
	 * Determines if the boat is in the proximity of the current waypoint
	 * to be considered in range.
	 * 
	 * @param  curr_pos 			The current position of the boat.
	 *********************************************************************************/
	bool CheckProximity(GPSPosition curr_pos);

	/**********************************************************************************
	 * Advances the current waypoint to the next in line.
	 *********************************************************************************/
	void Advance();

	/**********************************************************************************
	 * Pushes a waypoint to the back of the waypoint list.
	 *********************************************************************************/
	void PushWaypoint(GPSPosition waypoint);

	/**********************************************************************************
	 * Returns a copy of the waypoint at a particular index
	 *********************************************************************************/
	GPSPosition GetWaypoint(uint index);

	/**********************************************************************************
	 * Returns the number of waypoints currently stored in the waypoint manager.
	 *********************************************************************************/
	uint GetWaypointCount();

	/**********************************************************************************
	 * Clears the manager of all waypoints
	 *********************************************************************************/
	void Clear();
}
