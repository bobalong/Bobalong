/*
	waypoint_mgr.cpp

	Contains a list of all the waypoints the boat will attempt to travel to.

	This code is released under the terms of the LGPLv3 licence.
 */

#include "waypoint_mgr.h"

#define WP_MGR_MAX_WAYPOINTS

//////////////////////////////////////////////////////////////////////////
/// Static Variables
//////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////
GPSPosition WaypointMgr::GetCurrentWaypoint()
{
	return link_list->position;
}

//////////////////////////////////////////////////////////////////////////
bool WaypointMgr::CheckProximity(GPSPosition curr_pos)
{
	// todo
}

//////////////////////////////////////////////////////////////////////////
void WaypointMgr::Advance()
{
	Waypoint* next = link_list->next_waypoint;

	// Clean up the current waypoint
	delete link_list;

	link_list = next;
	waypoint_count--;
}

//////////////////////////////////////////////////////////////////////////
void WaypointMgr::PushWaypoint(GPSPosition position)
{
	if(waypoint_count != WP_MGR_MAX_WAYPOINTS) {
		waypoint_count++;

		// Create the new waypoint
		Waypoint wp = new Waypoint();
		wp->position = position;

		// Navigate to the last waypoint
		Waypoint* next_wp = link_list;
		for(uint i = 0; i < waypoint_count; i++) {
			next_wp = next_wp->next;
		}

		// Store a pointer in the last element of the list
		next_wp->next = wp;
	}
}

//////////////////////////////////////////////////////////////////////////
GPSPosition WaypointMgr::GetWaypoint()
{
	return link_list->position;
}

//////////////////////////////////////////////////////////////////////////
uint WaypointMgr::GetWaypointCount()
{
	return waypoint_count;
}

//////////////////////////////////////////////////////////////////////////
void WaypointMgr::Clear()
{
	uint curr_wp = 0;
	for(uint i = 0; i < waypoint_count; i++) {
		// Start from the first waypoint, advance to the current end and delete it
		Waypoint* next_wp = link_list;
		for(uint j = 0; j < waypoint_count - curr_wp; j++) {
			next_wp = next_wp->next;
		}
		delete next_wp;
		curr_wp++;
	}
}