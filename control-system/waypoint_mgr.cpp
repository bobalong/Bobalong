/*
	waypoint_mgr.cpp

	Contains a list of all the waypoints the boat will attempt to travel to.

	This code is released under the terms of the LGPLv3 licence.
 */

#include "waypoint_mgr.h"
#include "math.h"

#define WP_MGR_MAX_WAYPOINTS 		10
#define WP_MGR_PROXIMITY			0.0001

//////////////////////////////////////////////////////////////////////////
/// Static Variables
//////////////////////////////////////////////////////////////////////////

//uint WaypointMgr::waypoint_count;
//Waypoint* Waypoint::link_list;

//////////////////////////////////////////////////////////////////////////
GPSPosition WaypointMgr::GetCurrentWaypoint()
{
	return link_list->position;
}

//////////////////////////////////////////////////////////////////////////
bool WaypointMgr::CheckProximity(GPSPosition curr_pos)
{
	GPSPosition curr_wp = GetCurrentWaypoint();

	if(fabs(curr_pos.latitude - curr_wp.latitude) <= WP_MGR_PROXIMITY && 
		fabs(curr_pos.longitude - curr_wp.longitude) <= WP_MGR_PROXIMITY) {
		return true;
	} else {
		return false;
	}
}

//////////////////////////////////////////////////////////////////////////
void WaypointMgr::Advance()
{
	// Can't advance if we have no more waypoints
	if(waypoint_count > 1) {
		Waypoint* next = link_list->next;
		// Clean up the current waypoint
		delete link_list;

		link_list = next;
	}

	waypoint_count--;
}

//////////////////////////////////////////////////////////////////////////
void WaypointMgr::PushWaypoint(GPSPosition position)
{
	if(waypoint_count != WP_MGR_MAX_WAYPOINTS) {
		waypoint_count++;

		// Create the new waypoint
		Waypoint* wp = new Waypoint();
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
GPSPosition WaypointMgr::GetWaypoint(uint index)
{
    Waypoint* next_wp = link_list;
	for(uint i = 0; i <= index; i++) {
		next_wp = next_wp->next;
	} 
	return next_wp->position;
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
