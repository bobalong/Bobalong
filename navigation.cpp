/*
	navigation.h

	Contains functions for returning navigation information.
 */

#include "navigation.h"
#include "math.h"

#define MATH_DEG_TO_RAD 	3.14 / 180
#define MATH_RAD_TO_DEG		180 / 3.14	

//////////////////////////////////////////////////////////////////////////
int Navigation::GetDesiredHeading(GPSPosition curr_pos, GPSPosition destination)
{
	double lat_rad1 = curr_pos.latitude * MATH_DEG_TO_RAD;
	double lat_rad2 = destination.latitude * MATH_DEG_TO_RAD;

	// Uses the greate circle route
	double lon = (destination.longitude - curr_pos.longitude) * MATH_DEG_TO_RAD;
	double y = sin(lon) * cos(lat_rad2);
	double x = (cos(lat_rad1) * sin(lat_rad2)) - (sin(lat_rad1) * cos(lat_rad2) * cos(lon));
	return atan2(y, x) * MATH_RAD_TO_DEG;
}

//////////////////////////////////////////////////////////////////////////
double Navigation::GetDistance(GPSPosition pos_0, GPSPosition pos_1)
{
	double lat_rad1 = pos_0.latitude * MATH_DEG_TO_RAD;
	double lat_rad2 = pos_1.latitude * MATH_DEG_TO_RAD;
	double lon_rad1 = pos_0.longitude * MATH_DEG_TO_RAD;
	double lon_rad2 = pos_1.longitude * MATH_DEG_TO_RAD;

	double t1 = Math.cos(lat_rad1)*Math.cos(lon_rad1)*Math.cos(lat_rad2)*Math.cos(lon_rad2);
	double t2 = Math.cos(lat_rad1)*Math.sin(lon_rad1)*Math.cos(lat_rad2)*Math.sin(lon_rad2);
	double t3 = Math.sin(lat_rad1) * Math.sin(lat_rad2);
	double tt = Math.acos(t1 + t2 + t3);
return 6366000*tt;
}

//////////////////////////////////////////////////////////////////////////
int Navigation::GetHeadingDiff(int heading0, int heading1)
{
	int heading_diff = heading0 - heading1;

	if(heading_diff < -180) {
		return 360 + heading_diff;
	}
	
	if(heading_diff > 180) {
		return 0 - (360 - heading_diff);
	}

	return heading_diff;
}

//////////////////////////////////////////////////////////////////////////
int Navigation::ClampAngle(int angle)
{
	if(angle > 359) {
		return ClampAngle(angle - 359);
	} else if(angle < 0) {
		return ClampAngle(359 - angle);
	} else {
		return angle;
	}
}
