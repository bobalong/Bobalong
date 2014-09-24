/*
	navigation.h

	Contains functions for returning navigation information.
 */

#pragma once

#include "type_defs.h"

namespace Navigation
{
	/**
	 * Returns the desired heading from one GPSPosition to another GPSPosition.
	 * 
	 * @param  curr_pos    		The current GPS Position of the boat
	 * @param  destination 		The destination of the boat.
	 * @return             		The heading the boat should follow.
	 */
	int GetDesiredHeading(GPSPosition curr_pos, GPSPosition destination);

	/**
	 * Returns the distance between two GPSPositions.
	 */
	double GetDistance(GPSPosition pos_0, GPSPosition pos_1);

	/**
	 * Returns the heading difference between 0-359 degrees.
	 */
	int GetHeadingDiff(int heading0, int heading1);

	/**
	 * Clamps a angle in the 0-359 range.
	 */
	int ClampAngle(int angle);
}