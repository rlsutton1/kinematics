package robotics;

/**
 * Each Joint is described based on the type of motion it imparts to the link that it controls.
 * We use aviation terminology as it also describes motion in 3D.
 * @author bsutton
 *
 */

public enum Axis
{
	/**
	 * rotation about an axis running from nose to tail of a plane.
	 */
	ROLL, 	
	
	/**
	 * nose up or down about an axis running from wing to wing
	 */
	PITCH, 
	
	/**
	 * nose left or right about a vertical axis that pierces the aircraft body between the wings
	 */
	YAW;		

	/**
	 * returns the passed in angle if the Axis matches, otherwise returns 0
	 * 
	 * @param angle The angle in Radians
	 * @param axis The Axis that is being considered. 
	 * @return the passed in angle if the Axis matches, otherwise returns 0
	 */
	public double getRotatedAngle(double angle, Axis axis)
	{
		if (axis == this)
		{
			return angle;
		}
		return 0;
	}
}
