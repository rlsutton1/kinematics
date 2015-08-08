package robotics;

public enum Axis
{
	// Each Joint is described based on the type of motion it imparts to the link that it controls.
	// We use aviation terminology as it also describes motion in 3D.

	ROLL		// rotation about an axis running from nose to tail of a plane.
	, PITCH  	// nose up or down about an axis running from wing to wing
	, YAW;		// nose left or right about a vertical axis that pierces the aircraft body between the wings

	/**
	 * returns the passed in angle if the Axis matches, otherwise returns 0
	 * 
	 * @param angle
	 * @param pitch
	 * @return
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
