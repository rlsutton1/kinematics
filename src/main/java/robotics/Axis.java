package robotics;

public enum Axis
{
	ROLL, PITCH, YAW;

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
