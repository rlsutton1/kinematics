package robotics;



public interface iJoint 
{
	/**
	 * Called to set the joints angle when the joint needs to be mofed.
	 * 
	 * @param angle The Angle in radians to set the Joint angle to. 
	 */
	public void setAngle(double angle);
	
	public double getAngle();

}
