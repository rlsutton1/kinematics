package robotics.arm;

public class JointRelative extends Joint
{

	private Joint parentJoint;
	private double relativeAngleOffset;
	private double parentAngleWhenSet;
	private double actuatorMinAngle;
	private double actuatorMaxAngle;

	/**
	 * sometimes a joints angle is affected by another joint, such as when a
	 * lever is used and an actuator is mounted at on the same axis as another
	 * joint's.
	 * 
	 * this implementation allows specifying the parentJoint and an angle
	 * offset.
	 * 
	 * @param joint
	 * @param parentJoint
	 * @param relativeAngleOffset
	 */
	public JointRelative(DefineJoint joint, Joint parentJoint,
			double relativeAngleOffset, double actuatorMinAngle,
			double actuatorMaxAngle)
	{
		super(joint);
		this.parentJoint = parentJoint;
		this.relativeAngleOffset = relativeAngleOffset;
		this.actuatorMinAngle = actuatorMinAngle;
		this.actuatorMaxAngle = actuatorMaxAngle;
	}

	@Override
	void setAngle(double angle) throws IllegalJointAngleException
	{

		double absoluteAngle = super.getAngle() + parentJoint.getAngle()
				+ relativeAngleOffset;
		checkJointAngle(absoluteAngle, actuatorMinAngle, actuatorMaxAngle);

		parentAngleWhenSet = parentJoint.getAngle();
		super.setAngle(angle);

	}

	@Override
	public double getActuatorAngle()
	{
		try
		{
			if (parentJoint.getAngle() != parentAngleWhenSet)
			{
				throw new RuntimeException("The parent joint (" + parentJoint
						+ ") angle has changed since this (" + this
						+ ") relative joint angle was set");
			}

			double angle = super.getAngle() + parentJoint.getAngle()
					+ relativeAngleOffset;

			return checkJointAngle(angle, actuatorMinAngle, actuatorMaxAngle);
		} catch (IllegalJointAngleException e)
		{
			// this shouldn't be possible because the joint angles were checked
			// at the time they were set
			e.printStackTrace();
		}
		// it' shouldn't be possible to get here... we can only get here via
		// throwing an IllegalJointAngleException, see the comment in the catch block :)
		return Double.NaN;

	}

}
