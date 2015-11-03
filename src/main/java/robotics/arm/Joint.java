package robotics.arm;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;

import robotics.Axis;

/**
 * Used by Kinematics to make calculations about a definition as a pose is
 * calculated.
 * 
 * @author bsutton
 *
 */
class Joint extends Link
{
	private double[] angles;

	private Axis axis;

	private double currentAngle;

	private double minAngle;

	private double maxAngle;

	/**
	 * a joint is a mutable. and used by ArmKinematics to represent the
	 * mathematical state of the arm
	 * 
	 * @param name
	 * @param axis
	 * @param roll
	 * @param pitch
	 * @param yaw
	 */
	Joint(String name, Axis axis, double roll, double pitch, double yaw)
	{
		super(name, 0, 0, 0, roll, pitch, yaw);
		if (axis == null)
		{
			throw new RuntimeException("Axis can not be null");
		}
		this.axis = axis;
		this.angles = getRotation().getAngles(RotationOrder.XYZ);
		currentAngle = 0;
	}

	public Joint(DefineJoint jointDef)
	{
		super(jointDef.getName(), 0, 0, 0, jointDef.getRoll(), jointDef
				.getPitch(), jointDef.getYaw());
		this.axis = jointDef.getAxis();

		if (axis == null)
		{
			throw new RuntimeException("Axis can not be null");
		}

		this.angles = getRotation().getAngles(RotationOrder.XYZ);
		minAngle = jointDef.getMinAngleRadians();
		maxAngle = jointDef.getMaxAngleRadians();
		currentAngle = 0;
	}

	void setAngle(double angle) throws IllegalJointAngleException
	{
		currentAngle = checkJointAngle(angle, getMinAngle(), getMaxAngle());

		setRotation(new Rotation(RotationOrder.XYZ, angles[0]
				+ axis.getRotatedAngle(currentAngle, Axis.PITCH),

		angles[1] + axis.getRotatedAngle(currentAngle, Axis.ROLL),

		angles[2] + axis.getRotatedAngle(currentAngle, Axis.YAW)));

	}

	double getAngle()
	{
		return currentAngle;
	}

	public double getMaxAngle()
	{
		return maxAngle;
	}

	public double getMinAngle()
	{
		return minAngle;
	}

	public double getActuatorAngle()
	{
		return getAngle();
	}

	protected double checkJointAngle(double absoluteAngle, double min,
			double max) throws IllegalJointAngleException
	{
		if (absoluteAngle > max)
		{
			absoluteAngle -= Math.PI * 2.0;
		}
		if (absoluteAngle < min)
		{
			absoluteAngle += Math.PI * 2.0;
		}
		if (min != max && (min > absoluteAngle || max < absoluteAngle))
		{

			// try moving into correct quadrant before throwing exception

			throw new IllegalJointAngleException(
					"Attempt to set joint angle for " + getName()
							+ " out of bounds " + absoluteAngle + "("+Math.toDegrees(absoluteAngle)+") is not "
							+ min +  "("+Math.toDegrees(min)+") < X < " + max+  "("+Math.toDegrees(max)+")");
		}
		return absoluteAngle;
	}

}
