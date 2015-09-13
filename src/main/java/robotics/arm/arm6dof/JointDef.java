package robotics.arm.arm6dof;

import robotics.Axis;
import robotics.arm.DefineJoint;

class JointDef implements DefineJoint
{

	String name;
	Axis axis;
	double pitch;
	double roll;
	double yaw;
	double minAngle;
	double maxAngle;

	JointDef(String name, Axis axis, double pitch, double roll, double yaw,
			double minAngle, double maxAngle)
	{
		this.name = name;
		this.axis = axis;
		this.pitch = pitch;
		this.roll = roll;
		this.yaw = yaw;
		this.minAngle = minAngle;
		this.maxAngle = maxAngle;
	}

	@Override
	public String getName()
	{
		return name;
	}

	@Override
	public Axis getAxis()
	{
		return axis;
	}

	@Override
	public double getPitch()
	{
		return pitch;
	}

	@Override
	public double getRoll()
	{
		return roll;
	}

	@Override
	public double getYaw()
	{
		return yaw;
	}

	@Override
	public double getMinAngleRadians()
	{
		return minAngle;
	}

	@Override
	public double getMaxAngleRadians()
	{
		return maxAngle;
	}

}
