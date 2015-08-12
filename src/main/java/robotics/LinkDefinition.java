package robotics;

public class LinkDefinition implements Definition
{
	String name;
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;

	public LinkDefinition(String name, double x, double y, double z, double roll, double pitch, double yaw)
	{
		this.name = name;
		this.x = x;
		this.y = y;
		this.z = z;
		this.roll = roll;
		this.pitch = pitch;
		this.yaw = yaw;
	}

	@Override
	public ComputationalPose createPose()
	{
		return new ComputationalPose(this, x, y, z, roll, pitch, yaw);
	}

	public String getName()
	{
		return name;
	}

	@Override
	public Axis getAxisOfRotation()
	{
		throw new IllegalAccessError("This method is not available for a LinkDefinition");
	}

}
