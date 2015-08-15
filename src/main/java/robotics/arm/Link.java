package robotics.arm;

import robotics.Pose;

public class Link extends Pose
{

	public Link(String name, double x, double y, double z, double roll,
			double pitch, double yaw)
	{
		super(name, x, y, z, roll, pitch, yaw);
	}

}
