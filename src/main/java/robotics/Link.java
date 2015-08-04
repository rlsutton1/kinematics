package robotics;

public class Link extends Pose
{

	Link(String name, double x, double y, double z, double roll, double pitch, double yaw)
	{
		super(x, y, z, roll, pitch, yaw);
		this.name = name;
	}

}
