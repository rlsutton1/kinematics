package robotics;

public class Frame
{

	private Frame parentFrame;
	private Pose pose;
	private String name;
	

	Frame(String name, Frame parentFrame, Pose pose)
	{
		this.name = name;
		this.parentFrame = parentFrame;
		this.pose = pose;
	}

	Point3D toParentFrame(Point3D point)
	{

		return null;
		//return pose.transform(this,point);
	}

	Point3D toChildFrame(Frame childFrame, Point3D point)
	{
		return childFrame.getPose().applyPose(point);
	}

	private Pose getPose()
	{
		return pose;
	}

	public Frame getParentFrame()
	{
		return parentFrame;
	}

	private static final Frame worldFrame = new Frame("World Frame",null,null);
	
	public static Frame getWorldFrame()
	{
		return worldFrame;
		
	}

	public String getName()
	{
		return name;
	}
}
