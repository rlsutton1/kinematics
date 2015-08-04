package robotics;

public class Frame
{

	Frame parentFrame;
	Pose pose;
	String name;

	Frame(String name, Frame parentFrame, Pose pose)
	{
		this.name = name;
		this.parentFrame = parentFrame;
		this.pose = pose;
	}

	public Frame()
	{
		// TODO Auto-generated constructor stub
	}

	Point toParentFrame(Point point)
	{

		return null;
		//return pose.transform(this,point);
	}

	Point toChildFrame(Frame childFrame, Point point)
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

	static final Frame worldFrame = new Frame("World Frame",null,null);
	
	public static Frame getWorldFrame()
	{
		return worldFrame;
		
	}
}
