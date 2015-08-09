package robotics;

import java.text.NumberFormat;

import org.apache.commons.math3.geometry.euclidean.threed.CardanEulerSingularityException;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;

public class Pose
{
	// rotation + transform... homtrans?

	private Transform transform;
	private Rotation rotation;
	private String name;

	
	public Pose(double x, double y, double z, double roll, double pitch, double yaw)
	{
		this(null, x,y,z,roll,pitch,yaw);
	}

	public Pose(String name, double x, double y, double z, double roll, double pitch, double yaw)
	{
		this.name = name;
		this.transform = new Transform(x, y, z);
		this. rotation = new Rotation(RotationOrder.XYZ, roll, pitch, yaw);
	}

	/**
	 * Copies an existing Pose to new Pose.
	 * 
	 * @param name Name of the new pose
	 * @param pose The pose that we are making a copy of. If the pose is null than the 
	 * 		new pose will be based at the origin with zero angles for each part or the axis (roll, pitch and yaw).
	 */
	public Pose(String name, Pose pose)
	{
		if (pose == null)
		{
			pose = new Pose(name, 0, 0, 0, 0, 0, 0);
		}
		this.transform = pose.getTransform();
		this.rotation = pose.rotation;
		this.name = name;
	}

	/**
	 * Creates a new Pose. 
	 * @param name
	 * @param add
	 * @param applyInverseTo
	 */
	public Pose(String name, Point add, Rotation applyInverseTo)
	{
		this.transform = new Transform(add);
		this.rotation = applyInverseTo;
		this.name = name;
	}

	@Override
	public String toString()
	{
		try
		{
			NumberFormat nf = NumberFormat.getNumberInstance();
			nf.setMaximumFractionDigits(1);
			return getTransform() + " " + nf.format(getAngle(0)) + " "
					+ nf.format(getAngle(1)) + " " + nf.format(getAngle(2));
		} catch (CardanEulerSingularityException e)
		{
			return "Singularity";
		}
	}

	private double getAngle(int pos)
	{
		return rotation.getAngles(RotationOrder.XYZ)[pos]
				* (360 / (2 * Math.PI));
	}

	/**
	 * convert the given point into a Pose
	 * 
	 * @param point The point of where the tip of the robot arm is to be positioned (Posed).
	 * @return The Pose required to position the tip of the robot arm to the given 3D point.
	 */
	public Point applyPose(Point point)
	{
		return point.add(getTransform()).invRotate(getRotation());

	}

	Rotation getRotation()
	{
		return rotation;
	}


	/** TODO: remove this as setters are bad
	 * 
	 * @param rotation
	 */
	public void setRotation(Rotation rotation)
	{
		this.rotation = rotation;
		
	}

	Point revertPose(Point point)
	{
		return point.rotate(getRotation()).subtract(getTransform());
	}

	public String getName()
	{
		return name;
	}

	public Point getPoint(Frame frame)
	{
		return applyPose(new Point(frame, 0, 0, 0));
	}

	public double getX()
	{
		return getTransform().getVector().getX();
	}

	public double getY()
	{
		return getTransform().getVector().getY();
	}

	public double getZ()
	{
		return transform.getVector().getZ();
	}

	public Transform getTransform()
	{
		return transform;
	}


}
