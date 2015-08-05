package robotics;

import java.text.NumberFormat;

import org.apache.commons.math3.geometry.euclidean.threed.CardanEulerSingularityException;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;

public class Pose
{
	// rotation + transform... homtrans?

	Transform transform;
	Rotation rotation;
	String name;

	public Pose(double x, double y, double z, double roll, double pitch, double yaw)
	{
		transform = new Transform(x, y, z);
		rotation = new Rotation(RotationOrder.XYZ, roll, pitch, yaw);
	}

	public Pose(String name, Pose pose)
	{
		if (pose == null)
		{
			pose = new Pose(0, 0, 0, 0, 0, 0);
		}
		transform = pose.transform;
		rotation = pose.rotation;
		this.name = name;
	}

	public Pose(String name, Point add, Rotation applyInverseTo)
	{
		transform = new Transform(add);
		rotation = applyInverseTo;
		this.name = name;
	}

	@Override
	public String toString()
	{
		try
		{
			NumberFormat nf = NumberFormat.getNumberInstance();
			nf.setMaximumFractionDigits(1);
			return transform + " " + nf.format(getAngle(0)) + " "
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
	 * convert the given point into
	 * 
	 * @param point
	 * @return
	 */
	public Point applyPose(Point point)
	{
		return point.add(transform).invRotate(getRotation());

	}

	Rotation getRotation()
	{
		return rotation;
	}

	Point revertPose(Point point)
	{
		return point.rotate(getRotation()).subtract(transform);
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
		return transform.transform.getX();
	}

	public double getY()
	{
		return transform.transform.getY();
	}

	public double getZ()
	{
		return transform.transform.getZ();
	}

}
