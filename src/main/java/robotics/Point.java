package robotics;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Point
{

	Vector3D point;
	private Frame frame;

	Point(Frame frame, double x, double y, double z)
	{
		this.frame = frame;
		point = new Vector3D(x, y, z);
	}

	@Override 
	public String toString()
	{
		return point.getX()+" "+point.getY()+" "+point.getZ();
	}

	public Frame getFrame()
	{
		return frame;
	}

	public Point(Vector3D point)
	{
		this.point = point;
	}

	public Point subtract(Transform transform)
	{
		return transform.subtract(point);
	}

	public Point rotate(Rotation rotation)
	{
		return new Point(rotation.applyTo(point));
	}

	public Point invRotate(Rotation rotation)
	{
		return new Point(rotation.applyInverseTo(point));
	}

	public Point add(Transform transform)
	{
		return transform.add(point);
	}

	public double getX()
	{
		return point.getX();
	}

	public double getY()
	{
		return point.getY();
	}

	public double getZ()
	{
		return point.getZ();
	}

}
