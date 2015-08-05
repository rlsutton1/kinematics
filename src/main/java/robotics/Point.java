package robotics;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Point
{

	private Vector3D point;
	private Frame frame;

	public Point(Frame frame, double x, double y, double z)
	{
		this.frame = frame;
		this.point = new Vector3D(x, y, z);
	}

	@Override 
	public String toString()
	{
		return getPoint().getX()+" "+getPoint().getY()+" "+getPoint().getZ();
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
		return transform.subtract(getPoint());
	}

	public Point rotate(Rotation rotation)
	{
		return new Point(rotation.applyTo(getPoint()));
	}

	public Point invRotate(Rotation rotation)
	{
		return new Point(rotation.applyInverseTo(getPoint()));
	}

	public Point add(Transform transform)
	{
		return transform.add(getPoint());
	}

	public double getX()
	{
		return getPoint().getX();
	}

	public double getY()
	{
		return getPoint().getY();
	}

	public double getZ()
	{
		return getPoint().getZ();
	}

	public Vector3D getPoint()
	{
		return point;
	}
}
