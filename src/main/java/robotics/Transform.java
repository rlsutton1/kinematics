package robotics;

import java.text.NumberFormat;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Transform
{

	Vector3D transform;

	Transform(double x, double y, double z)
	{
		transform = new Vector3D(x, y, z);

	}

	public Transform(Point endPoint, Point turret)
	{
		double x = endPoint.getX() - turret.getX();
		double y = endPoint.getY() - turret.getY();
		double z = endPoint.getZ() - turret.getZ();

		transform = new Vector3D(x, y, z);

	}

	public Transform(Point add)
	{
		double x = add.getX();
		double y = add.getY();
		double z = add.getZ();
		transform = new Vector3D(x, y, z);
	}

	@Override
	public String toString()
	{
		NumberFormat format = NumberFormat.getInstance();
		format.setMaximumFractionDigits(1);
		return format.format(transform.getX()) + " " + format.format(transform.getY()) + " "
				+ format.format(transform.getZ());
	}

	public Point add(Vector3D point)
	{
		return new Point(point.add(transform));
	}

	public Point subtract(Vector3D point)
	{
		return new Point(point.subtract(transform));
	}

	public void getRotation()
	{
		throw new RuntimeException("not yet implemented");

	}

	public double getDistance()
	{

		return transform.distance(new Vector3D(0, 0, 0));

	}
}
