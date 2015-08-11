package robotics;

import java.text.NumberFormat;

import org.apache.commons.math3.geometry.euclidean.threed.CardanEulerSingularityException;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;

public class Pose
{
	// rotation + transform... homtrans?
	
	/**
	 * Optional name of this pose
	 */
	private String name;
	
	/**
	 * The current 3D transformation of those pose from the origin.
	 * i.e. an offset in the x,y,z axis to the location of this pose.
	 */
	private Transform transform;
	
	/**
	 * TODO: how do you describe the rotation?
	 * Is this a zero length vector that points at the pose from the direction
	 * defined by the rotation?
	 */
	private Rotation rotation;

	/**
	 * Creates a new pose based at the origin (0,0,0) with no rotation (i.e. all angles set to zero). 
	 * @param name Name of the new pose
	 */
	public Pose(String name)
	{
		this(name, 0,0,0,0,0,0);
	}
	
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
	 * Creates a new Pose. 
	 * @param name
	 * @param add
	 * @param applyInverseTo
	 */
	public Pose(String name, Point3D add, Rotation applyInverseTo)
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
	public Point3D applyPose(Point3D point)
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

	Point3D revertPose(Point3D point)
	{
		return point.rotate(getRotation()).subtract(getTransform());
	}

	public String getName()
	{
		return name;
	}

	public Point3D getPoint(Frame frame)
	{
		return applyPose(new Point3D(frame, 0, 0, 0));
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
