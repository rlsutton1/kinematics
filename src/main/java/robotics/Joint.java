package robotics;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;

public class Joint extends Pose
{

	private Axis axisOfRotation;
	private double[] angles;
	private Double currentJointAngle;
	private String name;

	public Joint(String name, Axis axisOfRotation, Pose pose, Double currentJointAngle)
	{
		super(name,pose);
		this.name = name;
		if (pose == null)
		{
			pose = new Pose(0,0,0,0,0,0);
		}
		this.currentJointAngle = currentJointAngle;
		angles = pose.rotation.getAngles(RotationOrder.XYZ);
		this.axisOfRotation = axisOfRotation;
	}

	

	/**
	 * sets the rotation for this joint for the given angle
	 * 
	 * @param angle The Angle in radians to set the Joint angle to. 
	 */
	public void setJointAngle(double angle)
	{
		currentJointAngle = angle;
		rotation = new Rotation(RotationOrder.XYZ, angles[0]
				+ axisOfRotation.getRotatedAngle(angle, Axis.PITCH), angles[1]
				+ axisOfRotation.getRotatedAngle(angle, Axis.ROLL), angles[2]
				+ axisOfRotation.getRotatedAngle(angle, Axis.YAW));

	}

	public String getName()
	{
		return name;
	}



	public Double getJointAngle()
	{
		return currentJointAngle;
	}
}
