package robotics;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;

/**
 * Used by Kinematics to make calculations about a definition as a pose is calculated. 
 * 
 * @author bsutton
 *
 */
public class ComputationalPose extends Pose
{
	private double[] angles;

	private Definition definition;

	public ComputationalPose(Definition definition)
	{
		super(definition.getName());
		this.definition = definition;
		this.angles = getRotation().getAngles(RotationOrder.XYZ);
	}
	
	public ComputationalPose(Definition definition, double x, double y, double z, double roll, double pitch, double yaw)
	{
		super(definition.getName(),x,y,z,roll,pitch,yaw);
	}

	public void setAngle(double angle)
	{
		setRotation(new Rotation(RotationOrder.XYZ, angles[0]
				+ definition.getAxisOfRotation().getRotatedAngle(angle, Axis.PITCH), angles[1]
				+ definition.getAxisOfRotation().getRotatedAngle(angle, Axis.ROLL), angles[2]
				+ definition.getAxisOfRotation().getRotatedAngle(angle, Axis.YAW)));
		
	}

}
