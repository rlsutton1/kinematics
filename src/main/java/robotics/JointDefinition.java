package robotics;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;


public class JointDefinition extends Pose
{

	private Axis axisOfRotation;
	private double[] angles;
	
	private String name;
	private iJoint joint;
	private double currentAngle;

	public JointDefinition(String name, Axis axisOfRotation)
	{
		super(name, new Pose(0,0,0,0,0,0));
		this.name = name;
		this.axisOfRotation = axisOfRotation;
		this.angles = getRotation().getAngles(RotationOrder.XYZ);
		this.joint = null;
		this.currentAngle = 0;
	}
	
	public void registerJoint(iJoint joint)
	{
		this.joint = joint;
	}

	public String getName()
	{
		return name;
	}


	public void setJointAngle(double angle)
	{
		this.currentAngle = angle;
		setRotation(new Rotation(RotationOrder.XYZ, angles[0]
				+ axisOfRotation.getRotatedAngle(angle, Axis.PITCH), angles[1]
				+ axisOfRotation.getRotatedAngle(angle, Axis.ROLL), angles[2]
				+ axisOfRotation.getRotatedAngle(angle, Axis.YAW)));
		
		// update the actual joint position.
		if (joint != null)
			joint.setAngle(this.currentAngle);
	}
	
	public double getJointAngle()
	{
		return this.currentAngle;
	}


	public iJoint getJoint()
	{
		return this.joint;
	}
}
