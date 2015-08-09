package robotics.meArm;

import static org.junit.Assert.assertTrue;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import robotics.ArmKinematics;
import robotics.Axis;
import robotics.Frame;
import robotics.InvKinematics;
import robotics.JointDefinition;
import robotics.LinkDefinition;
import robotics.Point;
import robotics.Pose;
import robotics.Transform;
import robotics.iJoint;

public class MeArmKinematics extends ArmKinematics
{

	protected static final String  ARM_TURRET_JOINT = "Turret";
	protected static final String ARM_BASE_JOINT = "Arm Base";
	protected static final String ARM_CENTER_JOINT = "Arm Center";
	protected static final String ARM_WRITS_JOINT = "Arm Wrist";
	
	protected static final JointDefinition TURRET_JOINT_DEF = new JointDefinition(ARM_TURRET_JOINT, Axis.YAW);
	protected static final JointDefinition BASE_JOINT_DEF = new JointDefinition(ARM_BASE_JOINT, Axis.PITCH);
	protected static final JointDefinition CENTRE_JOINT_DEF = new JointDefinition(ARM_CENTER_JOINT, Axis.PITCH);
	protected static final JointDefinition WRIST_JOINT_DEF = new JointDefinition(ARM_WRITS_JOINT, Axis.PITCH);

	Joint TURRET_JOINT= new Joint();
	Joint BASE_JOINT = new Joint();
	Joint CENTRE_JOINT = new Joint();
	Joint WRIST_JOINT = new Joint();

	

	public MeArmKinematics()
	{
		super(Frame.getWorldFrame(), new Pose("World Frame", 0, 0, 0, 0, 0, 0));

		// next time I'll use the standard as at chapter 7.2.1 in robotics book.
		//
		// the joint at the end of a link is aligned such that the axis of
		// rotation of the joint is around the z-axis of the link and the x-axis
		// is parallel to the link


		add(new LinkDefinition("Base to Servo", 0, 0, 20, 0, 0, 0));
		add(TURRET_JOINT_DEF);
		add(new LinkDefinition("Turret to arm Base", 0, 20, 20, 0, 0, 0));
		add(BASE_JOINT_DEF);
		add(new LinkDefinition("Arm segment 1", 0, 0, 81, 0, 0, 0));
		add(CENTRE_JOINT_DEF);
		add(new LinkDefinition("Arm segment 2", 0, 0, 81, 0, 0, 0));
		add(WRIST_JOINT_DEF);

		setInvKinematics(getInvKinematics());
		
		TURRET_JOINT_DEF.registerJoint(TURRET_JOINT);
		BASE_JOINT_DEF.registerJoint(BASE_JOINT);
		CENTRE_JOINT_DEF.registerJoint(CENTRE_JOINT);
		WRIST_JOINT_DEF.registerJoint(WRIST_JOINT);

	}

	public void checkError(ArmKinematics arm, Pose pose)
	{
		Vector3D endPoint = arm.getEndEffectorPose();

		double xdiff = Math.abs(pose.getTransform().getVector().getX()
				- endPoint.getX());
		double ydiff = Math.abs(pose.getTransform().getVector().getY()
				- endPoint.getY());
		double zdiff = Math.abs(pose.getTransform().getVector().getZ()
				- endPoint.getZ());
		// System.out.println(pose.transform + " " + endPoint);
		assertTrue(xdiff < 3 && ydiff < 3 && zdiff < 3);
	}

	private InvKinematics getInvKinematics()
	{
		return new InvKinematics()
		{

			public void determine(ArmKinematics arm, Pose endEffectorPose)
			{

				// determine and set turret angle
				Point endPoint = endEffectorPose.applyPose(new Point(arm
						.getFrame(), 0, 0, 0));

				double y = endPoint.getY();
				double x = endPoint.getX();

				double turretAngle = Math.atan2(x, y);

				TURRET_JOINT_DEF.setJointAngle(turretAngle);
				Vector3D armBase = arm.getPoint(BASE_JOINT_DEF);

				// calculate distance between armBase and wrist
				double extend = Vector3D.distance(armBase, endPoint.getPoint());
				// double extend = new Transform(endPoint,
				// armBase).getDistance();

				// calculate angle of the bend in the arm to give the desired
				// length.... should actually use the cosine rule if the 2 parts
				// of the arm are not the same length
				double midArmAngle = Math.acos(((extend / 2.0) / 81.0)) * 2.0;

				double z = endPoint.getZ() - armBase.getZ();
				x = new Transform(new Point(arm.getFrame(), endPoint.getX(),
						endPoint.getY(), 0), new Point(arm.getFrame(),
						armBase.getX(), armBase.getY(), 0)).getDistance();

				double baseAngle = Math.atan2(x, z) - (midArmAngle / 2.0);

				// set joint angles !!
				BASE_JOINT_DEF.setJointAngle(baseAngle);
				CENTRE_JOINT_DEF.setJointAngle(midArmAngle);
			}

		};
	}

	@Override
	public iJoint getJoint(JointDefinition definition)
	{
		
		return definition.getJoint();
	}

}
