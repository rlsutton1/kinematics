package robotics;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class TestArmKinematics extends ArmKinematics
{
	protected static final String ARM_TURRET_JOINT = "Turret";
	protected static final String ARM_BASE_JOINT = "Arm Base";
	protected static final String ARM_CENTER_JOINT = "Arm Center";
	protected static final String ARM_WRITS_JOINT = "Arm Wrist";

	Joint TURRET_JOINT = new Joint();
	Joint BASE_JOINT = new Joint();
	Joint CENTRE_JOINT = new Joint();
	Joint WRIST_JOINT = new Joint();

	protected static final JointDefinition TURRET_JOINT_DEF = new JointDefinition(ARM_TURRET_JOINT, Axis.YAW);
	protected static final JointDefinition BASE_JOINT_DEF = new JointDefinition(ARM_BASE_JOINT, Axis.PITCH);
	protected static final JointDefinition CENTRE_JOINT_DEF = new JointDefinition(ARM_CENTER_JOINT, Axis.PITCH);
	protected static final JointDefinition WRIST_JOINT_DEF = new JointDefinition(ARM_WRITS_JOINT, Axis.PITCH);

	public TestArmKinematics()
	{
		super(Frame.getWorldFrame(), new Pose(0, 0, 0, 0, 0, 0));

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

	private InvKinematics getInvKinematics()
	{
		return new InvKinematics()
		{

			public void determine(ArmKinematics arm, Pose endEffectorPose)
			{

				// determine and set turret angle
				Point endPoint = endEffectorPose.applyPose(new Point(arm.getFrame(), 0, 0, 0));

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
				// length
				double midArmAngle = Math.acos(((extend / 2.0) / 81.0)) * 2.0;

				// midArmAngle -= Math.PI;

				// calculate angle for armBase
				// atan(changeInXY/changeInZ)

				double z = endPoint.getZ() - armBase.getZ();
				x = new Transform(new Point(arm.getFrame(), endPoint.getX(), endPoint.getY(), 0), new Point(
						arm.getFrame(), armBase.getX(), armBase.getY(), 0)).getDistance();

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
