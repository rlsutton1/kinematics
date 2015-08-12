package robotics;

import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class TestArmKinematics extends ArmKinematics
{
	protected static final String ARM_TURRET_JOINT = "Turret";
	protected static final String ARM_BASE_JOINT = "Arm Base";
	protected static final String ARM_CENTER_JOINT = "Arm Center";
	protected static final String ARM_WRITS_JOINT = "Arm Wrist";

	Joint TURRET_JOINT;
	Joint BASE_JOINT;
	Joint CENTRE_JOINT;
	Joint WRIST_JOINT;

	static protected final JointDefinition TURRET_JOINT_DEF = new JointDefinition(ARM_TURRET_JOINT, Axis.YAW)
	{
		@Override
		protected iJoint createJoint()
		{
			return new Joint();
		}
	};
	static protected final JointDefinition BASE_JOINT_DEF = new JointDefinition(ARM_BASE_JOINT, Axis.PITCH)
	{
		@Override
		protected iJoint createJoint()
		{
			return new Joint();
		}
	};
	static protected final JointDefinition CENTRE_JOINT_DEF = new JointDefinition(ARM_CENTER_JOINT, Axis.PITCH)
	{
		@Override
		protected iJoint createJoint()
		{
			return new Joint();
		}
	};
	static protected final JointDefinition WRIST_JOINT_DEF = new JointDefinition(ARM_WRITS_JOINT, Axis.PITCH)
	{
		@Override
		protected iJoint createJoint()
		{
			return new Joint();
		}
	};

	public TestArmKinematics()
	{
		super(Frame.getWorldFrame(), new Pose(0, 0, 0, 0, 0, 0));

		add(new LinkDefinition("Base to Servo", 0, 0, 20, 0, 0, 0));
		TURRET_JOINT = add(TURRET_JOINT_DEF);
		add(new LinkDefinition("Turret to arm Base", 0, 20, 20, 0, 0, 0));
		BASE_JOINT = add(BASE_JOINT_DEF);
		add(new LinkDefinition("Arm segment 1", 0, 0, 81, 0, 0, 0));
		CENTRE_JOINT = add(CENTRE_JOINT_DEF);
		add(new LinkDefinition("Arm segment 2", 0, 0, 81, 0, 0, 0));
		WRIST_JOINT = add(WRIST_JOINT_DEF);

		setInvKinematics(getInvKinematics());
	}

	private InvKinematics getInvKinematics()
	{
		return new InvKinematics()
		{

			public void determine(ArmKinematics arm, Pose endEffectorPose)
			{

				// determine and set turret angle
				Point3D endPoint = endEffectorPose.applyPose(new Point3D(arm.getFrame(), 0, 0, 0));

				double y = endPoint.getY();
				double x = endPoint.getX();

				double turretAngle = Math.atan2(x, y);

				Map<Definition, ComputationalPose> poses = arm.getComputationalPoses(null);
				poses.get(TURRET_JOINT_DEF).setAngle(turretAngle);
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
				x = new Transform(new Point3D(arm.getFrame(), endPoint.getX(), endPoint.getY(), 0), new Point3D(
						arm.getFrame(), armBase.getX(), armBase.getY(), 0)).getDistance();

				double baseAngle = Math.atan2(x, z) - (midArmAngle / 2.0);

				// TODO: is it necessary to update the computational poses at this point as we have finished with them.
				poses.get(BASE_JOINT_DEF).setAngle(baseAngle);
				poses.get(CENTRE_JOINT_DEF).setAngle(midArmAngle);

				// set joint angles !!
				TURRET_JOINT_DEF.getJoint().setAngle(baseAngle);
				BASE_JOINT_DEF.getJoint().setAngle(baseAngle);
				CENTRE_JOINT_DEF.getJoint().setAngle(midArmAngle);

			}

		};
	}

	@Override
	public iJoint getJoint(JointDefinition definition)
	{
		return definition.getJoint();
	}

}
