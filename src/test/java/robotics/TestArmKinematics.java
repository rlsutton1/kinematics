package robotics;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import robotics.arm.ArmKinematics;
import robotics.arm.IllegalJointAngleException;
import robotics.arm.InvKinematics;
import robotics.arm.Link;

public class TestArmKinematics extends ArmKinematics
{

	public static final int END_EFFECTOR_LENGTH = 10;
	public final Segment BASE_TO_SERVO;
	public final Segment TURRET_JOINT;
	public final Segment TURRET_TO_ARM_BASE;
	public final Segment BASE_JOINT;
	public final Segment ARM_SEGMENT1;
	public final Segment CENTER_JOINT;
	public final Segment ARM_SEGMENT2;
	public final Segment WRIST_JOINT;
	private Segment END_EFFECTOR;

	public TestArmKinematics()
	{
		super(Frame.getWorldFrame(), new Pose(0, 0, 0, 0, 0, 0));

		BASE_TO_SERVO = addLink("Base to Servo", 0, 0, 20, 0, 0, 0);
		TURRET_JOINT = addJoint("", Axis.YAW, 0, 0, 0);
		TURRET_TO_ARM_BASE = addLink("Turret to arm Base", 0, 20, 20, 0, 0, 0);
		BASE_JOINT = addJoint("", Axis.PITCH, 0, 0, 0);
		ARM_SEGMENT1 = addLink("Arm segment 1", 0, 0, 81, 0, 0, 0);
		CENTER_JOINT = addJoint("", Axis.PITCH, 0, 0, 0);
		ARM_SEGMENT2 = addLink("Arm segment 2", 0, 0, 81, 0, 0, 0);
		WRIST_JOINT = addJoint("", Axis.PITCH, 0, 0, 0);
		END_EFFECTOR = addLink("EndEffector", 0, 0, END_EFFECTOR_LENGTH, 0, 0,
				0);

		setInvKinematics(getInvKinematics());
	}

	private InvKinematics getInvKinematics()
	{
		return new InvKinematics()
		{

			public void determine(Pose tipPose)
					throws IllegalJointAngleException
			{

				// determine and set turret angle

				resetJointsToZero();

				double y = tipPose.getY();
				double x = tipPose.getX();

				double turretAngle = Math.atan2(x, y);

				TURRET_JOINT.setJointAngle(turretAngle);
				Vector3D armBase = getSegmentPose(BASE_JOINT).getTransform()
						.getVector();

				Pose endEffectorPose = getEndEffectorPoseFromTipPose(tipPose,
						turretAngle);

				// calculate distance between armBase and wrist
				double extend = Vector3D.distance(armBase, endEffectorPose
						.getTransform().getVector());
				// double extend = new Transform(endPoint,
				// armBase).getDistance();

				// calculate angle of the bend in the arm to give the desired
				// length
				double upperArm = ARM_SEGMENT1.getLinkLength();
				double foreArm = ARM_SEGMENT2.getLinkLength();
				double c = extend;
				double midArmAngle = Math.PI
						- Math.acos((upperArm * upperArm + foreArm * foreArm - c
								* c)
								/ (2 * upperArm * foreArm));

				// midArmAngle -= Math.PI;

				// calculate angle for armBase
				// atan(changeInXY/changeInZ)

				double z = endEffectorPose.getZ() - armBase.getZ();
				x = new Transform(new Point3D(getFrame(),
						endEffectorPose.getX(), endEffectorPose.getY(), 0),
						new Point3D(getFrame(), armBase.getX(), armBase.getY(),
								0)).getDistance();

				double baseAngle = Math.atan2(x, z) - (midArmAngle / 2.0);

				BASE_JOINT.setJointAngle(baseAngle);
				CENTER_JOINT.setJointAngle(midArmAngle);

				double wristAngle = tipPose.getXAngle()
						- getSegmentPose(WRIST_JOINT).getXAngle();
				WRIST_JOINT.setJointAngle(wristAngle);

			}

			private Pose getEndEffectorPoseFromTipPose(Pose tipPose,
					double turretAngle)
			{
				Vector3D tipVector = getLink(END_EFFECTOR).getTransform()
						.getVector();
				Rotation turretCorrectedRotation = new Rotation(
						RotationOrder.XYZ, tipPose.getXAngle(), 0, turretAngle);
				double[] angles = turretCorrectedRotation
						.getAngles(RotationOrder.XYZ);

				Vector3D turretCorrectedTipVector = turretCorrectedRotation
						.applyInverseTo(tipVector);

				Point3D rootOfEndEffector = tipPose.getTransform().subtract(
						turretCorrectedTipVector);

				Pose tipPose1 = new Pose(rootOfEndEffector.getPoint(),
						turretCorrectedRotation);

				return tipPose1;
			}

		};

	}

}
