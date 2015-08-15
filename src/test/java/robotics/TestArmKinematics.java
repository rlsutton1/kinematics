package robotics;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import robotics.arm.ArmKinematics;
import robotics.arm.Axis;
import robotics.arm.InvKinematics;
import robotics.arm.Joint;
import robotics.arm.Link;

public class TestArmKinematics extends ArmKinematics
{

	public final Segment BASE_TO_SERVO;
	public final Segment TURRET_JOINT;
	public final Segment TURRET_TO_ARM_BASE;
	public final Segment BASE_JOINT;
	public final Segment ARM_SEGMENT1;
	public final Segment CENTER_JOINT;
	public final Segment ARM_SEGMENT2;
	public final Segment WRIST_JOINT;

	public TestArmKinematics()
	{
		super(Frame.getWorldFrame(), new Pose(0, 0, 0, 0, 0, 0));

		BASE_TO_SERVO = add(new Link("Base to Servo", 0, 0, 20, 0, 0, 0));
		TURRET_JOINT = add(new Joint("", Axis.YAW, 0, 0, 0));
		TURRET_TO_ARM_BASE = add(new Link("Turret to arm Base", 0, 20, 20, 0,
				0, 0));
		BASE_JOINT = add(new Joint("", Axis.PITCH, 0, 0, 0));
		ARM_SEGMENT1 = add(new Link("Arm segment 1", 0, 0, 81, 0, 0, 0));
		CENTER_JOINT = add(new Joint("", Axis.PITCH, 0, 0, 0));
		ARM_SEGMENT2 = add(new Link("Arm segment 2", 0, 0, 81, 0, 0, 0));
		WRIST_JOINT = add(new Joint("", Axis.PITCH, 0, 0, 0));

		setInvKinematics(getInvKinematics());
	}

	private InvKinematics getInvKinematics()
	{
		return new InvKinematics()
		{

			public void determine(ArmKinematics arm, Pose endEffectorPose)
			{

				// determine and set turret angle

				double y = endEffectorPose.getY();
				double x = endEffectorPose.getX();

				double turretAngle = Math.atan2(x, y);

				arm.getJoint(TURRET_JOINT).setAngle(turretAngle);
				Vector3D armBase = arm.getSegmentPose(BASE_JOINT)
						.getTransform().getVector();

				// calculate distance between armBase and wrist
				double extend = Vector3D.distance(armBase, endEffectorPose
						.getTransform().getVector());
				// double extend = new Transform(endPoint,
				// armBase).getDistance();

				// calculate angle of the bend in the arm to give the desired
				// length, assuming the two links are of the same length.
				double linkLength = 81.0;
				double midArmAngle = Math.acos(((extend / 2.0) / linkLength)) * 2.0;

				// midArmAngle -= Math.PI;

				// calculate angle for armBase
				// atan(changeInXY/changeInZ)

				double z = endEffectorPose.getZ() - armBase.getZ();
				x = new Transform(new Point3D(arm.getFrame(),
						endEffectorPose.getX(), endEffectorPose.getY(), 0),
						new Point3D(arm.getFrame(), armBase.getX(), armBase
								.getY(), 0)).getDistance();

				double baseAngle = Math.atan2(x, z) - (midArmAngle / 2.0);

				arm.getJoint(BASE_JOINT).setAngle(baseAngle);
				arm.getJoint(CENTER_JOINT).setAngle(midArmAngle);

			}

		};
	}

}
