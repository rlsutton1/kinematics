package robotics.arm.arm6dof;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import robotics.Axis;
import robotics.Frame;
import robotics.Point3D;
import robotics.Pose;
import robotics.Transform;
import robotics.arm.ArmKinematics;
import robotics.arm.IllegalJointAngleException;
import robotics.arm.InvKinematics;

public class Arm6dofKinematics extends ArmKinematics
{

	public final Segment BASE_TO_SERVO;
	public final Segment TURRET_JOINT;
	public final Segment TURRET_TO_ARM_BASE;
	public final Segment BASE_JOINT;
	public final Segment ARM_SEGMENT1;
	public final Segment CENTER_JOINT;
	public final Segment ARM_SEGMENT2;
	public final Segment WRIST_JOINT;

	public Arm6dofKinematics()
	{
		super(Frame.getWorldFrame(), new Pose(0, 0, 0, 0, 0, 0));

		// joints.add(new Joint("turret j1", 0, 180, 550));// 170 = -90 , 560 =
		// 80
		// joints.add(new Joint("shoulder j2", 4, 210, 540));// 220 = -70, 560 =
		// 80
		// joints.add(new Joint("elbow j3", 2, 150, 480)); // 490 = -50 ,160 =
		// 100 // note it's opposite to the others
		// joints.add(new Joint("turret j4", 6, 170, 570));
		// joints.add(new Joint("turret j5", 8, 180, 550)); // 600 = -90 , 165 =
		// 90
		// joints.add(new Joint("turret j6", 10, 170, 600));// 550 = 80, 180 =
		// -70

		// define our arms characteristics
		BASE_TO_SERVO = addLink("Base to Servo", 0, 0, 47, 0, 0, 0);

		TURRET_JOINT = addJoint(new JointDef("Turret Joint", Axis.YAW, 0, 0, 0,
				Math.toRadians(-90), Math.toRadians(80)));
		TURRET_TO_ARM_BASE = addLink("Turret to arm Base", 0, 25, 52, 0, 0, 0);

		BASE_JOINT = addJoint(new JointDef("Base Joint", Axis.PITCH, 0, 0, 0,
				Math.toRadians(-70), Math.toRadians(80)));
		ARM_SEGMENT1 = addLink("Arm segment 1", 0, 0, 120, 0, 0, 0);

		CENTER_JOINT = addJointRelative(
				new JointDef("Center Joint", Axis.PITCH, 0, 0, 0,
						Math.toRadians(-160), Math.toRadians(-30)), BASE_JOINT,
				Math.toRadians(90), Math.toRadians(-50), Math.toRadians(100));
		
		
		// 0,-20,123
		ARM_SEGMENT2 = addLink("Arm segment 2", 0, 0, 125, 0, 0, 0);

		WRIST_JOINT = addJoint("", Axis.PITCH, 0, 0, 0);

		setInvKinematics(getInvKinematics());
	}

	private InvKinematics getInvKinematics()
	{
		return new InvKinematics()
		{

			public void determine(Pose endEffectorPose)
					throws IllegalJointAngleException
			{

				// determine and set turret angle

				double y = endEffectorPose.getY();
				double x = endEffectorPose.getX();

				double turretAngle = Math.atan2(x, y);

				TURRET_JOINT.setJointAngle(turretAngle);
				Vector3D armBase = getSegmentPose(BASE_JOINT).getTransform()
						.getVector();

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

			}

		};
	}

}
