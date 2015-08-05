package robotics;

import static org.junit.Assert.assertTrue;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class MeArmKinematics extends ArmKinematics
{

	protected static final String ARM_CENTER_JOINT = "Arm Center";
	protected static final String ARM_BASE_JOINT = "Arm Base";
	protected static final String TURRET_JOINT = "Turret";

	public MeArmKinematics()
	{
		super(Frame.getWorldFrame(), new Pose(0, 0, 0, 0, 0, 0));

		// next time I'll use the standard as at chapter 7.2.1 in robotics book.
		//
		// the joint at the end of a link is aligned such that the axis of
		// rotation of the joint is around the z-axis of the link and the x-axis
		// is parelle to the link

		add(new Link("Base to Servo", 0, 0, 20, 0, 0, 0));
		add(new Joint(TURRET_JOINT, Axis.YAW, null, 0.0));
		add(new Link("Turret to arm Base", 0, 20, 20, 0, 0, 0));
		add(new Joint(ARM_BASE_JOINT, Axis.PITCH, null, 0.0));
		add(new Link("Arm segment 1", 0, 0, 81, 0, 0, 0));
		add(new Joint(ARM_CENTER_JOINT, Axis.PITCH, null, 0.0));
		add(new Link("Arm segment 2", 0, 0, 81, 0, 0, 0));
		add(new Joint("Wrist", Axis.PITCH, null, 0.0));

		setInvKinematics(getInvKinematics());

	}

	public void checkError(ArmKinematics arm, Pose pose)
	{
		Vector3D endPoint = arm.getEndEffectorPose();

		double xdiff = Math.abs(pose.transform.transform.getX()
				- endPoint.getX());
		double ydiff = Math.abs(pose.transform.transform.getY()
				- endPoint.getY());
		double zdiff = Math.abs(pose.transform.transform.getZ()
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

				arm.getJoint(TURRET_JOINT).setJointAngle(turretAngle);
				Vector3D armBase = arm.getPoint(ARM_BASE_JOINT);

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
				arm.getJoint(ARM_BASE_JOINT).setJointAngle(baseAngle);
				arm.getJoint(ARM_CENTER_JOINT).setJointAngle(midArmAngle);

			}

		};
	}

}
