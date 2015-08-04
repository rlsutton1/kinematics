package robotics;

import static org.junit.Assert.*;

import org.apache.commons.math3.geometry.VectorFormat;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3DFormat;
import org.junit.Test;

public class ArmTest
{
	// @Test
	// public void testTurretAngle()
	// {
	// // y= forward
	//
	// Arm arm = defineArm();
	//
	// // all angles zero
	// // Pose pose = new Pose(0, 20, 202, 0, 0, 0);
	//
	// int x = 0;
	// int z = 40;
	// for (int y = 40; y < 150; y++)
	// {
	// Pose pose = new Pose(x, y, z, 0, 0, 0);
	//
	// arm.setPosition(pose);
	// Pose endPoint = arm.getEndEffectorPose();
	//
	// double xdiff = Math.abs(pose.transform.transform.getX()
	// - endPoint.transform.transform.getX());
	// double ydiff = Math.abs(pose.transform.transform.getY()
	// - endPoint.transform.transform.getY());
	// double zdiff = Math.abs(pose.transform.transform.getZ()
	// - endPoint.transform.transform.getZ());
	//
	// System.out.println(arm.getJoint("Turret").getJointAngle());
	// System.out.println(arm.getJoint("Arm Base").getJointAngle());
	// System.out.println(arm.getJoint("Arm Center").getJointAngle());
	// System.out.println(x + " " + y + " " + z + " got " + endPoint);
	// System.out.println(xdiff + " " + ydiff + " " + zdiff);
	// assertTrue(xdiff < 2 && ydiff < 2 && zdiff < 2);
	// }
	//
	// }

	@Test
	public void test1()
	{
		Vector3D v1 = new Vector3D(1, 0, 0);
		// rotate around Y axis, X becomes Z
		Rotation r1 = new Rotation(RotationOrder.XYZ, 0, Math.PI / 2, 0);
		Vector3D v2 = new Vector3D(0, 1, 0);

		Vector3D result = r1.applyInverseTo(v1).add(v2);

		Vector3DFormat format = new Vector3DFormat();

		System.out.println(format.format(result));
	}

	@Test
	public void test2()
	{
		Vector3D v1 = new Vector3D(1, 0, 0);
		// rotate around Y axis, X becomes Z
		Rotation r1 = new Rotation(RotationOrder.XYZ, 0, Math.PI / 2, 0);
		Vector3D v2 = new Vector3D(0, 1, 0);

		Vector3D result = r1.applyInverseTo(v1).add(v2);

		Vector3DFormat format = new Vector3DFormat();

		System.out.println(format.format(result));
	}

	@Test
	public void testJoint1()
	{
		ArmKinematics arm = defineArm();
		arm.getJoint("Turret").setJointAngle(0);
		Pose pose = new Pose(0, 20, 202, 0, 0, 0);
		checkError(arm, pose);

		arm.getJoint("Turret").setJointAngle(Math.PI / -2);
		pose = new Pose(-20, 0, 202, 0, 0, 0);
		checkError(arm, pose);

		arm.getJoint("Turret").setJointAngle(Math.PI / 2);
		pose = new Pose(20, 0, 202, 0, 0, 0);
		checkError(arm, pose);

	}

	@Test
	public void testJoint2()
	{
		ArmKinematics arm = defineArm();
		arm.getJoint("Arm Base").setJointAngle(0);
		Pose pose = new Pose(0, 20, 202, 0, 0, 0);
		checkError(arm, pose);

		arm.getJoint("Arm Base").setJointAngle(Math.PI / 2);
		pose = new Pose(0, 182, 40, 0, 0, 0);
		checkError(arm, pose);

		arm.getJoint("Arm Base").setJointAngle(Math.PI / -2);
		pose = new Pose(0, -142, 40, 0, 0, 0);
		checkError(arm, pose);

	}

	@Test
	public void testTurretAngleDoesntAffectZ()
	{
		ArmKinematics arm = defineArm();
		Double z = arm.getEndEffectorPose().getZ();
		for (double a = -Math.PI; a < Math.PI; a += Math.PI / 10.0)
		{
			arm.getJoint("Turret").setJointAngle(a);
			System.out.println(z + " " + arm.getEndEffectorPose().getZ());
			assertTrue(Math.abs(arm.getEndEffectorPose().getZ() - z) < .1);

		}
	}

	@Test
	public void testArmBaseAngleDoesntAffectX()
	{
		ArmKinematics arm = defineArm();
		Double x = arm.getEndEffectorPose().getX();
		for (double a = -Math.PI; a < Math.PI; a += Math.PI / 10.0)
		{
			arm.getJoint("Arm Base").setJointAngle(a);
			System.out.println(x + " " + arm.getEndEffectorPose().getX());
			assertTrue(Math.abs(arm.getEndEffectorPose().getX() - x) < .1);

		}
	}

	@Test
	public void testArmMidAngleDoesntAffectX()
	{
		ArmKinematics arm = defineArm();
		Double x = arm.getEndEffectorPose().getX();
		for (double a = -Math.PI; a < Math.PI; a += Math.PI / 10.0)
		{
			arm.getJoint("Arm Center").setJointAngle(a);
			System.out.println(x + " " + arm.getEndEffectorPose().getX());
			assertTrue(Math.abs(arm.getEndEffectorPose().getX() - x) < .1);

		}
	}

	@Test
	public void testArmExtensionInZ()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// all angles zero
		// Pose pose = new Pose(0, 20, 202, 0, 0, 0);

		int x = 0;
		int y = 40;
		for (int z = 40; z < 150; z++)
		{
			Pose pose = new Pose(x, y, z, 0, 0, 0);

			arm.setPosition(pose);
			checkError(arm, pose);
		}

	}

	@Test
	public void testArmExtensionInXYZ()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// all angles zero
		// Pose pose = new Pose(0, 20, 202, 0, 0, 0);

		for (int y = 40; y < 100; y++)
			for (int x = -80; x < 80; x++)
				for (int z = 40; z < 100; z++)
				{
					Pose pose = new Pose(x, y, z, 0, 0, 0);

					arm.setPosition(pose);
					checkError(arm, pose);
				}

	}

	@Test
	public void testArmExtensionInY()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// all angles zero
		// Pose pose = new Pose(0, 20, 202, 0, 0, 0);

		int x = 0;
		int z = 40;
		for (int y = 40; y < 150; y++)
		{
			Pose pose = new Pose(x, y, z, 0, 0, 0);

			arm.setPosition(pose);
			checkError(arm, pose);
		}
	}

	@Test
	public void testArmExtensionInX()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// all angles zero
		// Pose pose = new Pose(0, 20, 202, 0, 0, 0);

		int y = 40;
		int z = 40;
		for (int x = -80; x < 80; x++)
		{
			Pose pose = new Pose(x, y, z, 0, 0, 0);

			arm.setPosition(pose);
			checkError(arm, pose);
		}

	}

	private ArmKinematics defineArm()
	{
		// next time I'll use the standard as at chapter 7.2.1 in robotics book.
		//
		// the joint at the end of a link is aligned such that the axis of
		// rotation of the joint is around the z-axis of the link and the x-axis
		// is parelle to the link
		ArmKinematics arm = new ArmKinematics(Frame.getWorldFrame(), new Pose(0, 0, 0, 0, 0, 0));

		arm.add(new Link("Base to Servo", 0, 0, 20, 0, 0, 0));
		arm.add(new Joint("Turret", Axis.YAW, null, 0.0));
		arm.add(new Link("Turret to arm Base", 0, 20, 20, 0, 0, 0));
		arm.add(new Joint("Arm Base", Axis.PITCH, null, 0.0));
		arm.add(new Link("Arm segment 1", 0, 0, 81, 0, 0, 0));
		arm.add(new Joint("Arm Center", Axis.PITCH, null, 0.0));
		arm.add(new Link("Arm segment 2", 0, 0, 81, 0, 0, 0));
		arm.add(new Joint("Wrist", Axis.PITCH, null, 0.0));

		arm.setInvKinematics(getInvKinematics());
		return arm;
	}

	@Test
	public void testArmBase90Degrees()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// arm base 90 degrees
		Pose pose = new Pose(0, 181.95, 40, 0, 0, 0);

		arm.setPosition(pose);

		checkError(arm, pose);

	}

	@Test
	public void testAllAnglesZero()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// all angles zero
		arm.getJoint("Turret").setJointAngle(0);
		arm.getJoint("Arm Base").setJointAngle(0);
		arm.getJoint("Arm Center").setJointAngle(0);

		Pose pose = new Pose(0, 20, 202, 0, 0, 0);

		arm.setPosition(pose);

		checkError(arm, pose);

	}

	@Test
	public void testTurret45Degrees()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// turret 90 degrees
		Pose pose = new Pose(14, 14, 101.00, 0, 0, 0);

		arm.setPosition(pose);

		checkError(arm, pose);

	}

	@Test
	public void testTurret22Degrees()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// turret 90 degrees
		Pose pose = new Pose(14, 34, 101.00, 0, 0, 0);

		arm.setPosition(pose);

		checkError(arm, pose);

	}

	@Test
	public void testTurret90Degrees()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// turret 90 degrees
		Pose pose = new Pose(19, 0, 150, 0, 0, 0);

		arm.setPosition(pose);

		checkError(arm, pose);

	}

	private void checkError(ArmKinematics arm, Pose pose)
	{
		Vector3D endPoint = arm.getEndEffectorPose();

		double xdiff = Math.abs(pose.transform.transform.getX()
				- endPoint.getX());
		double ydiff = Math.abs(pose.transform.transform.getY()
				- endPoint.getY());
		double zdiff = Math.abs(pose.transform.transform.getZ()
				- endPoint.getZ());
//		System.out.println(pose.transform + " " + endPoint);
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

				arm.getJoint("Turret").setJointAngle(turretAngle);
				Vector3D armBase = arm.getPoint("Arm Base");


				// calculate distance between armBase and wrist
				double extend = Vector3D.distance(armBase, endPoint.point);
				// double extend = new Transform(endPoint,
				// armBase).getDistance();

				// calculate angle of the bend in the arm to give the desired
				// length
				double midArmAngle = Math.acos(((extend / 2.0) / 81.0)) * 2.0;

				// midArmAngle -= Math.PI;

				// calculate angle for armBase
				// atan(changeInXY/changeInZ)

				double z = endPoint.getZ() - armBase.getZ();
				x = new Transform(new Point(arm.getFrame(), endPoint.getX(),
						endPoint.getY(), 0), new Point(arm.getFrame(),
						armBase.getX(), armBase.getY(), 0)).getDistance();

				double baseAngle = Math.atan2(x, z) - (midArmAngle / 2.0);

				// set joint angles !!
				arm.getJoint("Arm Base").setJointAngle(baseAngle);
				arm.getJoint("Arm Center").setJointAngle(midArmAngle);


			}

		};
	}

	
}
