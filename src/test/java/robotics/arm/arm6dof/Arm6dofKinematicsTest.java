package robotics.arm.arm6dof;

import static org.junit.Assert.*;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.Test;

import robotics.Pose;
import robotics.arm.ArmKinematics;
import robotics.arm.IllegalJointAngleException;

public class Arm6dofKinematicsTest
{

	@Test
	public void testArmExtensionInXYZplusPose()
			throws IllegalJointAngleException
	{
		// y= forward

		ArmKinematics arm = new Arm6dofKinematics();

		// all angles zero
		// Pose pose = new Pose(0, 20, 202, 0, 0, 0);

		for (int y = 130; y < 150; y++)
			for (int x = -80; x < 80; x++)
				for (int z = 100; z < 140; z++)
					for (int p = 90; p < 180; p++)
					{
						 System.out.println(x + " " + y + " " + z + " " + p);
						Pose pose = new Pose(x, y, z, Math.toRadians(p), 0, 0);

						arm.setPosition(pose);
						checkError(arm, pose);
					}

	}

	private void checkError(ArmKinematics arm, Pose pose)
	{
		Vector3D endPoint = arm.getEndEffectorPose().getTransform().getVector();

		double xdiff = Math.abs(pose.getTransform().getVector().getX()
				- endPoint.getX());
		double ydiff = Math.abs(pose.getTransform().getVector().getY()
				- endPoint.getY());
		double zdiff = Math.abs(pose.getTransform().getVector().getZ()
				- endPoint.getZ());
		// System.out.println(pose.transform + " " + endPoint);
		if (!((xdiff < 3 && ydiff < 3 && zdiff < 3)))
		{
			System.out.println(pose.getTransform() + " != " + endPoint);
		}
		assertTrue(xdiff < 4 && ydiff < 4 && zdiff < 4);
	}

}
