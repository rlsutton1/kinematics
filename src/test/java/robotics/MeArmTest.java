package robotics;

import org.junit.Test;

import robotics.meArm.MeArmKinematics;

public class MeArmTest
{

	@Test
	public void testArmExtensionInXYZ()
	{
		// y= forward

		MeArmKinematics arm = new MeArmKinematics();

		// all angles zero
		// Pose pose = new Pose(0, 20, 202, 0, 0, 0);

		for (int y = 40; y < 100; y++)
			for (int x = -80; x < 80; x++)
				for (int z = 40; z < 100; z++)
				{
					Pose pose = new Pose(x, y, z, 0, 0, 0);

					arm.setPosition(pose);
					arm.checkError(arm, pose);
				}

	}

}
