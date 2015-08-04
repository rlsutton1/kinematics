package robotics;

import org.junit.Test;

public class MeArmServoCalculatorTest
{

	@Test
	public void test1()
	{
		MeArmServoCalculator arm = new MeArmServoCalculator();
		
		Pose pose = new Pose(0, 101, 121, 0, 0, 0);
		arm.setPosition(pose);

		
				setServos(arm);

	}
	
	@Test
	public void test()
	{
		MeArmServoCalculator arm = new MeArmServoCalculator();
		Pose pose = new Pose(0, 0, 50, 0, 0, 0);
		arm.setPosition(pose);

		try
		{

			for (int z = 40; z < 150; z++)
			{
				System.out.println("Z " + z);
				pose = new Pose(0, 20, z, 0, 0, 0);
				arm.setPosition(pose);
				setServos(arm);

				Thread.sleep(50);

			}
		} catch (InterruptedException e1)
		{
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
	}

	private void setServos(MeArmServoCalculator arm)
	{
		arm.getTurretPwm();

		arm.getArmBasePwm();

		arm.getArmCenterPwm();
	}

}
