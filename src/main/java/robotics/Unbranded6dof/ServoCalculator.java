package robotics.Unbranded6dof;

import robotics.Joint;
import robotics.ServoAngleToPwmCalculator;

public class ServoCalculator extends Kinematics
{

	private ServoAngleToPwmCalculator turret;
	private ServoAngleToPwmCalculator shoulder;
	private ServoAngleToPwmCalculator elbow;

	public ServoCalculator()
	{
		turret = new ServoAngleToPwmCalculator(180, 590, -75, 90);

		shoulder = new ServoAngleToPwmCalculator(218, 560, -75, 45);

		elbow = new ServoAngleToPwmCalculator(110, 600, -70, 135);
	}

	public double getTurretPwm()
	{
		return turret.getPwmValue(getJoint(TURRET_JOINT).getJointAngle());
	}

	public double getArmShoulderPwm()
	{
		Joint joint = getJoint(ARM_SHOULDER);
		
		Double jointAngle = joint.getJointAngle();
		double pwmValue = shoulder.getPwmValue(jointAngle);
		System.out.println(joint.getName() +  Math.toDegrees(jointAngle) + " pwm: " + pwmValue);
		return pwmValue;
	}

	public double getArmElbowPwm()
	{
		Joint joint = getJoint(ARM_ELBOW);
		Double jointAngle =joint.getJointAngle();
		double pwmValue = elbow.getPwmValue(jointAngle);
		System.out.println(joint.getName() +  Math.toDegrees(jointAngle) + " pwm: " + pwmValue);
		return pwmValue;
	}

	public void setTurretAngle(double i)
	{
		getJoint(TURRET_JOINT).setJointAngle(i);
	}

	public void setArmShoulderAngle(double radians)
	{
		getJoint(ARM_SHOULDER).setJointAngle(radians);
	}

	public void setArmElbowAngle(double radians)
	{
		getJoint(ARM_ELBOW).setJointAngle(radians);
	}
}
