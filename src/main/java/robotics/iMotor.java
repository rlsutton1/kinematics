package robotics;

/**
 * Attempt to abstract out the definition of a motor to make it easier to 
 * create specific robot implementations
 * 
 * A motor powers a joint
 */
public interface iMotor
{
	String getName();
	
	double getMaxPwm();

	double getMinPwm();

	double getMinAngle();

	double getMaxAngle();
}
