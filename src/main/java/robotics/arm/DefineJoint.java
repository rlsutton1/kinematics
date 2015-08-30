package robotics.arm;

import robotics.Axis;

public interface DefineJoint
{

	String getName();

	Axis getAxis();

	double getPitch();

	double getRoll();

	double getYaw();

	double getMinAngleRadians();

	double getMaxAngleRadians();

}
