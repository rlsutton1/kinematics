package robotics;

public interface InvKinematics
{

	public void determine(ArmKinematics arm, Pose endEffectorPose);
}
