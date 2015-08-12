package robotics.meArm;

import static org.junit.Assert.assertTrue;

import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import robotics.ArmKinematics;
import robotics.Axis;
import robotics.ComputationalPose;
import robotics.Definition;
import robotics.DuplicateDefinition;
import robotics.Frame;
import robotics.InvKinematics;
import robotics.JointDefinition;
import robotics.LinkDefinition;
import robotics.Point3D;
import robotics.Pose;
import robotics.Transform;
import robotics.iJoint;

public class MeArmKinematics extends ArmKinematics
{

	protected static final String ARM_TURRET_JOINT = "Turret";
	protected static final String ARM_BASE_JOINT = "Arm Base";
	protected static final String ARM_CENTER_JOINT = "Arm Center";
	protected static final String ARM_WRITS_JOINT = "Arm Wrist";

	protected static final JointDefinition TURRET_JOINT_DEF = new JointDefinition(ARM_TURRET_JOINT, Axis.YAW)
	{
		@Override
		public iJoint createJoint()
		{
			return new Joint();
		}
	};
	protected static final JointDefinition BASE_JOINT_DEF = new JointDefinition(ARM_BASE_JOINT, Axis.PITCH)
	{
		@Override
		public iJoint createJoint()
		{
			return new Joint();
		}
	};
	protected static final JointDefinition CENTRE_JOINT_DEF = new JointDefinition(ARM_CENTER_JOINT, Axis.PITCH)
	{
		@Override
		public iJoint createJoint()
		{
			return new Joint();
		}
	};
	protected static final JointDefinition WRIST_JOINT_DEF = new JointDefinition(ARM_WRITS_JOINT, Axis.PITCH)
	{
		@Override
		public iJoint createJoint()
		{
			return new Joint();
		}
	};

	// Joint TURRET_JOINT = new Joint();
	// Joint BASE_JOINT = new Joint();
	// Joint CENTRE_JOINT = new Joint();
	// Joint WRIST_JOINT = new Joint();

	public MeArmKinematics()
	{
		super(Frame.getWorldFrame(), new Pose("World Frame", 0, 0, 0, 0, 0, 0));

		// next time I'll use the standard as at chapter 7.2.1 in robotics book.
		//
		// the joint at the end of a link is aligned such that the axis of
		// rotation of the joint is around the z-axis of the link and the x-axis
		// is parallel to the link

		try
		{
			add(new LinkDefinition("Base to Servo", 0, 0, 20, 0, 0, 0));
			add(TURRET_JOINT_DEF);
			add(new LinkDefinition("Turret to arm Base", 0, 20, 20, 0, 0, 0));
			add(BASE_JOINT_DEF);
			add(new LinkDefinition("Arm segment 1", 0, 0, 81, 0, 0, 0));
			add(CENTRE_JOINT_DEF);
			add(new LinkDefinition("Arm segment 2", 0, 0, 81, 0, 0, 0));
			add(WRIST_JOINT_DEF);
		}
		catch (DuplicateDefinition e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		setInvKinematics(getInvKinematics());
	}

	public void checkError(ArmKinematics arm, Pose pose)
	{
		Vector3D endPoint = arm.getEndEffectorPose().getTransform().getVector();

		double xdiff = Math.abs(pose.getTransform().getVector().getX() - endPoint.getX());
		double ydiff = Math.abs(pose.getTransform().getVector().getY() - endPoint.getY());
		double zdiff = Math.abs(pose.getTransform().getVector().getZ() - endPoint.getZ());
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
				Point3D endPoint = endEffectorPose.applyPose(new Point3D(arm.getFrame(), 0, 0, 0));

				double y = endPoint.getY();
				double x = endPoint.getX();

				double turretAngle = Math.atan2(x, y);

				// Get all of the poses that describe the arm.
				Map<Definition, ComputationalPose> poses = arm.getComputationalPoses(null);

				poses.get(TURRET_JOINT_DEF).setAngle(turretAngle);
				Vector3D armBase = arm.getSegmentPose(BASE_JOINT_DEF).getTransform().getVector();

				// calculate distance between armBase and wrist
				double extend = Vector3D.distance(armBase, endPoint.getPoint());
				// double extend = new Transform(endPoint,
				// armBase).getDistance();

				// calculate angle of the bend in the arm to give the desired
				// length.... should actually use the cosine rule if the 2 parts
				// of the arm are not the same length
				double midArmAngle = Math.acos(((extend / 2.0) / 81.0)) * 2.0;

				double z = endPoint.getZ() - armBase.getZ();
				x = new Transform(new Point3D(arm.getFrame(), endPoint.getX(), endPoint.getY(), 0), new Point3D(
						arm.getFrame(), armBase.getX(), armBase.getY(), 0)).getDistance();

				double baseAngle = Math.atan2(x, z) - (midArmAngle / 2.0);

				// set joint angles !!
				// HMMM: I don't think we need to set the computational angles here as we are done with computations.
				poses.get(BASE_JOINT_DEF).setAngle(baseAngle);
				poses.get(CENTRE_JOINT_DEF).setAngle(midArmAngle);

				// Finally move the arm.
				TURRET_JOINT_DEF.getJoint().setAngle(turretAngle);
				BASE_JOINT_DEF.getJoint().setAngle(baseAngle);
				CENTRE_JOINT_DEF.getJoint().setAngle(midArmAngle);

			}

		};
	}

	@Override
	public iJoint getJoint(JointDefinition definition)
	{

		return definition.getJoint();
	}

}
