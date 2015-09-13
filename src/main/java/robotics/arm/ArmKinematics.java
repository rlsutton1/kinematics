package robotics.arm;

import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import robotics.Axis;
import robotics.Frame;
import robotics.Pose;

public abstract class ArmKinematics
{

	private InvKinematics invKinematics;

	/**
	 * Maps the name of a segment Definition to that Definition
	 */
	final private Map<Segment, Link> segments = new LinkedHashMap<>();

	private Frame frame;

	@SuppressWarnings("unused")
	private Pose pose;

	final public static class Segment
	{
		String name;

		// Link link;
		Joint joint;

		public double linkLength;

		public void setJointAngle(double angle)
				throws IllegalJointAngleException
		{
			if (joint == null)
			{
				throw new RuntimeException("This segment " + name
						+ " is not a joint");
			}
			if (Double.isNaN(angle))
			{
				throw new RuntimeException("Angle is NaN");
			}
			joint.setAngle(angle);
		}

		public double getJointAngle()
		{
			if (joint == null)
			{
				throw new RuntimeException("This segment " + name
						+ " is not a joint");
			}
			return joint.getAngle();
		}

		public double getActuatorAngle()
		{
			if (joint == null)
			{
				throw new RuntimeException("This segment " + name
						+ " is not a joint");
			}
			return joint.getActuatorAngle();
		}

		@Override
		public String toString()
		{
			return name;
		}
		
		public double getLinkLength()
		{
			return linkLength;
		}

	}

	public ArmKinematics(Frame frame, Pose pose)
	{
		this.frame = frame;
		this.pose = pose;
	}

	/**
	 * The add(Link) and add (Joint) methods allow you to describe the physical
	 * geometry of your arm. Once the physical geometry of the arm is described
	 * you can use the 'setPosition' method to calculate the required joint
	 * angles for a given position.
	 * 
	 * @param link
	 * @throws DuplicateDefinition
	 */
	public Segment addLink(DefineLink linkDef)
	{
		Segment segment = new Segment();
		segment.name = linkDef.getName();
		segment.linkLength = new Vector3D(0,0,0).distance(new Vector3D(linkDef.getX(), linkDef.getY(),
				linkDef.getZ()));
		segments.put(segment, new Link(linkDef));
		return segment;
	}

	public Segment addLink(String name, double x, double y, double z,
			double roll, double pitch, double yaw)
	{
		Segment segment = new Segment();
		segment.name = name;
		segment.linkLength = new Vector3D(0,0,0).distance(new Vector3D(x, y, z));
		segments.put(segment, new Link(name, x, y, z, roll, pitch, yaw));
		return segment;
	}

	public Segment addJoint(DefineJoint jointDef)
	{
		Segment segment = new Segment();
		segment.name = jointDef.getName();
		segments.put(segment, new Joint(jointDef));
		segment.joint = (Joint) segments.get(segment);
		return segment;
	}

	/**
	 * a joint which the actuators angle is an offset against the angle of
	 * another joint
	 * 
	 * @param jointDef
	 * @param parentJoint
	 * @param relativeAngle
	 * @param minActuatorAngle
	 * @param maxActuatorAnlge
	 * @return
	 */
	public Segment addJointRelative(DefineJoint jointDef, Segment parentJoint,
			double relativeAngle, double minActuatorAngle,
			double maxActuatorAnlge)
	{
		Segment segment = new Segment();
		segment.name = jointDef.getName();
		segments.put(segment,
				new JointRelative(jointDef, (Joint) segments.get(parentJoint),
						relativeAngle, minActuatorAngle, maxActuatorAnlge));
		segment.joint = (Joint) segments.get(segment);
		return segment;
	}

	public Segment addJoint(String name, Axis axis, double roll, double pitch,
			double yaw)
	{
		Segment segment = new Segment();
		segment.name = name;
		segments.put(segment, new Joint(name, axis, roll, pitch, yaw));
		segment.joint = (Joint) segments.get(segment);
		return segment;
	}

	/**
	 * 
	 * @param endEffectorLink
	 * @throws IllegalJointAngleException
	 */
	public void setPosition(Pose endEffectorLink)
			throws IllegalJointAngleException
	{
		invKinematics.determine(endEffectorLink);

	}

	// /**
	// *
	// * @param segment
	// * @return angle of the joint in radians
	// */
	// private double getComputedJointAngle(Segment segment)
	// {
	// return accessJoint(segment).getAngle();
	// }

	/**
	 * Returns a list of Links and joints up to and including the given Segment.
	 * 
	 * If the Segment is null then all Joints and Links are returned.
	 * 
	 * @param definition
	 * @return
	 */
	private List<Pose> getSegmentPoses(Segment segment)
	{
		List<Pose> results = new LinkedList<>();

		for (Entry<Segment, Link> def : segments.entrySet())
		{
			results.add(def.getValue());
			if (def.getKey() == segment)
				break;
		}
		return results;
	}

	/**
	 * set an implementation of the InvKinematics interface that knows how to
	 * compute the inverse kinematics for a particular robot arm
	 * 
	 * @param invKinematics
	 */
	public void setInvKinematics(InvKinematics invKinematics)
	{
		this.invKinematics = invKinematics;

	}

	public void resetJointsToZero() throws IllegalJointAngleException
	{
		for (Link segment : segments.values())
		{
			if (segment instanceof Joint)
			{
				((Joint) segment).setAngle(0.0);
			}
		}
	}

	public Frame getFrame()
	{
		return frame;
	}

	/**
	 * determine the pose of "segment" in the frame of the arm
	 * 
	 * @param segment
	 * @return
	 */
	public Pose getSegmentPose(Segment segment)
	{

		Pose ret = new Pose(Vector3D.ZERO, new Rotation(RotationOrder.XYZ, 0.0,
				0.0, 0.0));
		for (Pose pose : getSegmentPoses(segment))
		{
			ret = ret.compound(pose);
		}
		return ret;
	}

	// /**
	// *
	// * @param segment
	// * @param angleRadians
	// * @throws IllegalJointAngleException
	// */
	// private void setJointAngle(Segment segment, double angleRadians)
	// throws IllegalJointAngleException
	// {
	// Joint joint = accessJoint(segment);
	//
	// joint.setAngle(angleRadians);
	// }

	public Pose getEndEffectorPose()
	{
		return getSegmentPose(null);
	}

	// /**
	// * ensure that we dont try to access a Link as a Joint
	// *
	// * @param segment
	// * @return
	// */
	// private Joint accessJoint(Segment segment)
	// {
	// Link joint = segments.get(segment);
	// if (!(joint instanceof Joint))
	// {
	// throw new RuntimeException(segment + " is not a joint");
	// }
	// return (Joint) joint;
	// }

	public Link getLink(Segment segment)
	{
		Link link = segments.get(segment);
		if (link instanceof Joint)
		{
			throw new RuntimeException(segment + " is not a link");
		}
		return link;
	}

}
