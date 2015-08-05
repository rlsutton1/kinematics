package robotics;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class ArmKinematics
{

	private InvKinematics invKinematics;

	Map<String, Pose> segments = new LinkedHashMap<>();

	private Frame frame;

	@SuppressWarnings("unused")
	private Pose pose;

	public ArmKinematics(Frame frame, Pose pose)
	{
		this.frame = frame;
		this.pose = pose;
	}

	public void add(Pose pose)
	{
		segments.put(pose.getName(), pose);

	}

	public void add(Joint joint)
	{
		segments.put(joint.getName(), joint);

	}

	public void setInvKinematics(InvKinematics invKinematics)
	{
		this.invKinematics = invKinematics;

	}

	public Frame getFrame()
	{
		return frame;
	}

	public Vector3D getPoint(String string)
	{
		Vector3D segmentPose = getSegmentPose(string);
		return segmentPose;
	}

	public Vector3D getSegmentPose(String segmentName)
	{

		List<Entry<String, Pose>> reversedSegments = new LinkedList<>();

		reversedSegments.addAll(segments.entrySet());
		Collections.reverse(reversedSegments);

		Vector3D ret = Vector3D.ZERO;

		boolean atSegment = segmentName == null;
		for (Entry<String, Pose> segment : reversedSegments)
		{
			if (!atSegment && segment.getKey().equalsIgnoreCase(segmentName))
			{
				atSegment = true;
			}
			if (atSegment)
			{
				// System.out.println("'" + segment.getKey() + "' "
				// + segment.getValue());
				ret = ret.add(segment.getValue().transform.transform);

				ret = segment.getValue().rotation.applyInverseTo(ret);
				// System.out.println(ret);
			}
		}
		return ret;
	}

	public void setPosition(Pose pose)
	{
		invKinematics.determine(this, pose);

	}

	public Vector3D getEndEffectorPose()
	{
		return getSegmentPose(null);
	}

	public Joint getJoint(String name)
	{
		return (Joint) segments.get(name);
	}

}
