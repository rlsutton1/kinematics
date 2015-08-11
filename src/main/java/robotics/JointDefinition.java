package robotics;



/**
 * A JointDefinition describes a joint and are immutable.
 * @author bsutton
 *
 */
abstract public class JointDefinition implements Definition
{

	private Axis axisOfRotation;
	
	private String name;
	private iJoint joint;

	public JointDefinition(String name, Axis axisOfRotation)
	{
		this.name = name;
		this.axisOfRotation = axisOfRotation;
		this.joint = null;
	}

	public ComputationalPose createPose()
	{
		return new ComputationalPose(this);
	}
	
	public Axis getAxisOfRotation()
	{
		return axisOfRotation;
	}

//	public void registerJoint(iJoint joint)
//	{
//		this.joint = joint;
//	}

	public String getName()
	{
		return name;
	}



	public iJoint getJoint()
	{
		// TODO: this could cause the creation of the joint to be delayed. Is this a problem?
		if (this.joint == null)
			this.joint = createJoint();
		
		return this.joint;
	}

	protected abstract iJoint createJoint();

}
