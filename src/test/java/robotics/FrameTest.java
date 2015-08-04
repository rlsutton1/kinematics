package robotics;

import static org.junit.Assert.*;

import org.junit.Test;

public class FrameTest
{

	@Test
	public void testTranslateToNewFrameWithXTranslation()
	{
		Frame f1 = new Frame("F1", null, null);
		Point p1 = new Point(f1, 1, 2, 3);
		Frame f2 = new Frame("f2", f1, new Pose(1, 0, 0, 0, 0, 0));

		assertValue(f1.toChildFrame(f2, p1).getX(), 2);
		assertValue(f1.toChildFrame(f2, p1).getY(), 2);
		assertValue(f1.toChildFrame(f2, p1).getZ(), 3);
	}

	@Test
	public void testTranslateToNewFrameWithYTranslation()
	{
		Frame f1 = new Frame("F1", null, null);
		Point p1 = new Point(f1, 1, 2, 3);
		Frame f2 = new Frame("f2", f1, new Pose(0, 1, 0, 0, 0, 0));

		assertValue(f1.toChildFrame(f2, p1).getX(), 1);
		assertValue(f1.toChildFrame(f2, p1).getY(), 3);
		assertValue(f1.toChildFrame(f2, p1).getZ(), 3);
	}

	@Test
	public void testTranslateToNewFrameWithZTranslation()
	{
		Frame f1 = new Frame("F1", null, null);
		Point p1 = new Point(f1, 1, 2, 3);
		Frame f2 = new Frame("f2", f1, new Pose(0, 0, 1, 0, 0, 0));

		assertValue(f1.toChildFrame(f2, p1).getX(), 1);
		assertValue(f1.toChildFrame(f2, p1).getY(), 2);
		assertValue(f1.toChildFrame(f2, p1).getZ(), 4);

	}

	@Test
	public void testTranslateToNewFrameWithXRotation()
	{
		Frame f1 = new Frame("F1", null, null);
		Point p1 = new Point(f1, 0, 0, 1);
		Frame f2 = new Frame("f2", f1, new Pose(0, 0, 0, Math.PI / 2, 0, 0));

		assertValue(f1.toChildFrame(f2, p1).getX(), 0);
		assertValue(f1.toChildFrame(f2, p1).getY(), 1);
		assertValue(f1.toChildFrame(f2, p1).getZ(), 0);

	}

	@Test
	public void testTranslateToNewFrameWithYRotation()
	{
		Frame f1 = new Frame("F1", null, null);
		Point p1 = new Point(f1, 1, 0, 0);
		Frame f2 = new Frame("f2", f1, new Pose(0, 0, 0, 0, Math.PI / 2, 0));

		assertValue(f1.toChildFrame(f2, p1).getX(), 0);
		assertValue(f1.toChildFrame(f2, p1).getY(), 0);
		assertValue(f1.toChildFrame(f2, p1).getZ(), 1);

	}

	@Test
	public void testTranslateToNewFrameWithZRotation()
	{
		Frame f1 = new Frame("F1", null, null);
		Point p1 = new Point(f1, 0, 1, 0);
		Frame f2 = new Frame("f2", f1, new Pose(0, 0, 0, 0, 0, Math.PI / 2));

		assertValue(f1.toChildFrame(f2, p1).getX(), 1);
		assertValue(f1.toChildFrame(f2, p1).getY(), 0);
		assertValue(f1.toChildFrame(f2, p1).getZ(), 0);

	}

	private void assertValue(double value, int expected)
	{
		assertTrue("got " + value + " expected " + expected,
				Math.abs(expected - value) < 0.01);
	}
}
