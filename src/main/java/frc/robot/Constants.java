// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class FieldPositionConstants
    {
	/** hub **/
	public static final double HUB_X = 4.6;
	public static final double HUB_Y = 4.1;

	/** trenches (left/right from driver pov) **/
	public static final double LEFT_TRENCH_CENTER_X = 4.5974;
	public static final double LEFT_TRENCH_CENTER_Y = 5.5626;
	
	public static final double RIGHT_TRENCH_CENTER_X = 4.5974;
	public static final double RIGHT_TRENCH_CENTER_Y = 0.6435;

	/** ball pile
	CL |---| FL
driver	   |   |
here	   |   |
	CR |---| FR
	 **/
	 public static final double BALLS_CLOSE_LEFT_X = 7.3434;
	 public static final double BALLS_CLOSE_LEFT_Y = 6.3315;
	 
	 public static final double BALLS_CLOSE_RIGHT_X = 7.3434;
	 public static final double BALLS_CLOSE_RIGHT_Y = 1.7112;

	 public static final double BALLS_FAR_LEFT_X = 9.1697;
	 public static final double BALLS_FAR_LEFT_Y = 6.3315;

	 public static final double BALLS_FAR_RIGHT_X = 9.1697;
	 public static final double BALLS_FAR_RIGHT_Y = 1.7112;
    }

}
