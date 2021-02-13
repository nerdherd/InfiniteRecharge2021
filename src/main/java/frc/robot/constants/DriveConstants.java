/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

/**
 * Add your docs here.
 */
public class DriveConstants {
    public static final double kLeftP = 8.78; //replace these
	public static final double kLeftI = 0;
	public static final double kLeftD = 0;

	public static final double kRightP = 10.4;
	public static final double kRightI = 0;
    public static final double kRightD = 0;
    
    public static final double kDriveMaxVel = 1;
    public static final double kDriveMaxAccel = 2.35;
    
    public static final double kGoalMetersY = -2.404;
    public static final double kAutoLineMeters = 3.048;
    public static final double kEnemyTrenchMetersX = 6.359;
    public static final double kEnemyTrenchMetersY = -7.506;
    
    
    public static final double kTrenchMetersX = 5.248;
    public static final double kTrenchMetersY = -0.70485;
    public static final double kTrenchThirdBallX = 7.991602;
    public static final double kEndTrenchMetersX = 9.62406;
    public static final double kEndTrenchMetersY = -0.6;
    
    public static final double kLeftRamseteS = 0.39;
    public static final double kLeftRamseteV = 2.05;
    public static final double kLeftRamseteA = 0.214;

    public static final double kramseteS = 0.286; //replace these when analyzing middle? 
    public static final double kramseteV = 2.08;
    public static final double kramseteA = 0.234;

    // public static final double kramseteP = 9.52;
    // public static final double kramseteI = 0;
    // public static final double kramseteD = 0;

    public static final double kRightRamseteS = 0.194;
    public static final double kRightRamseteV = 2.11;
    public static final double kRightRamseteA = 0.258;

	public static final double kRamseteMaxVolts = 10;

    
    
    public static double kLeftStatic = 0.39; 
    public static double kRightStatic =  0.194;

    public static double kLeftF = 0.0;
    public static double kRightF = 0.0;

    public static int kMaxVelocity = 10000;
    public static double kLeftTicksPerFoot = 12018.35;
    public static double kRightTicksPerFoot = 12018.35;

    public static double kTrackWidth = 0.8484;
	public static final double kMaxCentripetalAcceleration = 2.1;
    

}
