/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import java.util.List;

import com.nerdherd.lib.drivetrain.auto.DriveStraightContinuous;
import com.nerdherd.lib.drivetrain.experimental.Drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DriveConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Slalom extends SequentialCommandGroup {
  /**
   * Creates a new Slalom.
   */
  private Drivetrain m_drive;
  public Slalom(Drivetrain drive) {
    m_drive = drive;

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA),
      m_drive.m_kinematics, 
      DriveConstants.kRamseteMaxVolts);

    TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel)
    .setKinematics(m_drive.m_kinematics)
    .addConstraint(autoVoltageConstraint);

    Trajectory startToFinish = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.762, -0.762, new Rotation2d(3*Math.PI/2)), 
    List.of(
      new Translation2d(1.524,-2.286),
      new Translation2d(2.413,-3.048), 
      new Translation2d(3.048,-4.572),
      new Translation2d(2.286,-6.223),
      new Translation2d(1.54,-6.858),
      new Translation2d(0.762,-7.62),
      new Translation2d(1.524,-8.89),
      new Translation2d(2.54,-7.62),
      new Translation2d(1.524,-6.68),
      new Translation2d(6.096,0.762),
      new Translation2d(4.572,3.048),
      new Translation2d(3.048,1.016),
      new Translation2d(2.286,1.524)),
    new Pose2d(0.762, 2.286, new Rotation2d(Math.PI/2)), 
    config);

    RamseteCommand driveStartToFinish = new RamseteCommand(startToFinish, 
    m_drive::getPose2d, 
    new RamseteController(1, 0.65), //tune here
    new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), //change after Characterizing
    m_drive.m_kinematics, m_drive::getCurrentSpeeds,
    new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD), //change in constants after characterizing
    new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
    m_drive::setVoltage, 
    m_drive);
    
    // Trajectory d4TOd8

    // Trajectory d8Tod10

    // Trajectory d10tod8

    // Trajectory d8tod4

    // Trajectory d4toFinish

    addCommands(
    new InstantCommand(() -> m_drive.setPose(new Pose2d(0.762, -0.762, new Rotation2d(3*Math.PI/2)))),  
    driveStartToFinish,
    new DriveStraightContinuous(m_drive, 0, 0)
    
    );
  
  }
}
