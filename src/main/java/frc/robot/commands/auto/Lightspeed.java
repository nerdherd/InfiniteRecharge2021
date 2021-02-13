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
public class Lightspeed extends SequentialCommandGroup {
  /**
   * Creates a new Lightspeed.
   */
  private Drivetrain m_drive;
  public Lightspeed(Drivetrain drive) {
    m_drive = drive;

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA),
      m_drive.m_kinematics, 
      DriveConstants.kRamseteMaxVolts);

    TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel)
    .setKinematics(m_drive.m_kinematics)
    .addConstraint(autoVoltageConstraint);

    Trajectory startToFinish = TrajectoryGenerator.generateTrajectory(
    new Pose2d(1.143, 2.540, new Rotation2d(0)), 
    List.of(
      new Translation2d(1.651,3.429),
      new Translation2d(2.540,3.937), 
      new Translation2d(3.429,3.429),
      new Translation2d(4.191,2.667),
      new Translation2d(4.826,2.540),
      new Translation2d(5.842,2.794),
      new Translation2d(6.604,3.556),
      new Translation2d(6.858,4.064),
      new Translation2d(6.985,3.556),
      new Translation2d(8.001,2.667),
      new Translation2d(8.255,1.524),
      new Translation2d(8.001,1.016),
      new Translation2d(6.350,1.143),
      new Translation2d(4.826,0.889),
      new Translation2d(4.191,1.270),
      new Translation2d(2.921,2.159),
      new Translation2d(2.413,2.235),
      new Translation2d(1.905,2.921),
      new Translation2d(1.778,3.556),
      new Translation2d(2.540,3.937), 
      new Translation2d(3.429,3.429),
      new Translation2d(4.191,2.667),
      new Translation2d(4.826,2.540),
      new Translation2d(5.842,2.794),
      new Translation2d(6.604,3.556),
      new Translation2d(6.858,4.064),
      new Translation2d(6.985,3.556),
      new Translation2d(8.001,2.667),
      new Translation2d(8.255,1.524),
      new Translation2d(8.001,1.016),
      new Translation2d(6.350,1.143),
      new Translation2d(4.826,0.889),
      new Translation2d(4.191,1.270),
      new Translation2d(2.921,2.159),
      new Translation2d(2.413,2.235),
      new Translation2d(1.905,1.905)),
    new Pose2d(1.524, 0.762, new Rotation2d(Math.PI)), 
    config);

    RamseteCommand driveStartToFinish = new RamseteCommand(startToFinish, 
    m_drive::getPose2d, 
    new RamseteController(2.0, 0.7), //tune here
    new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), //change after Characterizing
    m_drive.m_kinematics, m_drive::getCurrentSpeeds,
    new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD), //change in constants after characterizing
    new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
    m_drive::setVoltage, 
    m_drive);

    addCommands(
    new InstantCommand(() -> m_drive.setPose(new Pose2d(1.143, 2.540, new Rotation2d(0)))),  
    driveStartToFinish,
    new DriveStraightContinuous(m_drive, 0, 0)
    
    );
  
  }
}
