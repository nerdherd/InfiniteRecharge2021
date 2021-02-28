// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestRamsete extends SequentialCommandGroup {
  /** Creates a new TestRamsete. */
  private Drivetrain m_drive;
  public TestRamsete(Drivetrain drive) {
    m_drive = drive;

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA),
      m_drive.m_kinematics, 
      DriveConstants.kRamseteMaxVolts);

    TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel)
    .setKinematics(m_drive.m_kinematics)
    .addConstraint(autoVoltageConstraint);

    Trajectory startToFinish = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)), 
    List.of(new Translation2d(2,0)), new Pose2d(7, 0, new Rotation2d(0)), 
    config);

    RamseteCommand test = new RamseteCommand(startToFinish, 
    m_drive::getPose2d, 
    // new RamseteController(1, 0.65), //tune here
    new RamseteController(1, 0.65),
    new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), //change after Characterizing
    m_drive.m_kinematics, m_drive::getCurrentSpeeds,
    new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD), //change in constants after characterizing
    new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
    m_drive::setVoltage, 
    m_drive);

  addCommands(
    new InstantCommand(() -> m_drive.setPose(new Pose2d(0, 0, new Rotation2d(0)))),
    test,
    new DriveStraightContinuous(m_drive, 0, 0)
  );
  
}
}
