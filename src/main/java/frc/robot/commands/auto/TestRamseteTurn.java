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
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
public class TestRamseteTurn extends SequentialCommandGroup {
  /**
   * Creates a new Slalom.
   */
  private Drivetrain m_drive;
  public TestRamseteTurn(Drivetrain drive) {
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
    List.of(
      new Translation2d(2,-2)),
    new Pose2d(0, -4, new Rotation2d(Math.PI)), 
    config);

    RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
              double angularVelocityRefRadiansPerSecond) {
          return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
      }
  };

  var leftController = new PIDController(DriveConstants.kLeftP, 0, 0);
  var rightController = new PIDController(DriveConstants.kRightP, 0, 0);
    RamseteCommand driveStartToFinish = new RamseteCommand(startToFinish, 
    m_drive::getPose2d, 
    new RamseteController(2, 0.7), //tune here
    // disabledRamsete,
    new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), //change after Characterizing
    m_drive.m_kinematics, m_drive::getCurrentSpeeds,
    leftController, //change in constants after characterizing
    rightController,
    (leftVolts, rightVolts) -> {
      m_drive.setVoltage(leftVolts, rightVolts);

      SmartDashboard.putNumber("Left wheel speeds", m_drive.getCurrentSpeeds().leftMetersPerSecond);
      SmartDashboard.putNumber("Left Desired Speeds", leftController.getSetpoint());

      SmartDashboard.putNumber("Right wheel speeds", m_drive.getCurrentSpeeds().rightMetersPerSecond);
      SmartDashboard.putNumber("Right Desired Speeds", rightController.getSetpoint());
  },
    m_drive);
    
    // Trajectory d4TOd8

    // Trajectory d8Tod10

    // Trajectory d10tod8

    // Trajectory d8tod4

    // Trajectory d4toFinish

    addCommands(
    new InstantCommand(() -> m_drive.setPose(new Pose2d(0, 0, new Rotation2d(0)))),  
    driveStartToFinish,
    new DriveStraightContinuous(m_drive, 0, 0)
    
    );
  
  }
}
