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
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
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

    var autoCentripetalAccelerationConstraint = new CentripetalAccelerationConstraint(DriveConstants.kMaxCentripetalAcceleration);


    // var CentripetalAccelerationConstraint = new DifferentialDriveCentro
    TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel)
    .setKinematics(m_drive.m_kinematics)
    .addConstraints(List.of(autoVoltageConstraint, autoCentripetalAccelerationConstraint));

    Trajectory startToFinish = TrajectoryGenerator.generateTrajectory(
    new Pose2d(2.54, -1.143, new Rotation2d(0)), 
    List.of(
      new Translation2d(3.429,-1.651),
      new Translation2d(3.937,-2.54), 
      new Translation2d(3.429,-3.429),
      new Translation2d(2.667,-4.191),
      new Translation2d(2.54,-4.826),
      new Translation2d(2.794,-5.842),
      new Translation2d(3.556,-6.604),
      new Translation2d(4.064,-6.858),
      new Translation2d(3.556,-6.985),
      new Translation2d(2.667,-8.001),
      new Translation2d(1.524,-8.255),
      new Translation2d(1.016,-8.001),
      new Translation2d(1.143,-6.35),
      new Translation2d(0.889,-4.826),
      new Translation2d(1.27,-4.191),
      new Translation2d(2.159,-2.921),
      new Translation2d(2.413,-2.235),
      new Translation2d(1.905,-2.921),
      new Translation2d(3.556,-1.778),
      new Translation2d(3.937,-2.54), 
      new Translation2d(3.429,-3.429),
      new Translation2d(2.667,-4.191),
      new Translation2d(2.54,-4.826),
      new Translation2d(2.794,-5.842),
      new Translation2d(3.556,-6.604),
      new Translation2d(4.064,-6.6858),
      new Translation2d(3.556,-6.985),
      new Translation2d(2.667,-8.001),
      new Translation2d(1.524,-8.255),
      new Translation2d(1.016,-8.001),
      new Translation2d(1.143,-6.35),
      new Translation2d(0.889,-4.826),
      new Translation2d(1.27,-4.191),
      new Translation2d(2.159,-2.921),
      new Translation2d(2.2352,-2.413),
      new Translation2d(1.905,-1.905)),
    new Pose2d(0.762, -1.524, new Rotation2d(Math.PI)), 
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
        disabledRamsete,
        new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), //change after Characterizing
        m_drive.m_kinematics, m_drive::getCurrentSpeeds,
        leftController,
        rightController, 
        (leftVolts, rightVolts) -> {
          m_drive.setVoltage(leftVolts, rightVolts);
          
          SmartDashboard.putNumber("Left Wheel speeds", m_drive.getCurrentSpeeds().leftMetersPerSecond);
          SmartDashboard.putNumber("Left Desired Speeds", leftController.getSetpoint());
          SmartDashboard.putNumber("Left Position Error", leftController.getPositionError());
    
          SmartDashboard.putNumber("Right Wheel speeds", m_drive.getCurrentSpeeds().rightMetersPerSecond);
          SmartDashboard.putNumber("Right Desired Speeds", rightController.getSetpoint());
          SmartDashboard.putNumber("Velocity Position Error", rightController.getPositionError());
          // prevTime = time;
    
      }, 
      m_drive);

    addCommands(
    new InstantCommand(() -> m_drive.setPose(new Pose2d(2.54, -1.143, new Rotation2d(0)))),  
    driveStartToFinish,
    new DriveStraightContinuous(m_drive, 0, 0)
    
    );
  
  }
}
