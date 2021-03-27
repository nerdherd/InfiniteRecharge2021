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
public class BounceBackwards extends SequentialCommandGroup {
  /**
   * Creates a new Bounce.
   */
  private Drivetrain m_drive;
  public BounceBackwards(Drivetrain drive) {
    m_drive = drive;

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA),
      m_drive.m_kinematics, 
      DriveConstants.kRamseteMaxVolts);

    TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel)
    .setKinematics(m_drive.m_kinematics)
    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig configBackwards = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel)
    .setKinematics(m_drive.m_kinematics)
    .addConstraint(autoVoltageConstraint);
    configBackwards.setReversed(true);  

    Trajectory startToA3 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.762, 2.032, new Rotation2d(0)), 
    List.of(
      new Translation2d(1.905,2.032)),
      new Pose2d(2.286,3.810, new Rotation2d(Math.PI/2)),  
    config);

    Trajectory A3toA6 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(2.286,3.810, new Rotation2d(Math.PI/2)), 
    List.of(
      new Translation2d(2.413,1.778),  
      new Translation2d(3.810,0.762),
      new Translation2d(4.572,1.524)),
      new Pose2d(4.572,3.810, new Rotation2d(-Math.PI/2)),  
    configBackwards);


    Trajectory A6toA9 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(4.572,3.810, new Rotation2d(-Math.PI/2)), 
    List.of(
      new Translation2d(4.699,2.540),  
      new Translation2d(5.334,0.762),
      new Translation2d(6.096,0.762),
      new Translation2d(6.731,2.540)),
      new Pose2d(6.858,3.810, new Rotation2d(Math.PI/2)),  
    config);

    Trajectory A9toFinish = TrajectoryGenerator.generateTrajectory(
    new Pose2d(6.858,3.810, new Rotation2d(Math.PI/2)), 
    List.of(
      new Translation2d(6.858,3.810)
      ),
      new Pose2d(7.239,2.794, new Rotation2d(0)),  
    configBackwards);

    RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, 
            double angularVelocityRefRadiansPerSecond) {
              return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
            }
          };
          var leftController = new PIDController(DriveConstants.kLeftP, 0, 0);
          var rightController = new PIDController(DriveConstants.kRightP, 0, 0);
          RamseteCommand driveStartA3 = new RamseteCommand(startToA3, 
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

        RamseteCommand driveA3A6 = new RamseteCommand(A3toA6, 
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

        RamseteCommand driveA6A9 = new RamseteCommand(A6toA9, 
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

        RamseteCommand driveA9Finish = new RamseteCommand(A9toFinish, 
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

      new InstantCommand(() -> m_drive.setPose(new Pose2d(0.762, 2.032, new Rotation2d(0)))),  
      driveStartA3,
      driveA3A6,
      driveA6A9,
      driveA9Finish,
      new DriveStraightContinuous(m_drive, 0, 0)
    
    );
  
  }
}
