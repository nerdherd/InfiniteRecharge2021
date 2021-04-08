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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.intake.Stow;
import frc.robot.constants.DriveConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class GalacticReal extends SequentialCommandGroup {
  /**
   * Creates a new GalacticPathARed.
   */
  private Drivetrain m_drive;
  public GalacticReal(Drivetrain drive) {
    m_drive = drive;


    TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel)
    .setKinematics(m_drive.m_kinematics);

    Trajectory startToFinish = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.286, -0.762, new Rotation2d(-Math.PI/2)), 
      List.of(
        new Translation2d(3.048,-2.286),
        new Translation2d(2.286,-2.286), 

        new Translation2d(1.524,-3.81),
        new Translation2d(0.762,-3.81),
        new Translation2d(0.762,-4.572),
        new Translation2d(1.524,-4.572),
        new Translation2d(3.81,-4.572),
        new Translation2d(3.048,-5.334),
        new Translation2d(3.048,-6.096),

        new Translation2d(2.286,-6.858),
        new Translation2d(1.524,-6.858),
        new Translation2d(0.762,-6.858),
        new Translation2d(1.524,-7.62)
        ),

      new Pose2d(2.286, -7.62, new Rotation2d(-Math.PI/2)), 
      config);

      RamseteController disabledRamsete = new RamseteController() {
        @Override
        public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, 
            double angularVelocityRefRadiansPerSecond) {
              return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
            }
          };
          // var leftController = new PIDController(DriveConstants.kLeftP, 0, 0);
          // var rightController = new PIDController(DriveConstants.kRightP, 0, 0);
          RamseteCommand driveStartToFinish = new RamseteCommand(startToFinish, 
            m_drive::getPose2d, 
            // disabledRamsete,
            new RamseteController(2.0, 0.4),
            // new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), //change after Characterizing
            m_drive.m_kinematics, 
            (leftVel, rightVel) -> {
              m_drive.setVelocity(3886 * leftVel, 3886 * rightVel);

            SmartDashboard.putNumber("Left vel", leftVel);
            SmartDashboard.putNumber("Right vel", rightVel);

            
            // leftController,
            // rightController, 
            // (leftVolts, rightVolts) -> {
            //   m_drive.setVoltage(leftVolts, rightVolts);
            
            // SmartDashboard.putNumber("Left Wheel speeds", m_drive.getCurrentSpeeds().leftMetersPerSecond);
            // SmartDashboard.putNumber("Left Desired Speeds", leftController.getSetpoint());
            // SmartDashboard.putNumber("Left Position Error", leftController.getPositionError());
      
            // SmartDashboard.putNumber("Right Wheel speeds", m_drive.getCurrentSpeeds().rightMetersPerSecond);
            // SmartDashboard.putNumber("Right Desired Speeds", rightController.getSetpoint());
            // SmartDashboard.putNumber("Velocity Position Error", rightController.getPositionError());
            // prevTime = time;
      
        }, 
        m_drive);
    

    addCommands(
      new InstantCommand(() -> m_drive.setPose(new Pose2d(2.286, -0.762, new Rotation2d(Math.PI / 2)))),  
      new ParallelRaceGroup(new IntakeBalls(), driveStartToFinish),
      new DriveStraightContinuous(m_drive, 0, 0)
      // new Stow()
    );
  
  }
}