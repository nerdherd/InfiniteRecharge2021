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
import frc.robot.commands.shooting.ShootBall;
import frc.robot.commands.shooting.TrenchShot;
import frc.robot.constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PowerPort extends SequentialCommandGroup {
  /**
   * Creates a new PowerPort.
   */
  private Drivetrain m_drive;
  public PowerPort(Drivetrain drive) {
    m_drive = drive;

    double shootingY = -2; //fix
    double reIntroY = -5; //fix


    TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel)
    .setKinematics(m_drive.m_kinematics);

    Trajectory toReIntro = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.286, shootingY, new Rotation2d(Math.PI/2)), 
      List.of(
        new Translation2d(2.286, 5.334)),
      new Pose2d(2.286, reIntroY, new Rotation2d(-Math.PI/2)), 
      config);

    Trajectory toShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.286, reIntroY, new Rotation2d(-Math.PI/2)), 
      List.of(
        new Translation2d(2.286, 5.334)),
      new Pose2d(2.286, shootingY, new Rotation2d(Math.PI/2)), 
      config);


    // var leftController = new PIDController(DriveConstants.kLeftP, 0, 0);
    // var rightController = new PIDController(DriveConstants.kRightP, 0, 0);
    RamseteCommand driveToReIntro = new RamseteCommand(toReIntro, 
      m_drive::getPose2d, 
      // disabledRamsete,
      new RamseteController(2.0, 0.4),
      // new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), //change after Characterizing
      m_drive.m_kinematics, 
      (leftVel, rightVel) -> {
        m_drive.setVelocity(3886 * leftVel, 3886 * rightVel);

      SmartDashboard.putNumber("Left vel", leftVel);
      SmartDashboard.putNumber("Right vel", rightVel);

  }, 
  m_drive);
    
    RamseteCommand driveToShoot = new RamseteCommand(toShoot, 
      m_drive::getPose2d, 
      // disabledRamsete,
      new RamseteController(2.0, 0.4),
      // new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), //change after Characterizing
      m_drive.m_kinematics, 
      (leftVel, rightVel) -> {
        m_drive.setVelocity(3886 * leftVel, 3886 * rightVel);

      SmartDashboard.putNumber("Left vel", leftVel);
      SmartDashboard.putNumber("Right vel", rightVel);

}, 
m_drive);

    addCommands(

      new InstantCommand(() -> m_drive.setPose(new Pose2d(2.286, shootingY, new Rotation2d(Math.PI / 2)))),  //set pose

      new ParallelRaceGroup(new TrenchShot(), new WaitCommand(2.75)),
      new ParallelRaceGroup(new ShootBall(), new WaitCommand(2.75)),
      driveToReIntro,
      new ParallelRaceGroup(new IntakeBalls(), new WaitCommand(2.75))

      // new InstantCommand(() -> new Stow()),
      // driveToShoot

      // new InstantCommand(() -> new TrenchShot()),
      // new WaitCommand(2.75),
      // new InstantCommand(() -> new ShootBall()), 
      // new WaitCommand(2.75),
      // driveToReIntro,
      // new InstantCommand(() -> new IntakeBalls()),
      // new WaitCommand(2.75),
      // // new InstantCommand(() -> new Stow()),
      // driveToShoot,

      // new InstantCommand(() -> new TrenchShot()),
      // new WaitCommand(2.75),
      // new InstantCommand(() -> new ShootBall()), 
      // new WaitCommand(2.75),
      // driveToReIntro,
      // new InstantCommand(() -> new IntakeBalls()),
      // new WaitCommand(2.75),
      // // new InstantCommand(() -> new Stow()),
      // driveToShoot,
      
      // new InstantCommand(() -> new TrenchShot()),
      // new WaitCommand(2.75),
      // new InstantCommand(() -> new ShootBall()), 
      // new WaitCommand(2.75),
      // driveToReIntro,
      // new InstantCommand(() -> new IntakeBalls()),
      // new WaitCommand(2.75),
      // // new InstantCommand(() -> new Stow()),
      // driveToShoot
      
    );
  
  }
}
