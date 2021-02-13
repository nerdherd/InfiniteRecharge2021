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
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.intake.Stow;
import frc.robot.commands.shooting.ShootBall;
import frc.robot.commands.shooting.TrenchShot;
import frc.robot.commands.vision.TurnToAngle;
import frc.robot.commands.vision.TurnToAngleLime;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoLineTrenchThree extends SequentialCommandGroup {
  private Drivetrain m_drive;
  public AutoLineTrenchThree(Drivetrain drive) {
    m_drive = drive;
    
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA),
      m_drive.m_kinematics, 
      DriveConstants.kRamseteMaxVolts);

      var autoCentripetalAccelerationConstraint = new CentripetalAccelerationConstraint(DriveConstants.kMaxCentripetalAcceleration);
  
    TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel);
    config.addConstraints(List.of(autoVoltageConstraint, autoCentripetalAccelerationConstraint));

    
      Trajectory goalIntoTrench = TrajectoryGenerator.generateTrajectory(
      new Pose2d(DriveConstants.kAutoLineMeters, DriveConstants.kGoalMetersY, new Rotation2d(Math.PI)),
      List.of(new Translation2d(2, -0.85), new Translation2d(DriveConstants.kTrenchMetersX, -0.6)), 
      new Pose2d(7.69, -0.6, new Rotation2d(0)),
      config);


    RamseteCommand driveToTrench = new RamseteCommand(goalIntoTrench, 
      m_drive::getPose2d, new RamseteController(1.05, 0.14), 
      new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), 
      m_drive.m_kinematics, m_drive::getCurrentSpeeds, 
      new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD), 
      new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
      m_drive::setVoltage, m_drive);
    
      

      TrajectoryConfig config2 = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel);
      config2.addConstraints(List.of(autoVoltageConstraint, autoCentripetalAccelerationConstraint));
      config2.setReversed(true);
      Trajectory shootFromTrench = TrajectoryGenerator.generateTrajectory(
        new Pose2d(7.69, -0.6, new Rotation2d(0)),
        List.of(new Translation2d(DriveConstants.kTrenchMetersX, -0.6)), 
        new Pose2d(5.430,-1.568, new Rotation2d(Math.toRadians(170))),
        config2);
  
  
      RamseteCommand trenchShoot = new RamseteCommand(shootFromTrench, 
        m_drive::getPose2d, new RamseteController(1.5, 0.65), 
        new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), 
        m_drive.m_kinematics, m_drive::getCurrentSpeeds, 
        new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD), 
        new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
        m_drive::setVoltage, m_drive);
      
    
    addCommands(
      new BasicAutoNoMove(),
      new InstantCommand(() -> m_drive.setPose(new Pose2d(DriveConstants.kAutoLineMeters, DriveConstants.kGoalMetersY, new Rotation2d(Math.PI)))),
      // new IntakeBalls(),
      new InstantCommand(()-> Robot.hood.setAngleMotionMagic(-10)),
      new ParallelRaceGroup(new IntakeBalls(), driveToTrench),
      // new InstantCommand(() -> Robo)
      new InstantCommand(() -> Robot.shooter.setVelocity(ShooterConstants.kTrenchShotVelocity)),
      trenchShoot,
      // new ParallelCommandGroup(),
      new ParallelCommandGroup(new InstantCommand(() -> Robot.hood.setAngle(34))),
      new ParallelRaceGroup(new WaitCommand(1), new TurnToAngleLime(VisionConstants.kRotP_lime)),
      // new ParallelRaceGroup(new WaitCommand(1), new TrenchShot()),
      new ParallelRaceGroup(new ShootBall(), new WaitCommand(5))
      // new DriveStraightContinuous(m_drive, 0, 0)
      


    );
  }
}
