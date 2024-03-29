// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands.auto;

// import java.util.List;

// import com.nerdherd.lib.drivetrain.auto.DriveStraightContinuous;
// import com.nerdherd.lib.drivetrain.experimental.Drivetrain;

// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.controller.RamseteController;
// import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
// import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Robot;
// import frc.robot.commands.intake.IntakeBalls;
// import frc.robot.commands.intake.Stow;
// import frc.robot.constants.DriveConstants;
// // import frc.robot.auto.BasicAuto;

// public class StealTwoIntoTrench extends SequentialCommandGroup {
//   private Drivetrain m_drive;

//   public StealTwoIntoTrench(Drivetrain drive) {
//     m_drive = drive;
//     drive.setPose(new Pose2d(DriveConstants.kAutoLineMeters, DriveConstants.kEnemyTrenchMetersY, new Rotation2d(0)));
  
//     var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
//       new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA),
//       m_drive.m_kinematics, 
//       DriveConstants.kRamseteMaxVolts);
  
//     var autoCentripetalAccelerationConstraint = new CentripetalAccelerationConstraint(DriveConstants.kMaxCentripetalAcceleration);
  
//     TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel);
//     config.addConstraints(List.of(autoVoltageConstraint, autoCentripetalAccelerationConstraint));

//     Trajectory stealTrenchPath = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(DriveConstants.kAutoLineMeters, DriveConstants.kEnemyTrenchMetersY, new Rotation2d(0)),
//       List.of(), 
//       new Pose2d(DriveConstants.kEnemyTrenchMetersX, DriveConstants.kEnemyTrenchMetersY, new Rotation2d(0)),
//       config);

//     RamseteCommand stealTrench = new RamseteCommand(stealTrenchPath, 
//       m_drive::getPose2d, new RamseteController(2.0, 0.7), 
//       new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), 
//       m_drive.m_kinematics, m_drive::getCurrentSpeeds, 
//       new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD), 
//       new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
//       m_drive::setVoltage, m_drive);
    
      
//     Trajectory stealIntoShoot = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(DriveConstants.kEnemyTrenchMetersX, DriveConstants.kEnemyTrenchMetersY, new Rotation2d(0)),
//       List.of(new Translation2d(3.777, -6.569)), new Pose2d(DriveConstants.kAutoLineMeters, DriveConstants.kGoalMetersY, new Rotation2d(Math.PI)),
//       config);

//     RamseteCommand stealShoot = new RamseteCommand(stealIntoShoot, 
//       m_drive::getPose2d, new RamseteController(2.0, 0.7), 
//       new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), 
//       m_drive.m_kinematics, m_drive::getCurrentSpeeds, 
//       new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD), 
//       new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
//       m_drive::setVoltage, m_drive);
    
//       Trajectory shootIntoTrench = TrajectoryGenerator.generateTrajectory(
//         new Pose2d(DriveConstants.kAutoLineMeters, DriveConstants.kGoalMetersY, new Rotation2d(Math.PI)),
//         List.of(new Translation2d(2, -0.85), new Translation2d(DriveConstants.kTrenchMetersX, DriveConstants.kTrenchMetersY)), 
//         new Pose2d(DriveConstants.kTrenchThirdBallX, DriveConstants.kEndTrenchMetersY, new Rotation2d(0)),
//         config);
  
  
//       RamseteCommand trenchPath = new RamseteCommand(shootIntoTrench, 
//         m_drive::getPose2d, new RamseteController(1.25, 0.4), 
//         new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), 
//         m_drive.m_kinematics, m_drive::getCurrentSpeeds, 
//         new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD), 
//         new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
//         m_drive::setVoltage, m_drive);

//         TrajectoryConfig config2 = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel);
//         config2.addConstraints(List.of(autoVoltageConstraint, autoCentripetalAccelerationConstraint));
//         config2.setReversed(true);
//         Trajectory shootFromTrench = TrajectoryGenerator.generateTrajectory(
//           new Pose2d(DriveConstants.kEndTrenchMetersX, -0.6, new Rotation2d(0)),
//           List.of(new Translation2d(DriveConstants.kTrenchMetersX, -0.6)), 
//           new Pose2d(5.430,-1.568, new Rotation2d(3.23)),
//           config2);
    
    
//         RamseteCommand trenchShoot = new RamseteCommand(shootFromTrench, 
//           m_drive::getPose2d, new RamseteController(1.5, 0.7), 
//           new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA), 
//           m_drive.m_kinematics, m_drive::getCurrentSpeeds, 
//           new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD), 
//           new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
//           m_drive::setVoltage, m_drive);
        
     
    
//     addCommands(
//       new InstantCommand(()-> Robot.hood.setAngleMotionMagic(-10)),
//       new ParallelRaceGroup(new IntakeBalls(), stealTrench),
//       new ParallelRaceGroup(new Stow(), stealShoot),
//       new BasicAutoNoMove(),
//       new ParallelRaceGroup(new IntakeBalls(), trenchPath),
//       new ParallelRaceGroup(new Stow(), trenchShoot),
//       new DriveStraightContinuous(m_drive, 0, 0),
//       new BasicAutoNoMove()
//     );
//   }
// }