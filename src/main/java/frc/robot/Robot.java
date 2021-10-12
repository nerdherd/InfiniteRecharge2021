/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.nerdherd.lib.drivetrain.teleop.ArcadeDrive;
import com.nerdherd.lib.drivetrain.teleop.TankDrive;
import com.nerdherd.lib.logging.NerdyBadlog;
import com.nerdherd.lib.misc.AutoChooser;
import com.nerdherd.lib.motor.commands.ResetSingleMotorEncoder;
import com.nerdherd.lib.motor.single.SingleMotorMechanism;
import com.nerdherd.lib.motor.single.SingleMotorVictorSPX;
import com.nerdherd.lib.pneumatics.Piston;
import com.nerdherd.lib.sensor.analog.PressureSensor;
import com.nerdherd.lib.oi.XboxDriverOI;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.auto.AutoLineIntoTrench;
import frc.robot.commands.auto.AutoLineIntoTrenchFive;
import frc.robot.commands.auto.AutoLineTrenchThree;
import frc.robot.commands.auto.BarrelRacing;
import frc.robot.commands.auto.BasicAuto;
import frc.robot.commands.auto.Bounce;
import frc.robot.commands.auto.BounceBackwards;
import frc.robot.commands.auto.Lightspeed;
import frc.robot.commands.auto.Slalom;
import frc.robot.commands.intake.Stow;
// import frc.robot.commands.auto.StealTwoEnemyTrench;
// import frc.robot.commands.auto.StealTwoIntoTrench;
// import frc.robot.commands.auto.TenBallAuto;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Jevois;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  public static final String kDate = "2020_01_25_";

  public static AutoChooser chooser;
  public static Drive drive;
  public static Jevois jevois;
  public static Hood hood;
  // public static Indexer indexer;
  public static DriverStation ds;
  public static Shooter shooter;
  public static Hopper hopper;
  public static SingleMotorVictorSPX intakeRoll;
  public static Indexer index;
  public static PressureSensor pes;
  // public static SingleMotorMechanism motor;
  // public static PowerDistributionPanel pdp;
  public static Command m_autonomousCommand;
  public static Piston intake;
  public static Piston panelPos;
  public static SingleMotorMechanism panelRot;
  public static Climber climber;
  public static Limelight limelight;
  // public static OI oi;
  public static XboxOI oi;
  public static ResetSingleMotorEncoder hoodReset;
  public static SendableChooser<Command> autoChooser;

  // public static SingleMotorMechanism falcon;

  // public DifferentialDrivetrainSim m_driveSim =
  // new DifferentialDrivetrainSim(

  // );

  @Override
  public void robotInit() {
    drive = new Drive();
    hood = new Hood();
    climber = new Climber();
    index = new Indexer();
    // falcon = new SingleMotorMechanism(1, "shooter", true, true);
    limelight = new Limelight();
    // jevois = new Jevois(115200, SerialPort.Port.kUSB);
    // jevois.startCameraStream();
    shooter = new Shooter();
    // motor = new SingleMotorMechanism(6, "Motor", true, true);
    ds = DriverStation.getInstance();
    // climberRatchet = new Piston(6, 9);
    pes = new PressureSensor("name", 2);
    // climberRatchet.setReverse();
    CameraServer.getInstance().startAutomaticCapture();

    hopper = new Hopper();
    // index = new SingleMotorMechanism(RobotMap.kIndex, "Index", false, false);
    intakeRoll = new SingleMotorVictorSPX(RobotMap.kIntakeRoll, "intake rollers", false);
    intake = new Piston(RobotMap.kIntakePort1, RobotMap.kIntakePort2);
    // / pdp = new PowerDistributionPanel();
    // panelPos = new Piston(RobotMap.kPanelPort1ID, RobotMap.kPanelPort2ID);
    // panelRot = new SingleMotorMechanism(RobotMap.kPanelRollerID, "Control Panel",
    // false, false);
    limelight.setOff();
    hoodReset = new ResetSingleMotorEncoder(Robot.hood);

    // climberReset = new ParallelCommandGroup( ));
    // oi = new OI();

    oi = new XboxOI();

    drive.setDefaultCommand(new TankDrive(Robot.drive, Robot.oi));
    drive.configKinematics(DriveConstants.kTrackWidth, new Rotation2d(0), new Pose2d(0, 0, new Rotation2d(0)));
    NerdyBadlog.initAndLog("/home/lvuser/logs/", "FeedForwardTest", 0.02, shooter, hood, index, hopper, drive);

    // m_autonomousCommand = new BasicAuto();
    autoChooser = new SendableChooser<Command>();
    // autoChooser.setDefaultOption("Basic Auto", new BasicAuto());
    autoChooser.addOption("6Ball", new AutoLineTrenchThree(drive));
    autoChooser.addOption("Slalom", new Slalom(drive));
    autoChooser.addOption("Bounce", new Bounce(drive));
    autoChooser.addOption("BounceBackwards", new BounceBackwards(drive));
    autoChooser.addOption("Barrel", new BarrelRacing(drive));
    autoChooser.addOption("Light", new Lightspeed(drive));

    // autoChooser.addOption("8Ball", new AutoLineIntoTrenchFive(drive));
    // autoChooser.addOption("5 Steal", new StealTwoEnemyTrench(drive));
    // autoChooser.addOption("10 Ball", new StealTwoIntoTrench(drive));
    SmartDashboard.putData("Autos", autoChooser);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    drive.reportToSmartDashboard();
    // System.out.println(shooter.kF);
    shooter.reportToSmartDashboard();
    hopper.reportToSmartDashboard();
    index.reportToSmartDashboard();
    // jevois.reportToSmartDashboard();
    limelight.reportToSmartDashboard();
    // motor.reportToSmartDashboard();
    hood.reportToSmartDashboard();
    climber.reportToSmartDashboard();
    SmartDashboard.putNumber("DesiredAngle", Robot.hood.distToAngle(Robot.limelight.getDistanceWidth()));
    SmartDashboard.putNumber("Right Voltage 1", drive.getRightOutputVoltage());
    SmartDashboard.putNumber("Pressure: ", pes.getScaled());

    climber.reportToSmartDashboard();

  }

  @Override
  public void disabledInit() {
    hood.resetEncoder();
    Robot.climber.followerFalcon.resetEncoder();
    Robot.climber.mainFalcon.resetEncoder();

    drive.setCoastMode();
  }

  @Override
  public void disabledPeriodic() {
    if (oi.getRawButton(5)) { // TODO: What is this?
      hood.resetEncoder();
      climber.followerFalcon.resetEncoder();
      climber.mainFalcon.resetEncoder();
      drive.resetEncoders();
      drive.resetYaw();
      drive.resetXY();
    }
    // Robot.climber.followerFalcon.resetEncoder();
    // Robot.climber.mainFalcon.resetEncoder();

  }

  @Override
  public void autonomousInit() {
    drive.setBrakeMode();
    m_autonomousCommand = autoChooser.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void teleopInit() {
    drive.setCoastMode();
    // drive.setPose(new Pose2d(0, 0, new Rotation2d(Math.PI)));

  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    // drive.setCoastMode();

    if (oi.getTriggerAxis(Hand.kRight) >= 0.5) {
      new Stow().execute();
    }

    if (oi.getTriggerAxis(Hand.kLeft) >= 0.5) {
      Robot.hopper.setTopHopperPower(-0.41);
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
