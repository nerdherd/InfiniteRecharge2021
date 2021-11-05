/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.nerdherd.lib.drivetrain.auto.DriveDistanceMotionMagic;
import com.nerdherd.lib.drivetrain.auto.DriveStraightContinuous;
import com.nerdherd.lib.drivetrain.auto.ResetDriveEncoders;
import com.nerdherd.lib.drivetrain.auto.ResetGyro;
import com.nerdherd.lib.drivetrain.characterization.DriveCharacterizationTest;
import com.nerdherd.lib.drivetrain.shifting.ShiftHigh;
import com.nerdherd.lib.drivetrain.shifting.ShiftLow;
import com.nerdherd.lib.motor.commands.ResetSingleMotorEncoder;
import com.nerdherd.lib.motor.commands.SetMotorPower;
import com.nerdherd.lib.motor.single.SingleMotorMechanism;
import com.nerdherd.lib.oi.DefaultOI;
import com.nerdherd.lib.oi.XboxDriverOI;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

// import org.graalvm.compiler.lir.aarch64.AArch64Move.StoreConstantOp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.InGameResetHood;
import frc.robot.commands.auto.AutoLineIntoTrench;
import frc.robot.commands.auto.BarrelRacing;
import frc.robot.commands.auto.Bounce;
import frc.robot.commands.auto.BounceBackwards;
import frc.robot.commands.auto.GalacticPathABlue;
import frc.robot.commands.auto.GalacticPathARed;
import frc.robot.commands.auto.GalacticPathBBlue;
import frc.robot.commands.auto.GalacticPathBRed;
import frc.robot.commands.auto.GalacticReal;
import frc.robot.commands.auto.Lightspeed;
import frc.robot.commands.auto.PowerPort;
import frc.robot.commands.auto.Slalom;
import frc.robot.commands.auto.TestRamsete;
import frc.robot.commands.auto.TestRamseteTurn;
import frc.robot.commands.auto.Trapezoid;
// import frc.robot.commands.AutolineShot;
// import frc.robot.commands.ShootBall;
// import frc.robot.commands.ShootBallTemp;
// import frc.robot.commands.ShootBallTempStop;
// import frc.robot.commands.TrenchShot;
// import frc.robot.commands.WallShot;
// import frc.robot.commands.auto.StealTwoEnemyTrench;
import frc.robot.commands.climber.ClimberLift;
import frc.robot.commands.climber.ClimberReady;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.intake.Stow;
import frc.robot.commands.other.SetAngle;
import frc.robot.commands.other.ShootBallTemp;
import frc.robot.commands.shooting.AutolineShot;
import frc.robot.commands.shooting.RendezvousShot;
import frc.robot.commands.shooting.ShootBall;
// import frc.robot.commands.ShootBallTemp;
// import frc.robot.commands.ShootBallTempStop;
import frc.robot.commands.shooting.TrenchShot;
import frc.robot.commands.shooting.WallShot;
import frc.robot.commands.vision.DistanceToAngle;
import frc.robot.commands.vision.TurnToAngleLime;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Indexer.IndexerState;

/**
 * Xbox OI Class
 */
public class XboxOI extends XboxDriverOI {

    public JoystickButton startShooting_2, startShootingOld_3, trenchShot_7, autolineShot_9, stow_10, wallShot_11, autoDistance_12, hoodAngle_5, outtake_6, togglePipeline_4, rendezvousShot_8, outtakeBrushes_8, intake_1;

    public JoystickButton shiftLow_A, shiftHigh_B, climbReady_X, climbLift_Y, turnToAngle_LB, stow_RB, resetEncoder_BA;
    public Button intake_RT, outtake_LT;

    public final int BUTTON_A = 1, BUTTON_B = 2, BUTTON_X = 3, BUTTON_Y = 4, BUTTON_LB = 5, BUTTON_RB = 6, BUTTON_BACK = 7, BUTTON_START = 8, BUTTON_LEFT_STICK = 9, BUTTON_RIGHT_STICK = 10;

    private double m_triggerThreshold = 0.25;

    public XboxOI() {
        this(0);
    }

    public XboxOI(double deadband) {
        super(deadband);
        intake_1 = new JoystickButton(super.operatorJoy, 1);
        startShooting_2 = new JoystickButton(super.operatorJoy, 2);
        startShootingOld_3 = new JoystickButton(super.operatorJoy, 3);
        trenchShot_7 = new JoystickButton(super.operatorJoy, 7);
        autolineShot_9 = new JoystickButton(super.operatorJoy, 9);
        stow_10 = new JoystickButton(super.operatorJoy, 10);
        wallShot_11 = new JoystickButton(super.operatorJoy, 11);
        autoDistance_12 = new JoystickButton(super.operatorJoy, 12);
        hoodAngle_5 = new JoystickButton(super.operatorJoy, 5);
        outtake_6 = new JoystickButton(super.operatorJoy, 6);
        togglePipeline_4 = new JoystickButton(super.operatorJoy, 4);
        rendezvousShot_8 = new JoystickButton(super.operatorJoy, 8);
        outtakeBrushes_8 = new JoystickButton(super.operatorJoy, 8);

        shiftLow_A = new JoystickButton(super.driverController, BUTTON_A);
        shiftHigh_B = new JoystickButton(super.driverController, BUTTON_B);
        climbReady_X = new JoystickButton(super.driverController, BUTTON_X);
        climbLift_Y = new JoystickButton(super.driverController, BUTTON_Y);
        turnToAngle_LB = new JoystickButton(super.driverController, BUTTON_LB);
        stow_RB = new JoystickButton(super.driverController, BUTTON_RB);
        resetEncoder_BA = new JoystickButton(super.driverController, BUTTON_BACK);

        intake_RT = new Button() {
            @Override
            public boolean get() {
                return getTrigger(Hand.kRight);
            }
        };
        outtake_LT = new Button() {
            @Override
            public boolean get() {
                return getTrigger(Hand.kLeft);
            }
        };

        intake_1.whenPressed(new IntakeBalls());
        startShooting_2.whileHeld(new ShootBall());
        startShootingOld_3.whileHeld(new ShootBallTemp());
        togglePipeline_4.whenPressed(new InstantCommand(() -> Robot.limelight.togglePipeline()));
        hoodAngle_5.whenPressed(new SetAngle());
        outtake_6.whenPressed(new SetMotorPower(Robot.intakeRoll, -0.75).alongWith(
                new InstantCommand(() -> Robot.hopper.setPowerWithoutTop(-0.4, -0.8)),
                new SetMotorPower(Robot.index, -0.33), new InstantCommand(() -> Robot.hopper.setTopHopperPower(0.41))));
        trenchShot_7.whenPressed(new TrenchShot().alongWith(new InstantCommand(() -> Robot.hood.setAngleMotionMagic(Robot.hood.storedAngle))));
        rendezvousShot_8.whenPressed(new RendezvousShot().alongWith(new InstantCommand(() -> Robot.hood.setAngleMotionMagic(Robot.hood.storedAngle))));
        outtakeBrushes_8.whenHeld(new InstantCommand(() -> Robot.hopper.setTopHopperPower(-0.41)));
        autolineShot_9.whenPressed(new AutolineShot().alongWith(new InstantCommand(() -> Robot.hood.setAngleMotionMagic(Robot.hood.storedAngle))));
        stow_10.whenPressed(new Stow());
        wallShot_11.whenPressed(new WallShot().alongWith(new InstantCommand(() -> Robot.hood.setAngleMotionMagic(Robot.hood.storedAngle))));
        autoDistance_12.whenPressed(new DistanceToAngle());

        outtake_LT.whenPressed(new SetMotorPower(Robot.intakeRoll, -0.75).alongWith(
                new InstantCommand(() -> Robot.hopper.setPowerWithoutTop(-0.4, -0.8)),
                new SetMotorPower(Robot.index, -0.33), new InstantCommand(() -> Robot.hopper.setTopHopperPower(0.41))));
        stow_RB.whenPressed(new Stow());
        shiftHigh_B.whenPressed(new ShiftHigh(Robot.drive));
        shiftLow_A.whenPressed(new ShiftLow(Robot.drive));
        turnToAngle_LB.whileHeld(new TurnToAngleLime(VisionConstants.kRotP_lime)); // .009 before
        intake_RT.whenPressed(new IntakeBalls());
        climbReady_X.whenPressed(new ClimberReady());
        climbLift_Y.whileHeld(new ClimberLift());
        resetEncoder_BA.whenPressed(new ResetDriveEncoders(Robot.drive));
        
        SmartDashboard.putData("Reset indexer",
                new InstantCommand(() -> Robot.index.indexerState = IndexerState.EMPTY));
        SmartDashboard.putData("cLIMBER UP", new ClimberReady());
        SmartDashboard.putData("Climber Lift", new ClimberLift());
        SmartDashboard.putData("Full Send", new DriveStraightContinuous(Robot.drive, 200000, 0.125));
        SmartDashboard.putData("FJFJFJFJFJFJFJFJFJF", new GalacticReal(Robot.drive));
        SmartDashboard.putData("PowerPort", new PowerPort(Robot.drive));
        SmartDashboard.putData("REAL TRAP",
                new InstantCommand(() -> Robot.drive.setPositionMotionMagic(200000, 200000, 2000, 1500)));
        SmartDashboard.putData("trapezoid", new Trapezoid(10, Robot.drive));
        SmartDashboard.putData("Slalom Drive", new Slalom(Robot.drive));
        SmartDashboard.putData("Ramp Test", new DriveCharacterizationTest(Robot.drive, 0.25));
        SmartDashboard.putData("Bounce Drive", new Bounce(Robot.drive));
        SmartDashboard.putData("Bounce Backwards Drive", new BounceBackwards(Robot.drive));
        SmartDashboard.putData("Lightspeed Drive", new Lightspeed(Robot.drive));
        SmartDashboard.putData("Barrel Racing Drive", new BarrelRacing(Robot.drive));
        SmartDashboard.putData("Galactic Path A Red Drive", new GalacticPathARed(Robot.drive));
        SmartDashboard.putData("Galactic Path A Blue Drive", new GalacticPathABlue(Robot.drive));
        SmartDashboard.putData("Galactic Path B Red Drive", new GalacticPathBRed(Robot.drive));
        SmartDashboard.putData("Galactic Path B Blue Drive", new GalacticPathBBlue(Robot.drive));
        SmartDashboard.putData("Test Ramsete", new TestRamsete(Robot.drive));
        SmartDashboard.putNumber("Left Voltage", Robot.drive.getLeftOutputVoltage());
        SmartDashboard.putData("Config Slalom Heading",
                new InstantCommand(() -> Robot.drive.setPose(new Pose2d(0.762, -0.762, new Rotation2d(Math.PI / 2)))));
        SmartDashboard.putData("Config Galactic Heading",
                new InstantCommand(() -> Robot.drive.setPose(new Pose2d(2.286, -0.762, new Rotation2d(Math.PI / 2)))));
        SmartDashboard.putData("4VShooter", new SetMotorPower(Robot.shooter, 0.33));
        SmartDashboard.putData("SetHoodFF lime -> deg", new DistanceToAngle());
        SmartDashboard.putData("ResetHoodEncoder", new ResetSingleMotorEncoder(Robot.hood));
        SmartDashboard.putData("Reset Encoders", new ResetDriveEncoders(Robot.drive));
        SmartDashboard.putData("Reset XY", new InstantCommand(() -> Robot.drive.resetXY()));
        SmartDashboard.putData("Reset Gyro", new ResetGyro(Robot.drive));
        SmartDashboard.putData("Test Ramsete", new TestRamsete(Robot.drive));
        SmartDashboard.putData("Test Ramsete Turn", new TestRamseteTurn(Robot.drive));
        SmartDashboard.putData("InGameResetEncoders", new InGameResetHood());
    }

    public boolean getRawButton(int n) {
        return driverController.getRawButton(n);
    }

    public double getTriggerAxis(Hand hand) {
        return super.driverController.getTriggerAxis(hand);
    }

    public void setRumble(RumbleType rumbleType, double value) {
        super.driverController.setRumble(rumbleType, value);
    }

    public boolean getTrigger(Hand hand) {
        return getTriggerAxis(hand) >= m_triggerThreshold;
    }
}