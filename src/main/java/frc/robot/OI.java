/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.nerdherd.lib.drivetrain.auto.DriveStraightContinuous;
import com.nerdherd.lib.drivetrain.auto.ResetDriveEncoders;
import com.nerdherd.lib.drivetrain.auto.ResetGyro;
import com.nerdherd.lib.drivetrain.shifting.ShiftHigh;
import com.nerdherd.lib.drivetrain.shifting.ShiftLow;
import com.nerdherd.lib.motor.commands.ResetSingleMotorEncoder;
import com.nerdherd.lib.motor.commands.SetMotorPower;
import com.nerdherd.lib.motor.single.SingleMotorMechanism;
import com.nerdherd.lib.oi.DefaultOI;

import edu.wpi.first.wpilibj.Sendable;

// import org.graalvm.compiler.lir.aarch64.AArch64Move.StoreConstantOp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.InGameResetHood;
import frc.robot.commands.auto.AutoLineIntoTrench;
import frc.robot.commands.auto.Slalom;
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
 * Add your docs here.
 */
public class OI extends DefaultOI {

    public JoystickButton intake_1, startShooting_2, startShootingOld_3, trenchShot_7, autolineShot_9, stow_10,
            wallShot_11, autoDistance_12, hoodAngle_5, turnToAngle_1L, turnToAngle_1R, resetEncoders_5R,
            resetEncoders_5L, outtake_6, shiftHigh_6L, shiftLow_6R, ploughIntake_2, togglePipeline_4, rendezvousShot_8,
            climbReady_3L, climbLift_4L, outtakeBrushes_8;
    // trench and auto manual shooting position for shooter
    // climberExtend_5, climberRetract_6

    public OI() {
        super();
        ploughIntake_2 = new JoystickButton(super.driveJoyLeft, 2); // Check in with Drivers
        shiftHigh_6L = new JoystickButton(super.driveJoyLeft, 6);
        shiftLow_6R = new JoystickButton(super.driveJoyRight, 6);
        turnToAngle_1L = new JoystickButton(super.driveJoyLeft, 1);
        turnToAngle_1R = new JoystickButton(super.driveJoyRight, 1);
        resetEncoders_5L = new JoystickButton(super.driveJoyLeft, 5);
        resetEncoders_5R = new JoystickButton(super.driveJoyRight, 5);
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
        climbReady_3L = new JoystickButton(super.driveJoyLeft, 3);
        climbLift_4L = new JoystickButton(super.driveJoyLeft, 4);

        ploughIntake_2.whenPressed(new SetMotorPower(Robot.intakeRoll, -0.75));
        shiftHigh_6L.whenPressed(new ShiftHigh(Robot.drive));
        shiftLow_6R.whenPressed(new ShiftLow(Robot.drive));
        turnToAngle_1L.whileHeld(new TurnToAngleLime(VisionConstants.kRotP_lime)); // .009 before
        turnToAngle_1R.whileHeld(new TurnToAngleLime(VisionConstants.kRotP_lime));
        intake_1.whenPressed(new IntakeBalls());
        startShooting_2.whileHeld(new ShootBall());
        startShootingOld_3.whileHeld(new ShootBallTemp());
        trenchShot_7.whenPressed(new TrenchShot());
        autolineShot_9.whenPressed(new AutolineShot());
        rendezvousShot_8.whenPressed(new RendezvousShot());
        outtakeBrushes_8.whenHeld(new InstantCommand(()-> Robot.hopper.setTopHopperPower(-0.41)));
        stow_10.whenPressed(new Stow());
        wallShot_11.whenPressed(new WallShot());
        hoodAngle_5.whenPressed(new SetAngle());
        outtake_6.whenPressed(new SetMotorPower(Robot.intakeRoll, -0.75).alongWith(
                new InstantCommand(() -> Robot.hopper.setPowerWithoutTop(-0.4, -0.8)),
                new SetMotorPower(Robot.index, -0.33),
                new InstantCommand(() -> Robot.hopper.setTopHopperPower(0.41))));
        resetEncoders_5L.whenPressed(Robot.hoodReset);
        resetEncoders_5R.whenPressed(Robot.hoodReset);
        autoDistance_12.whenPressed(new DistanceToAngle());
        togglePipeline_4.whenPressed(new InstantCommand(() -> Robot.limelight.togglePipeline()));
        climbReady_3L.whenPressed(new ClimberReady());
        climbLift_4L.whileHeld(new ClimberLift());

        SmartDashboard.putData("Reset indexer",
                new InstantCommand(() -> Robot.index.indexerState = IndexerState.EMPTY));
        SmartDashboard.putData("cLIMBER UP", new ClimberReady());
        SmartDashboard.putData("Climber Lift", new ClimberLift());
        // SmartDashboard.putData("Ramsete TEST", new BasicRamseteForward(Robot.drive));
        // SmartDashboard.putData("Ramsete Trench", new
        // AutoLineIntoTrench(Robot.drive));
        // SmartDashboard.putData("Reset Left Climber", new
        // ResetSingleMotorEncoder(Robot.climber.followerFalcon));
        // shoot_2.whileHeld(new ShootBall());
        // infiniteRecharge_3.whenPressed(new InfiniteRecharge());
        // stow_4.whenPressed(new Stow());
        // // climb_5.whenPressed(new ClimberClimb());
        // panelRetract_7.whenPressed(new PanelRetract());
        // panelExtend_8.whenPressed(new PanelExtend());
        // panelRotation_9.whenPressed(new PanelRotate());
        // wallShot_11.whenPressed(new WallShot());
        // SmartDashboard.putData("Run Forward", new DriveStraightContinuous(Robot.drive, 2000, 0.5));
        SmartDashboard.putData("Slalom Drive", new Slalom(Robot.drive));
        // SmartDashboard.putData("TurnToAngle", new TurnToAngleLime(0.1));

        // SmartDashboard.putData("1VIndex", new SetMotorPower(Robot.index, 0.0833));
        // SmartDashboard.putData("2VIndex", new SetMotorPower(Robot.index, 0.166));
        // SmartDashboard.putData("3VIndex", new SetMotorPower(Robot.index, 0.25));

        // SmartDashboard.putData("TogglePipeline", new InstantCommand(() -> Robot.limelight.togglePipeline()));
        // SmartDashboard.putData("Auto into trench", new AutoLineIntoTrench(Robot.drive));
        // SmartDashboard.putData("Feeder45", new SetDualMotorPower(Robot.feeder, 0.45,
        // 0.45));
        // SmartDashboard.putData("Feeder25", new SetDualMotorPower(Robot.feeder, 0.25,
        // 0.25));
        // SmartDashboard.putData("Feeder35", new SetDualMotorPower(Robot.feeder, 0.35,
        // 0.35));

        // SmartDashboard.putData("backFeeder", new SetDualMotorPower(Robot.feeder,
        // -0.25, -0.25));
        // SmartDashboard.putData("Feeder25", new SetDualMotorPower(Robot.feeder, 0.25,
        // 0.25));

        // SmartDashboard.putData("Feeder50", new SetDualMotorPower(Robot.feeder, 0.5,
        // 0.5));
        // SmartDashboard.putData("Feeder55", new SetDualMotorPower(Robot.feeder, 0.55,
        // 0.55));
        // SmartDashboard.putData("Feeder60", new SetDualMotorPower(Robot.feeder, 0.60,
        // 0.60));
        // SmartDashboard.putData("Feeder65", new SetDualMotorPower(Robot.feeder, 0.65,
        // 0.65));
        // SmartDashboard.putData("Feeder70", new SetDualMotorPower(Robot.feeder, 0.70,
        // 0.70));
        // SmartDashboard.putData("Feeder100", new SetDualMotorPower(Robot.feeder, 1.0,
        // 0.0));
        // SmartDashboard.putData("FeederOff", new SetDualMotorPower(Robot.feeder, 0.0,
        // 0.0));

        // SmartDashboard.putData("12V ", new SetMotorPower(Robot.shooter, 1));
        // SmartDashboard.putData("75%Volts", new SetMotorPower(Robot.shooter, 0.75));
        // SmartDashboard.putData("70%Volts", new SetMotorPower(Robot.shooter, 0.7));
        // SmartDashboard.putData("65%Volts", new SetMotorPower(Robot.shooter, 0.65));
        // SmartDashboard.putData("65%Volts", new SetMotorPower(Robot.shooter, 0.65));
        // SmartDashboard.putData("45%Volts", new SetMotorPower(Robot.index, 0.45));
        // SmartDashboard.putData("30%Volts", new SetMotorPower(Robot.shooter, 0.3));
        // SmartDashboard.putData("ShooterOff", new SetMotorPower(Robot.shooter, 0.0));

        SmartDashboard.putData("4VShooter", new SetMotorPower(Robot.shooter, 0.33));
        // SmartDashboard.putData("4.5VShooter", new SetMotorPower(Robot.shooter,
        // (4.5/12.0)));
        // SmartDashboard.putData("4.7VShooter", new SetMotorPower(Robot.shooter,
        // (4.7/12.0)));
        // SmartDashboard.putData("4.6VShooter", new SetMotorPower(Robot.shooter,
        // (4.6/12.0)));
        // SmartDashboard.putData("1VShooter", new SetMotorPower(Robot.shooter,
        // 0.083333333));
        // SmartDashboard.putData("3VShooter", new SetMotorPower(Robot.shooter, 0.25));
        // SmartDashboard.putData("2VShooter", new SetMotorPower(Robot.shooter, 0.167));
        // SmartDashboard.putData("5Vshooter", new SetMotorPower(Robot.shooter, 0.416));
        // SmartDashboard.putData("5.5VShooter", new SetMotorPower(Robot.shooter,
        // 0.45833));
        // SmartDashboard.putData("5.7VShooter", new SetMotorPower(Robot.shooter,
        // 0.475));

        // SmartDashboard.putData("6VShooter", new SetMotorPower(Robot.shooter, 0.5));
        // SmartDashboard.putData("7VShooter", new SetMotorPower(Robot.shooter,
        // 0.58333333));
        // SmartDashboard.putData("8VShooter", new SetMotorPower(Robot.shooter,
        // 0.66666));
        // SmartDashboard.putData("9VShooter", new SetMotorPower(Robot.shooter, 0.75));
        // SmartDashboard.putData("10VShooter", new SetMotorPower(Robot.shooter,
        // 0.83333333));
        // SmartDashboard.putData("11VShooter", new SetMotorPower(Robot.shooter,
        // 0.9166));
        // SmartDashboard.putData("12VShooter", new SetMotorPower(Robot.shooter, 1.0));

        // SmartDashboard.putData("15000", new SetMotorVelocity(Robot.shooter, 15000));
        // SmartDashboard.putData("15100", new SetMotorVelocity(Robot.shooter, 15100));
        // SmartDashboard.putData("15200", new SetMotorVelocity(Robot.shooter, 15200));
        // SmartDashboard.putData("15300", new SetMotorVelocity(Robot.shooter, 15300));
        // SmartDashboard.putData("15400", new SetMotorVelocity(Robot.shooter, 15400));
        // SmartDashboard.putData("15500", new SetMotorVelocity(Robot.shooter, 15500));
        // SmartDashboard.putData("15600", new SetMotorVelocity(Robot.shooter, 15600));
        // SmartDashboard.putData("15700", new SetMotorVelocity(Robot.shooter, 15700));
        // SmartDashboard.putData("15800", new SetMotorVelocity(Robot.shooter, 15800));
        // SmartDashboard.putData("15900", new SetMotorVelocity(Robot.shooter, 15900));

        // // SmartDashboard.putData("14000", new SetMotorVelocity(Robot.shooter,
        // 14000));
        // // SmartDashboard.putData("13000", new SetMotorVelocity(Robot.shooter,
        // 13000));
        // SmartDashboard.putData("16000", new SetMotorVelocity(Robot.shooter, 16000));
        // SmartDashboard.putData("16100", new SetMotorVelocity(Robot.shooter, 16100));
        // SmartDashboard.putData("16200", new SetMotorVelocity(Robot.shooter, 16200));
        // SmartDashboard.putData("16300", new SetMotorVelocity(Robot.shooter, 16300));
        // SmartDashboard.putData("16400", new SetMotorVelocity(Robot.shooter, 16400));
        // SmartDashboard.putData("16500", new SetMotorVelocity(Robot.shooter, 16500));
        // SmartDashboard.putData("16600", new SetMotorVelocity(Robot.shooter, 16600));
        // SmartDashboard.putData("16700", new SetMotorVelocity(Robot.shooter, 16700));
        // SmartDashboard.putData("16800", new SetMotorVelocity(Robot.shooter, 16800));

        // SmartDashboard.putData("16500", new SetMotorVelocity(Robot.shooter, 16500));
        // SmartDashboard.putData("17000", new SetMotorVelocity(Robot.shooter, 17000));
        // SmartDashboard.putData("17500", new SetMotorVelocity(Robot.shooter, 17500));
        // SmartDashboard.putData("18000", new SetMotorVelocity(Robot.shooter, 18000));

        // for (int i = 16200; i < 16800; i += 100) {
        // SmartDashboard.putData(String.valueOf(i), new SetMotorVelocity(Robot.shooter,
        // i));
        // }

        // SmartDashboard.putData("18050", new SetMotorVelocity(Robot.shooter, 18050));
        // SmartDashboard.putData("18100", new SetMotorVelocity(Robot.shooter, 18100));
        // SmartDashboard.putData("18150", new SetMotorVelocity(Robot.shooter, 18150));
        // SmartDashboard.putData("18200", new SetMotorVelocity(Robot.shooter, 18200));
        // SmartDashboard.putData("18250", new SetMotorVelocity(Robot.shooter, 18250));
        // // This is a speshul speed
        // SmartDashboard.putData("18300", new SetMotorVelocity(Robot.shooter, 18300));
        // SmartDashboard.putData("18350", new SetMotorVelocity(Robot.shooter, 18350));
        // SmartDashboard.putData("18400", new SetMotorVelocity(Robot.shooter, 18400));
        // SmartDashboard.putData("18450", new SetMotorVelocity(Robot.shooter, 18450));

        // SmartDashboard.putData("18500", new SetMotorVelocity(Robot.shooter, 18500));
        // SmartDashboard.putData("19000", new SetMotorVelocity(Robot.shooter, 19000));
        // SmartDashboard.putData("19500", new SetMotorVelocity(Robot.shooter, 19500));
        // SmartDashboard.putData("20000", new SetMotorVelocity(Robot.shooter, 20000));

        // SmartDashboard.putData("4VShooterVel", new SetMotorVelocity(Robot.shooter,
        // 6750));
        // SmartDashboard.putData("4.5VShooterVel", new SetMotorVelocity(Robot.shooter,
        // 8437));
        // SmartDashboard.putData("4VIndex", new SetMotorPower(Robot.index, 0.));

        // SmartDashboard.putData("1VSingleFalcon", new SetMotorPower(Robot.falcon,
        // 0.083333333));

        // SmartDashboard.putData("ShooterRamp", new MotorVoltageRamping(Robot.shooter,
        // 0.25));
        // SmartDashboard.putData("HoodRamp", new MotorVoltageRamping(Robot.hood,
        // 0.125));
        // SmartDashboard.putData("SetHoodPower", new SetMotorPower(Robot.hood, 0.15));
        // SmartDashboard.putData("-SetHoodPower", new SetMotorPower(Robot.hood, 0.15));
        // SmartDashboard.putData("SetHoodFF -10 deg", new
        // SetArmAngleMotionMagic(Robot.hood, -10));
        // SmartDashboard.putData("SetHoodFF -5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, -5));
        // SmartDashboard.putData("SetHoodFF 0 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 0));
        // SmartDashboard.putData("SetHoodFF 5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 5));
        // SmartDashboard.putData("SetHoodFF 10 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 10));
        // SmartDashboard.putData("SetHoodFF 15 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 15));
        // SmartDashboard.putData("SetHoodFF 20 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 20));
        // SmartDashboard.putData("SetHoodFF 7 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 7));
        // SmartDashboard.putData("SetHoodFF 8 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 8));
        // SmartDashboard.putData("SetHoodFF 9 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 9));

        // SmartDashboard.putData("SetHoodFF 10 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 10));
        // SmartDashboard.putData("SetHoodFF 10.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 10.5));
        // SmartDashboard.putData("SetHoodFF 11 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 11));
        // SmartDashboard.putData("SetHoodFF 11.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 11.5));
        // SmartDashboard.putData("SetHoodFF 12 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 12));
        // SmartDashboard.putData("SetHoodFF 12.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 12.5));
        // SmartDashboard.putData("SetHoodFF 13 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 13));
        // SmartDashboard.putData("SetHoodFF 13.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 13.5));

        // SmartDashboard.putData("SetHoodFF 14 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 14));
        // SmartDashboard.putData("SetHoodFF 14.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 14.5));
        // SmartDashboard.putData("SetHoodFF 15 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 15));
        // SmartDashboard.putData("SetHoodFF 15.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 15.5));
        // SmartDashboard.putData("SetHoodFF 16 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 16));
        // SmartDashboard.putData("SetHoodFF 16.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 16.5));
        // SmartDashboard.putData("SetHoodFF 17 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 17));
        // SmartDashboard.putData("SetHoodFF 17.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 17.5));
        // SmartDashboard.putData("SetHoodFF 18 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 18));
        // SmartDashboard.putData("SetHoodFF 18.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 18.5));
        // SmartDashboard.putData("SetHoodFF 19 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 19));

        SmartDashboard.putData("SetHoodFF lime -> deg", new DistanceToAngle());
        // SmartDashboard.putData("SetHoodFF jevois -> deg", new
        // SetArmAngleMotionMagic(Robot.hood,
        // Robot.hood.distToAngle(Robot.jevois.getOldDistance())));
        // SmartDashboard.putData("SetHoodFF 20 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 20));
        // SmartDashboard.putData("SetHoodFF 20.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 20.5));
        // SmartDashboard.putData("SetHoodFF 21 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 21));
        // SmartDashboard.putData("SetHoodFF 21.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 21.5));
        // SmartDashboard.putData("SetHoodFF 22 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 22));
        // SmartDashboard.putData("SetHoodFF 22.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 22.5));
        // SmartDashboard.putData("SetHoodFF 23 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 23));
        // SmartDashboard.putData("SetHoodFF 23.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 23.5));
        // SmartDashboard.putData("SetHoodFF 24 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 24));
        // SmartDashboard.putData("SetHoodFF 24.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 24.5));
        // SmartDashboard.putData("SetHoodFF 25 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 25));
        // SmartDashboard.putData("SetHoodFF 26 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 26));
        // SmartDashboard.putData("SetHoodFF 26.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 26.5));
        // SmartDashboard.putData("SetHoodFF 27 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 27));
        // SmartDashboard.putData("SetHoodFF 28 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 28));
        // SmartDashboard.putData("SetHoodFF 29 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 29));
        // SmartDashboard.putData("SetHoodFF 29.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 29.5));
        // SmartDashboard.putData("SetHoodFF 30 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 30));
        // SmartDashboard.putData("SetHoodFF 30.5 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 30.5));
        // SmartDashboard.putData("SetHoodFF 30.75 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 30.75));
        // SmartDashboard.putData("SetHoodFF 31 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 31));
        // SmartDashboard.putData("SetHoodFF 32 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 32));
        // SmartDashboard.putData("SetHoodFF 33 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 33));
        // SmartDashboard.putData("SetHoodFF 34 deg", new
        // SetArmAngleMotionMagic(Robot.hood, 34));
        // SmartDashboard.putData("Set hood angle 10", new InstantCommand(() ->
        // Robot.hood.motor.setPositionPID(1000)));
        // SmartDashboard.putData("Tell me what you are", new InstantCommand(() ->
        // System.out.println(Robot.hood.motor.getClass().toString())));
        SmartDashboard.putData("ResetHoodEncoder", new ResetSingleMotorEncoder(Robot.hood));
        SmartDashboard.putData("Reset Encoders", new ResetDriveEncoders(Robot.drive));
        SmartDashboard.putData("Reset XY", new InstantCommand(() -> Robot.drive.resetXY()));
        SmartDashboard.putData("Reset Gyro", new ResetGyro(Robot.drive));
        // SmartDashboard.putData("Steal Two", new StealTwoEnemyTrench(Robot.drive));

        // SmartDashboard.putNumber("Right Encoder",
        // Robot.drive.getRightMasterPosition());
        // SmartDashboard.putNumber("Left Encoder",
        // Robot.drive.getLeftMasterPosition());

        // SmartDashboard.putData("3VHopper", new SetDualMotorPower(Robot.hopper, 0.4,
        // 0.6));
        // SmartDashboard.putData("-3VHopper", new SetDualMotorPower(Robot.hopper,
        // -0.45, -0.25));
        // SmartDashboard.putData("StopHopper", new SetDualMotorPower(Robot.hopper, 0.0,
        // 0.0));
        // SmartDashboard.putData("StopShooter", new SetMotorPower(Robot.shooter, 0.0));
        // SmartDashboard.putData("3VIntakeRollers", new SetMotorPower(Robot.intakeRoll,
        // 0.25));
        // SmartDashboard.putData("6VIntakeRollers", new SetMotorPower(Robot.intakeRoll,
        // 0.5));
        // SmartDashboard.putData("Neg3VIntakeRollers", new
        // SetMotorPower(Robot.intakeRoll, -0.5));
        // SmartDashboard.putData("Neg6VIntakeRollers", new
        // SetMotorPower(Robot.intakeRoll, -0.6
        // ));

        // SmartDashboard.putData("StopIntakeRollers", new
        // SetMotorPower(Robot.intakeRoll, 0.0));
        // SmartDashboard.putData("6VIndex", new SetMotorPower(Robot.index, 0.5));
        // SmartDashboard.putData("12VIndex", new SetMotorPower(Robot.index, 1));
        // SmartDashboard.putData("9VIndex", new SetMotorPower(Robot.index, 0.75));
        // SmartDashboard.putData("Neg6VIndex", new SetMotorPower(Robot.index, -0.5));
        // SmartDashboard.putData("StopIndex", new SetMotorPower(Robot.index, 0.0));
        // SmartDashboard.putData("ExtendIntake", new ExtendPiston(Robot.intake));
        // SmartDashboard.putData("RetractIntake", new RetractPiston(Robot.intake));

        // SmartDashboard.putData("ShiftUp", new ShiftHigh(Robot.drive));
        // SmartDashboard.putData("ShiftDown", new ShiftLow(Robot.drive));

        // SmartDashboard.putData("ShooterRamp", new MotorVoltageRamping(Robot.shooter,
        // 0.25/12));
        // SmartDashboard.putNumber("Encoder Rate Left",
        // Robot.drive.getLeftMasterVelocity());
        // SmartDashboard.putNumber("Encoder Rate Right",
        // Robot.drive.getLeftMasterVelocity());
        // // SmartDashboard.putData("Shoot", new ShootBallTemp());
        // // SmartDashboard.putData("ShootStop", new ShootBallTempStop());
        // SmartDashboard.putData("InfiniteRecharge", new InfiniteRecharge());
        // SmartDashboard.putData("drive voltage ramp", new
        // DriveCharacterizationTest(Robot.drive, 0.25));

        // SmartDashboard.putData("TimeOfFlightTest", new TimeOfFlightStop());
        // SmartDashboard.putData("IndexerRamp", new MotorVoltageRamping(Robot.indexer,
        // 0.25));
        // SmartDashboard.putData("6371Indexer", new SetMotorVelocity(Robot.indexer,
        // 6371));

        SmartDashboard.putData("InGameResetEncoders", new InGameResetHood());

    }

	private Sendable SetMotorPower(SingleMotorMechanism index, double d) {
		return null;
	}
}