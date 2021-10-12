package frc.robot;

import java.util.function.BooleanSupplier;

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

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.climber.ClimberLift;
import frc.robot.commands.climber.ClimberReady;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.other.SetAngle;
import frc.robot.commands.shooting.AutolineShot;
import frc.robot.commands.shooting.RendezvousShot;
import frc.robot.commands.shooting.ShootBall;
import frc.robot.commands.shooting.TrenchShot;
import frc.robot.commands.shooting.WallShot;

public class XboxOI extends XboxDriverOI {
    
    public JoystickButton shiftLow, shiftHigh, intake, outtake, rightEncoder, leftEncoder; // driver
    public Button stow, outtakeBrushes; // driver triggers
    public JoystickButton startShooting, climbReady, climbLift, wallShot, autolineShot, hoodAngle, rendezvousShot, trenchShot; // operator
    
    public double triggerStart = 0.25;

    public XboxOI() {
        super();

        shiftLow = new JoystickButton(super.driverController, XboxController.Button.kA.value);
        shiftHigh = new JoystickButton(super.driverController, XboxController.Button.kB.value);
        intake = new JoystickButton(super.driverController, XboxController.Button.kBumperRight.value);
        outtake = new JoystickButton(super.driverController, XboxController.Button.kBumperLeft.value);
        leftEncoder = new JoystickButton(super.driverController, XboxController.Button.kBack.value);
        rightEncoder = new JoystickButton(super.driverController, XboxController.Button.kStart.value);

        shiftLow.whenPressed(new ShiftLow(Robot.drive));
        shiftHigh.whenPressed(new ShiftHigh(Robot.drive));
        intake.whenPressed(new IntakeBalls());
        outtake.whenPressed(new SetMotorPower(Robot.intakeRoll, -0.75).alongWith(
                new InstantCommand(() -> Robot.hopper.setPowerWithoutTop(-0.4, -0.8)),
                new SetMotorPower(Robot.index, -0.33), new InstantCommand(() -> Robot.hopper.setTopHopperPower(0.41))));
        rightEncoder.whenPressed(Robot.hoodReset);
        leftEncoder.whenPressed(Robot.hoodReset);

        stow = new Button(new BooleanSupplier(){
            public boolean getAsBoolean() {
                return getTrigger(Hand.kRight);
            };
        });
        
        outtakeBrushes = new Button(new BooleanSupplier(){
            public boolean getAsBoolean() {
                return getTrigger(Hand.kLeft);
            }
        });

        startShooting = new JoystickButton(super.operatorJoy, 1);
        hoodAngle = new JoystickButton(super.operatorJoy, 2);
        climbReady = new JoystickButton(super.operatorJoy, 3);
        climbLift = new JoystickButton(super.operatorJoy, 4);
        wallShot = new JoystickButton(super.operatorJoy, 7);
        autolineShot = new JoystickButton(super.operatorJoy, 8);
        rendezvousShot = new JoystickButton(super.operatorJoy, 11);
        trenchShot = new JoystickButton(super.operatorJoy, 12);

        startShooting.whenPressed(new ShootBall());
        hoodAngle.whenPressed(new SetAngle());
        climbReady.whenPressed(new ClimberReady());
        climbLift.whenPressed(new ClimberLift());
        wallShot.whenPressed(new WallShot());
        autolineShot.whenPressed(new AutolineShot());
        rendezvousShot.whenPressed(new RendezvousShot());
        trenchShot.whenPressed(new TrenchShot());
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
        return getTriggerAxis(hand) >= triggerStart;
    }

}
