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
import frc.robot.commands.intake.Stow;
import frc.robot.commands.other.SetAngle;
import frc.robot.commands.shooting.AutolineShot;
import frc.robot.commands.shooting.RendezvousShot;
import frc.robot.commands.shooting.ShootBall;
import frc.robot.commands.shooting.TrenchShot;
import frc.robot.commands.shooting.WallShot;

public class XboxOI extends XboxDriverOI {
    
    private JoystickButton m_shiftLow, m_shiftHigh, m_intake, m_outtake, m_rightEncoder, m_leftEncoder; // driver
    private Button m_stow, m_outtakeBrushes; // driver triggers
    private JoystickButton m_startShooting, m_climbReady, m_climbLift, m_wallShot, m_autolineShot, m_hoodAngle, m_rendezvousShot, m_trenchShot; // operator
    
    private double m_triggerThreshold = 0.25;

    public XboxOI() {
        super();

        m_shiftLow = new JoystickButton(super.driverController, XboxController.Button.kA.value);
        m_shiftHigh = new JoystickButton(super.driverController, XboxController.Button.kB.value);
        m_intake = new JoystickButton(super.driverController, XboxController.Button.kBumperRight.value);
        m_outtake = new JoystickButton(super.driverController, XboxController.Button.kBumperLeft.value);
        m_leftEncoder = new JoystickButton(super.driverController, XboxController.Button.kBack.value);
        m_rightEncoder = new JoystickButton(super.driverController, XboxController.Button.kStart.value);

        // create an analog trigger for L and R
        m_stow = new Button(new BooleanSupplier(){
            public boolean getAsBoolean() {
                return getTrigger(Hand.kRight);
            };
        });
        m_outtakeBrushes = new Button(new BooleanSupplier(){
            public boolean getAsBoolean() {
                return getTrigger(Hand.kLeft);
            }
        });

        m_shiftLow.whenPressed(new ShiftLow(Robot.drive));
        m_shiftHigh.whenPressed(new ShiftHigh(Robot.drive));
        m_intake.whenPressed(new IntakeBalls());
        m_outtake.whenPressed(new SetMotorPower(Robot.intakeRoll, -0.75).alongWith(
                new InstantCommand(() -> Robot.hopper.setPowerWithoutTop(-0.4, -0.8)),
                new SetMotorPower(Robot.index, -0.33), new InstantCommand(() -> Robot.hopper.setTopHopperPower(0.41))));
        m_rightEncoder.whenPressed(Robot.hoodReset);
        m_leftEncoder.whenPressed(Robot.hoodReset);
        m_stow.whenPressed(new Stow());
        m_outtakeBrushes.whenHeld(new InstantCommand(() -> Robot.hopper.setTopHopperPower(-0.41)));

        m_startShooting = new JoystickButton(super.operatorJoy, 1);
        m_hoodAngle = new JoystickButton(super.operatorJoy, 2);
        m_climbReady = new JoystickButton(super.operatorJoy, 3);
        m_climbLift = new JoystickButton(super.operatorJoy, 4);
        m_wallShot = new JoystickButton(super.operatorJoy, 7);
        m_autolineShot = new JoystickButton(super.operatorJoy, 8);
        m_rendezvousShot = new JoystickButton(super.operatorJoy, 11);
        m_trenchShot = new JoystickButton(super.operatorJoy, 12);

        m_startShooting.whileHeld(new ShootBall());
        m_hoodAngle.whenPressed(new SetAngle());
        m_climbReady.whenPressed(new ClimberReady());
        m_climbLift.whenPressed(new ClimberLift());
        m_wallShot.whenPressed(new WallShot());
        m_autolineShot.whenPressed(new AutolineShot());
        m_rendezvousShot.whenPressed(new RendezvousShot());
        m_trenchShot.whenPressed(new TrenchShot());
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
