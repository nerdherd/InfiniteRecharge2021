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
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class XboxOI extends XboxDriverOI {
    
    public JoystickButton shiftLow, shiftHigh, intake, stow, outtake, outtakeBrushes; // driver
    public JoystickButton startShooting, climbReady, climbLift, wallShot, autolineShot, rendezvousShot, trenchShot; // operator
    
    public XboxOI() {
        super();

        shiftLow = new JoystickButton(super.driverController, 1);

        shiftLow.whenPressed(new InstantCommand(()-> System.out.println("Button 6 pressed!")));
    }

}
