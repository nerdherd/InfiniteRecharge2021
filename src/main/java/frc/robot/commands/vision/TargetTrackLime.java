package frc.robot.commands.vision;

import frc.robot.Robot;
import frc.robot.constants.VisionConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TargetTrackLime extends CommandBase {

    private double m_rotP;

    public TargetTrackLime(double kRotP) {
        addRequirements(Robot.drive);
        addRequirements(Robot.limelight);

        m_rotP = kRotP;
    }

    @Override

    public void initialize() {
        SmartDashboard.putString("Current Command", "LiveTargetTrack");
    }

    @Override
    public void execute() {
        double getAngularTargetError = -Robot.limelight.getXOffsetFromTarget();
        double robotAngle = (360 - Robot.drive.getRawYaw()) % 360;
        double power = -m_rotP * getAngularTargetError;
        if (!(Math.abs(getAngularTargetError) < VisionConstants.kDriveRotationDeadband)) {
            Robot.drive.setPowerFeedforward(power + -Robot.oi.getDriveJoyRightY(),
                    -power + -Robot.oi.getDriveJoyRightY());
        } else {
            Robot.drive.setPowerFeedforward(-Robot.oi.getDriveJoyRightY(), -Robot.oi.getDriveJoyRightY());
        }

        SmartDashboard.putNumber("Left Power", power);
        SmartDashboard.putNumber("Right Power", -power);
        SmartDashboard.putNumber("Robot Angle", robotAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.drive.setPowerZero();
    }

}