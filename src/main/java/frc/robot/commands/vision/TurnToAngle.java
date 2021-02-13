/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;
import frc.robot.constants.VisionConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnToAngle extends CommandBase {

    private double m_rotP;

    public TurnToAngle(double kRotP) {
        addRequirements(Robot.drive);
        addRequirements(Robot.jevois);


        m_rotP = kRotP;
    }

    @Override

    public void initialize() {
        Robot.jevois.enableStream();
        SmartDashboard.putString("Current Command", "TurnToAngle");
    }

    @Override
    public void execute() {
        double getAngularTargetError = -Robot.jevois.getAngleToTurn();
        double robotAngle = (360 - Robot.drive.getRawYaw()) % 360;
        double power = -m_rotP * getAngularTargetError;
        if (!(Math.abs(getAngularTargetError) < VisionConstants.kDriveRotationDeadband)) {
            Robot.drive.setPowerFeedforward(-power, power);
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