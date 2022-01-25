/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

public class PhotonVision extends CommandBase {

    //Photon Vision constants
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    // PID constants
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    private Robot m_robot;

    public void turnToAnglePhotonVision() {
        double forwardSpeed;
        double rotationSpeed;

        //forwardSpeed = -xboxController.getRightY();

        var result = m_robot.camera.getLatestResult();
    
        SmartDashboard.putBoolean("Has targets:", result.hasTargets());
    
        if (result.hasTargets()) {
            // Calculate angular turn power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        } else {
            // If we have no targets, stay still.
            rotationSpeed = 0;
        }
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Current Command", "TurnToAnglePhotonVision");
    }

    @Override
    public void execute() {
        turnToAnglePhotonVision();
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