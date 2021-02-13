/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.ShooterConstants;

public class WallShot extends CommandBase {
  /**
   * Creates a new WallShot.
   */
  public WallShot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.hood, Robot.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Robot.hood.storeAngle(HoodConstants.kWallShotAngle);
    Robot.shooter.setVelocity(ShooterConstants.kWallShotVelocity);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
