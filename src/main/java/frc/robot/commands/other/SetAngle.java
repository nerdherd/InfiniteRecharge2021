/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.other;

import com.nerdherd.lib.motor.commands.SetMotorMotionMagic;
import com.nerdherd.lib.motor.commands.SetMotorVelocity;
import com.nerdherd.lib.motor.commands.mechanisms.SetArmAngleMotionMagic;
import com.nerdherd.lib.motor.single.SingleMotorMechanism;
import com.nerdherd.lib.motor.single.mechanisms.SingleMotorArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.HoodConstants;

public class SetAngle extends SetArmAngleMotionMagic {
  /**
   * Creates a new SetAngle.
   */
  public SetAngle() {
    super(Robot.hood, Robot.hood.storedAngle);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  @Override
  public void execute() {
    // TODO Auto-generated method stub
    Robot.hood.setAngleMotionMagic(Robot.hood.storedAngle);
  }

}
