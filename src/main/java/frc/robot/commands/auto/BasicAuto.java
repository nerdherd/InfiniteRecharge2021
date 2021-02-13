/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import com.nerdherd.lib.drivetrain.characterization.OpenLoopDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.shooting.AutolineShot;
import frc.robot.commands.shooting.ShootBall;
import frc.robot.commands.vision.TurnToAngleLime;

/**
 * Add your docs here.
 */
public class BasicAuto extends SequentialCommandGroup {

    public BasicAuto() {
        addCommands(
            new AutolineShot(), 
            new InstantCommand(() -> Robot.hood.setStoredAngle(), Robot.hood), 
            new ParallelRaceGroup(new WaitCommand(2), new TurnToAngleLime(0.009)),
            new ParallelRaceGroup(new ShootBall(), new WaitCommand(8)),
            new ParallelRaceGroup(new OpenLoopDrive(Robot.drive, -0.2), new WaitCommand(1))
        );
    }

}
