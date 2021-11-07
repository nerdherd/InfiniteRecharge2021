/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

// import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.Indexer.IndexerState;

public class IntakeBalls extends CommandBase {

  private double m_startTime;
  private int state, counter;
  /**
   * Creates a new Intake.
   */
  public IntakeBalls() {
    state = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.intake, Robot.intakeRoll);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Robot.hopper.setPower(-0.33, -0.167);
    Robot.hopper.setPower(-0.33, 0.167);
    m_startTime = Timer.getFPGATimestamp();
    if (Robot.index.indexerState == IndexerState.EMPTY) {
      Robot.index.indexerState = IndexerState.WAITING;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.intakeRoll.setPower(0.95);
    Robot.intake.setForwards();
    Robot.shooter.setPower(0.0);

    if (Robot.index.indexerBallDetected()){
      Robot.index.setPower(0.0);
      Robot.hopper.setPowerWithoutTop(0.0, 0.0);
    }else{
      Robot.index.setPower(0.16);
      // Robot.hopper.setPowerWithoutTop(0.4, 0.8);
    }


    // if (Robot.index.indexerHighBallDetected() && !Robot.index.indexerLowBallDetected()){
    //   Robot.index.indexerState = IndexerState.ERROR;
    // }
    // if (Robot.index.indexerState == IndexerState.ERROR){
    //   Robot.hopper.setPower(0.0, 0.0);
    //   Robot.index.setPower(-0.15);
    //   if(Robot.index.indexerLowBallDetected()){
    //     Robot.index.indexerState = IndexerState.WAITING;
    //   }
    // }else if (Robot.index.indexerState == IndexerState.WAITING) {
    //   Robot.hopper.setPower(0.0, 0.0);
    //   Robot.index.setPower(0.0);
    //   if (Robot.index.hopperBallDetected() && !Robot.index.indexerLowBallDetected()) {
    //     Robot.index.indexerState = IndexerState.INTAKING_ONE;
    //   }else if(Robot.index.hopperBallDetected() && Robot.index.indexerLowBallDetected()){
    //     Robot.index.indexerState = IndexerState.INTAKING_TWO;
    //   }
    // } else if (Robot.index.indexerState == IndexerState.INTAKING_ONE) {
    //   Robot.hopper.setPower(0.4, 0.8);
    //   Robot.index.setPower(0.25);
    //   if(Robot.index.indexerLowBallDetected()){
    //     Robot.index.indexerState = IndexerState.INTAKING_TWO;
    //   }
    // } else if (Robot.index.indexerState == IndexerState.INTAKING_TWO) {
    //   Robot.hopper.setPower(0.4, 0.8);
    //   Robot.index.setPower(0.25);
    //   if(Robot.index.indexerHighBallDetected() && Robot.index.indexerLowBallDetected()){
    //     Robot.index.indexerState = IndexerState.FULL;
    //   }
    // } else if (Robot.index.indexerState == IndexerState.FULL) {
    //   Robot.hopper.setPower(0, 0);
    //   Robot.index.setPower(0);
    // }




    // if(Robot.index.ultrasonic.getRangeInches() <= 4) {
    //   Robot.index.setPower(0.25);
    // } else {
    //   Robot.index.setPower(0);

    //   hasBallInIndexer = true;
    // }
    // 0 = intake one 
    // 1 = intake one ready
    // 2 = intake both
    // 3 = full
    // 4 = empty
    // if(Robot.indexer.timeOfFlight1.getRange() <= IndexerConstants.kTimeOfFlight1){
    //   Robot.indexer.intakeDetermine(0);
    // }else if(Robot.indexer.timeOfFlight2.getRange() <= IndexerConstants.kTimeOfFlight2){
    //   Robot.indexer.intakeDetermine(1);
    // }else if(Robot.indexer.timeOfFlight1.getRange() <= IndexerConstants.kTimeOfFlight1 && Robot.indexer.timeOfFlight2.getRange() <= IndexerConstants.kTimeOfFlight2){
    //   Robot.indexer.intakeDetermine(2);
    // }else if(Robot.indexer.timeOfFlight1.getRange() > IndexerConstants.kTimeOfFlight1 && Robot.indexer.timeOfFlight2.getRange() > IndexerConstants.kTimeOfFlight2){
      
    // }
        /*if(Robot.indexer.timeOfFlight1.getRange() <= IndexerConstants.kTimeOfFlight1){
      Robot.index.setPower(0.25);
    }else if(Robot.indexer.timeOfFlight1.getRange() <= IndexerConstants.kTimeOfFlight1 && Robot.indexer.timeOfFlight2.getRange() <= IndexerConstants.kTimeOfFlight2){
      Robot.index.setPower(0.25);
    }else if(Robot.indexer.timeOfFlight2.getRange() <= IndexerConstants.kTimeOfFlight2){
      Robot.index.setPower(0.0);
    }*/
    if (Timer.getFPGATimestamp() - m_startTime > 1.5) {
      Robot.hopper.setTopHopperPower(0.67); 
    }
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
