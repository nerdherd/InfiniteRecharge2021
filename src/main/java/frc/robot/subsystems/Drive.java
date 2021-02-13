/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.nerdherd.lib.drivetrain.experimental.ShiftingDrivetrain;
import com.nerdherd.lib.motor.motorcontrollers.CANMotorController;
import com.nerdherd.lib.motor.motorcontrollers.NerdyFalcon;
import com.nerdherd.lib.pneumatics.Piston;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.constants.DriveConstants;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends ShiftingDrivetrain {
  /**
   * Creates a new Drive.
   */
  public Drive() {
    super(new NerdyFalcon(RobotMap.kLeftMasterTalonID),
     new NerdyFalcon(RobotMap.kRightMasterTalonID),
    new CANMotorController[] {
      new NerdyFalcon(RobotMap.kLeftFollowerTalon1ID),
    },
    new CANMotorController[] {
      new NerdyFalcon(RobotMap.kRightFollowerTalon1ID),
    },
     true, false, new Piston(RobotMap.  kShifterPort1ID, RobotMap.kShifterPort2ID),
      DriveConstants.kTrackWidth);
      
    // (NerdyFalcon) super.m_leftSlaves[0]
    // super.m_rightSlaves[0].followCANMotorController(super.m_rightMaster);

    // super.m_rightSlaves[]  
    //  super.configAutoChooser(Robot.autoChooser);
     super.configMaxVelocity(DriveConstants.kMaxVelocity);
     super.configSensorPhase(false, true);
     super.configTicksPerFoot(DriveConstants.kLeftTicksPerFoot, DriveConstants.kRightTicksPerFoot);
     super.configLeftPIDF(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD, DriveConstants.kLeftF);
     super.configRightPIDF(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD, DriveConstants.kRightF);
     super.configStaticFeedforward(DriveConstants.kLeftRamseteS, DriveConstants.kRightRamseteS);
    
     super.m_leftMaster.configCurrentLimitContinuous(50);
     super.m_rightMaster.configCurrentLimitContinuous(50);
     super.m_leftMaster.configCurrentLimitPeak(50);
     super.m_rightMaster.configCurrentLimitPeak(50);

     super.m_leftSlaves[0].configCurrentLimitContinuous(50);
     super.m_rightSlaves[0].configCurrentLimitContinuous(50);
     super.m_leftSlaves[0].configCurrentLimitPeak(50);
     super.m_rightSlaves[0].configCurrentLimitPeak(50);
     setCoastMode();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.updateOdometry();
    super.reportToSmartDashboard();
  }
}
