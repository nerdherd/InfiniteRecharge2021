/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.nerdherd.lib.drivetrain.experimental.ShiftingDrivetrain;
import com.nerdherd.lib.motor.motorcontrollers.CANMotorController;
import com.nerdherd.lib.motor.motorcontrollers.NerdyFalcon;
import com.nerdherd.lib.pneumatics.Piston;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.constants.DriveConstants;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends ShiftingDrivetrain {
  /**
   * Creates a new Drive.
   */
  public DifferentialDrivetrainSim m_driveSim;
  public AHRS m_gyroSim;

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
     
     m_driveSim = new DifferentialDrivetrainSim(
      LinearSystemId.identifyDrivetrainSystem(
        DriveConstants.kVLinear, DriveConstants.kALinear, DriveConstants.kVAngular, DriveConstants.kAAngular), 
       DCMotor.getFalcon500(4), 
       DriveConstants.gearReduction, 
       DriveConstants.kTrackWidth, 
       DriveConstants.wheelRadiusMeters, 
       VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
    
    m_gyroSim = new AHRS();
  }

  @Override
  public void simulationPeriodic() {
    // int dev = SimDeviceDataJNI.getSimDeviceHandle(“nav  X-Sensor[0]”);
    // SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, “Yaw”));
    // angle.set(5.0);
    // SimDevice gyro = new SimDevice(0);
    // int dev = SimDeviceDataJNI.getSimDeviceHandle(“navX-Sensor[0]”);
    // SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, “Yaw”));
    // angle.set(5.0);
    // m_driveSim.
    m_driveSim.setInputs(1,1);
    // m_driveSim.
    m_driveSim.update(0.02);
    // m_gyroSim.set(90);
    SmartDashboard.putNumber("Sim Pose X Meters", m_driveSim.getPose().getX());
    SmartDashboard.putNumber("Sim Pose Y Meters", m_driveSim.getPose().getY());
    SmartDashboard.putNumber("Sim heading", m_driveSim.getPose().getRotation().getDegrees());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.updateOdometry();
    super.reportToSmartDashboard();
  }
}
