// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Scoring;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.ShooterConstants.ShooterWantedState;
import frc.robot.Constants.ShooterConstants.SystemState;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
  private CommandSwerveDrivetrain drivetrain;

  /* MOTORS */
  private TalonFX hoodMotor = new TalonFX(ShooterConstants.hoodMotorID, CANBus.roboRIO());
  private TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
  private TalonFX shooterMotor1 = new TalonFX(ShooterConstants.shooterMotor1ID, CANBus.roboRIO());
  private TalonFX shooterMotor2 = new TalonFX(ShooterConstants.shooterMotor2ID, CANBus.roboRIO());
  private TalonFXConfiguration shooterMotor1Config = new TalonFXConfiguration();
    private TalonFXConfiguration shooterMotor2Config = new TalonFXConfiguration();


  //for velocity control
    final DutyCycleOut t_request = new DutyCycleOut(0);
  private double motorspeed = 0.0;
  final MotionMagicVelocityVoltage mm_request = new MotionMagicVelocityVoltage(0);
  //for position control
  private double position = 0.0;
  final PositionVoltage mmE_request = new PositionVoltage(0);

  /* PIDFF CONTROL */
  private LoggedTunableNumber k_S = new LoggedTunableNumber("shooter_s", ShooterConstants.shooterSVA[0]);
  private LoggedTunableNumber k_V = new LoggedTunableNumber("shooter_v", ShooterConstants.shooterSVA[1]);
  private LoggedTunableNumber k_A = new LoggedTunableNumber("shooter_a", ShooterConstants.shooterSVA[2]);
  private LoggedTunableNumber k_P = new LoggedTunableNumber("shooter_p", ShooterConstants.shooterPID[0]);
  private LoggedTunableNumber k_I = new LoggedTunableNumber("shooter_i", ShooterConstants.shooterPID[1]);
  private LoggedTunableNumber k_D = new LoggedTunableNumber("shooter_d", ShooterConstants.shooterPID[2]);

  /* STATES */
  ShooterWantedState wantedState = ShooterWantedState.IDLE;
  SystemState systemState = SystemState.IDLING;

  /** Creates a new Shooter */
  public Shooter(CommandSwerveDrivetrain m_drivetrain) {
    this.drivetrain = m_drivetrain;
    /* SETUP CONFIG */
    
    // CURRENT LIMITS
    hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
    hoodMotorConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.StatorCurrentLimit;
    shooterMotor2Config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
    shooterMotor2Config.CurrentLimits.StatorCurrentLimit = ShooterConstants.StatorCurrentLimit;
    shooterMotor1Config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
    shooterMotor1Config.CurrentLimits.StatorCurrentLimit = ShooterConstants.StatorCurrentLimit;
    
    //PID CONSTANTS
    hoodMotorConfig.Slot0.kS = ShooterConstants.hoodSVA[0];
    hoodMotorConfig.Slot0.kV = ShooterConstants.hoodSVA[1];
    hoodMotorConfig.Slot0.kA = ShooterConstants.hoodSVA[2]; 
    hoodMotorConfig.Slot0.kP = ShooterConstants.hoodPID[0];
    hoodMotorConfig.Slot0.kI = ShooterConstants.hoodPID[1];
    hoodMotorConfig.Slot0.kD = ShooterConstants.hoodPID[2];

    shooterMotor1Config.Slot0.kS = k_S.get();
    shooterMotor1Config.Slot0.kV = k_V.get();
    shooterMotor1Config.Slot0.kA = k_A.get(); 
    shooterMotor1Config.Slot0.kP = k_P.get();
    shooterMotor1Config.Slot0.kI = k_I.get();
    shooterMotor1Config.Slot0.kD = k_D.get();
    
    shooterMotor2Config.Slot0.kS = k_S.get();
    shooterMotor2Config.Slot0.kV = k_V.get();
    shooterMotor2Config.Slot0.kA = k_A.get(); 
    shooterMotor2Config.Slot0.kP = k_P.get();
    shooterMotor2Config.Slot0.kI = k_I.get();
    shooterMotor2Config.Slot0.kD = k_D.get();

    shooterMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterMotor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    shooterMotor1Config.Voltage.PeakReverseVoltage = 0;
    shooterMotor2Config.Voltage.PeakReverseVoltage = 0;


    shooterMotor1Config.MotionMagic.MotionMagicAcceleration = ShooterConstants.shooterMotionMagicAccel;
    shooterMotor1Config.MotionMagic.MotionMagicJerk = ShooterConstants.shooterMotionMagicJerk;

    shooterMotor2Config.MotionMagic.MotionMagicAcceleration = ShooterConstants.shooterMotionMagicAccel; 
    shooterMotor2Config.MotionMagic.MotionMagicJerk = ShooterConstants.shooterMotionMagicJerk;


    //use this for motion magic expo (very good control of position)
    hoodMotorConfig.MotionMagic.MotionMagicExpo_kV = ShooterConstants.shooterMotionMagicExpoK_V;
    hoodMotorConfig.MotionMagic.MotionMagicExpo_kA = ShooterConstants.shooterMotionMagicExpoK_A;

    //APPLY CONFIG TO MOTOR
    StatusCode hoodMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      hoodMotorStatus = hoodMotor.getConfigurator().apply(hoodMotorConfig);
      if (hoodMotorStatus.isOK()) break;
    }
    if (!hoodMotorStatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + hoodMotorStatus.toString()  + hoodMotor.getDeviceID());
    }

    StatusCode shooterMotor1Status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      shooterMotor1Status = shooterMotor1.getConfigurator().apply(shooterMotor1Config);
      if (shooterMotor1Status.isOK()) break;
    }
    if (!shooterMotor1Status.isOK()) {
      System.out.println("Could not apply configs, error code: " + shooterMotor1Status.toString() + shooterMotor1.getDeviceID());
    }

    StatusCode shooterMotor2Status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      shooterMotor2Status = shooterMotor2.getConfigurator().apply(shooterMotor2Config);
      if (shooterMotor2Status.isOK()) {
            break;
      }
    }
    if (!shooterMotor2Status.isOK()) {
      System.out.println("Could not apply configs, error code: " + shooterMotor2Status.toString() + shooterMotor2.getDeviceID());
    }
    hoodMotor.setPosition(0);
  }

  public void setWantedShooterState(ShooterWantedState desiredState) {
    this.wantedState = desiredState;
  }

  private SystemState changeCurrentSystemState() {
    /// DriverStation.getGameSpecificMessage();     DriverStation.getAlliance();
    return switch (wantedState) {
      case IDLE:
        yield SystemState.IDLING;
      case WAIT:
        yield SystemState.ACTIVE_WAITING;
        // if(DriverStation.getGameSpecificMessage() == getAlliance()){
        //   if((DriverStation.getMatchTime() <= 105 && DriverStation.getMatchTime() > 80) || (DriverStation.getMatchTime() <= 55 && DriverStation.getMatchTime() > 30)){
        //     yield SystemState.ACTIVE_WAITING;
        //   }
        //   if((DriverStation.getMatchTime() <= 130 && DriverStation.getMatchTime() > 105) || (DriverStation.getMatchTime() <= 80 && DriverStation.getMatchTime() > 55)){
        //     yield SystemState.INACTIVE_WAITING;
        //   }
        // }
        // else{
        //   if((DriverStation.getMatchTime() <= 130 && DriverStation.getMatchTime() > 105) || (DriverStation.getMatchTime() <= 80 && DriverStation.getMatchTime() > 55)){
        //     yield SystemState.ACTIVE_WAITING;
        //   }
        //   if((DriverStation.getMatchTime() <= 105 && DriverStation.getMatchTime() > 80) || (DriverStation.getMatchTime() <= 55 && DriverStation.getMatchTime() > 30)){
        //     yield SystemState.INACTIVE_WAITING;
        //   }
        // }
      case TRENCH_SHOOT:
        yield systemState.TRENCH_SHOOTING;
      case PASS_SHOOT:
        yield SystemState.PASS_SHOOTING;
      case HUB_SHOOT:
        yield SystemState.HUB_SHOOTING;
      case HOME:
        yield SystemState.HOMING;
      case TEST:
        yield SystemState.TESTING;
      case RETRACT_AUTO:
        yield SystemState.RETRACTING_AUTO;
      case TURN_ON_AUTO:
        yield SystemState.TURNING_ON_AUTO;
    };
  }

    
  private void applyState(){
    switch(systemState){
      case IDLING:
        motorspeed = 0.0;
        position = 0.0;
        break;
      case ACTIVE_WAITING:
        motorspeed = ShooterConstants.activeWaitingSpeed;
        position = 0.0;
        break;
      case INACTIVE_WAITING:
        motorspeed = ShooterConstants.inactiveWaitingSpeed;
        position = 0.0;
        break;
      case TRENCH_SHOOTING:
        motorspeed = 50;
        position = 5.5;
        break;
      case HUB_SHOOTING:
    Translation2d correctedVector = drivetrain.getSOTFTurretAngle("hub");
    double correctedDistance = correctedVector.getNorm();

    motorspeed = ShooterConstants.shooterSpeedInterpolation
        .getPrediction(correctedDistance);

    position = ShooterConstants.hoodAngleInterpolation
        .getPrediction(correctedDistance);
    break;
      case PASS_SHOOTING:
         Translation2d correctedVector2 = drivetrain.getSOTFTurretAngle("pass");
    double correctedDistance2 = correctedVector2.getNorm();

    motorspeed = ShooterConstants.shooterSpeedInterpolation
        .getPrediction(correctedDistance2);

    position = ShooterConstants.hoodAngleInterpolation
        .getPrediction(correctedDistance2);
    break;
      case HOMING:
        position = -.1;
        if (hoodMotor.getSupplyCurrent().getValueAsDouble() >= ShooterConstants.homingThreshold) {
          hoodMotor.setPosition(0);
          position = 0;
        } 
        setWantedShooterState(ShooterWantedState.IDLE);
        break;
      case TESTING:
        // change these to find interpolation values
        motorspeed = 45;
        position = 3;
        break;
      case RETRACTING_AUTO:
        position = 0;
      case TURNING_ON_AUTO:
        motorspeed = 50;
    }
  } 


  /**
   * Check LoggedTunableNumbers. If changed, update PID and SVA values of motor
   */
  public void checkTunableValues() {    

    if (k_S.hasChanged() || k_V.hasChanged() || k_A.hasChanged() 
    || k_P.hasChanged() || k_I.hasChanged() || k_D.hasChanged()) {
      shooterMotor1Config.Slot0.kS = k_S.get();
      shooterMotor1Config.Slot0.kV = k_V.get();
      shooterMotor1Config.Slot0.kA = k_A.get(); 
      shooterMotor1Config.Slot0.kP = k_P.get();
      shooterMotor1Config.Slot0.kI = k_I.get();
      shooterMotor1Config.Slot0.kD = k_D.get();
      
      shooterMotor2Config.Slot0.kS = k_S.get();
      shooterMotor2Config.Slot0.kV = k_V.get();
      shooterMotor2Config.Slot0.kA = k_A.get(); 
      shooterMotor2Config.Slot0.kP = k_P.get();
      shooterMotor2Config.Slot0.kI = k_I.get();
      shooterMotor2Config.Slot0.kD = k_D.get();
      
    }
  }

  private String getAlliance(){
    if(DriverStation.getAlliance().get() == Alliance.Red){
      return "R";
    } else{
      return "B";
    }
  }

  public boolean shooterIsReady() {
    if(Math.abs(shooterMotor1.getVelocity().getValueAsDouble() - motorspeed) < 1){
      return true;
    } else {
      return false;
    }
  }

  private void logValues() {
    SmartDashboard.putNumber("Shooter Actual Speed", shooterMotor1.getVelocity().getValueAsDouble());
    // SmartDashboard.putNumber("Hood Wanted Position", position);
    SmartDashboard.putNumber("Hood Actual Position", hoodMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Wanted Speed", motorspeed);
    SmartDashboard.putNumber("Hood Motor Current", hoodMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putString("SHOOTER WANTED STATE", wantedState.toString());
    SmartDashboard.putString("SHOOTER SYSTEM STATE", systemState.toString());
  }

  @Override
  public void periodic() {
    logValues();
    systemState = changeCurrentSystemState();
    applyState();
    //example of how to control motor for position
    hoodMotor.setControl(mmE_request.withPosition(position));
    //example of how to control motor for velocity
    shooterMotor1.setControl(mm_request.withVelocity(motorspeed));
    shooterMotor2.setControl(mm_request.withVelocity(motorspeed));
    // shooterMotor1.set(motorspeed);
    // shooterMotor2.set(motorspeed);
    
  }

}