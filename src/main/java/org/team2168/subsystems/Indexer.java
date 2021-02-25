/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Add your docs here.
 */
public class Indexer extends SubsystemBase {
  // Puts methods for controlling this subsystem
  // here. Call these from Commands.
  private final boolean _INDEXER_MOTOR_REVERSED = false;
  private CANSparkMax _motor;
  private TalonSRX _srx_motor;
  private static DigitalInput entranceLineBreak;
  private static DigitalInput exitLineBreak;
  private static Indexer _instance = null;

  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20; //amps
  private final double TRIGGER_THRESHOLD_LIMIT = 30; //amp
  private final double TRIGGER_THRESHOLD_TIME = 0.02; //s

  private Indexer(){
    entranceLineBreak = new DigitalInput(RobotMap.ENTRANCE_LINE_BREAK);
    exitLineBreak = new DigitalInput(RobotMap.EXIT_LINE_BREAK);

    //The practice bot currently has a SPARK MAX on the indexer, the comp bot has a Talon SRX
    if(Robot.isPracticeBot()) {
      _motor = new CANSparkMax(RobotMap.INDEXER_MOTOR_PDP, MotorType.kBrushed);
      _motor.setIdleMode(IdleMode.kBrake);

      _motor.setSmartCurrentLimit((int)TRIGGER_THRESHOLD_LIMIT);
      _motor.setControlFramePeriodMs((int)(1000 * TRIGGER_THRESHOLD_TIME));
      _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
      _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
      _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    } else {
      _srx_motor = new TalonSRX(RobotMap.INDEXER_MOTOR_PDP);

      talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
      CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

      _srx_motor.configFactoryDefault();
      _srx_motor.configSupplyCurrentLimit(talonCurrentLimit);

      _srx_motor.setNeutralMode(NeutralMode.Brake);
      _srx_motor.setInverted(true); //output polarity is apparently reversed between SRX and SPARK MAX?
      _srx_motor.configNeutralDeadband(0.05);
    }
    // ConsolePrinter.putBoolean("isBallEntering", ()->{return isBallEntering();}, true, false);
    // ConsolePrinter.putBoolean("isBallExiting", ()->{return isBallExiting();}, true, false);
    ConsolePrinter.putNumber("isBallEntering", ()->{return isBallEnteringDashboard();}, true, false);
    ConsolePrinter.putNumber("isBallExiting", ()->{return isBallExitingDashboard();}, true, false);

  }

  public static Indexer getInstance(){
    if(_instance == null){
      _instance = new Indexer();
    }
    return _instance;
  }

  /**
    * Cycles the indexer
    * @param speed 1.0 to -1.0,  positive is toward the shooter, negative is away from the shooter
    */
  public void drive(double speed) {
    if(_INDEXER_MOTOR_REVERSED) {
      speed = speed * -1;
    }

    if(Robot.isPracticeBot()) {
      _motor.set(speed);
    } else {
      _srx_motor.set(ControlMode.PercentOutput, speed);
    }
  }

  public double isBallEnteringDashboard() {
    if(!entranceLineBreak.get()) {
      return 1.0;
    }
    else {
      return 0.0;
    }
    //return !entranceLineBreak.get();
  }

  public boolean isBallEntering() {
    return !entranceLineBreak.get();
  }

  public double isBallExitingDashboard() {
    if(!exitLineBreak.get()) {
      return 1.0;
    }
    else {
      return 0.0;
    }
  }

  public boolean isBallExiting() {
    return !exitLineBreak.get();
  }

  // @Override
  // public void initDefaultCommand() {
  //   // Set the default command for a subsystem here.
  //   // setDefaultCommand(new DriveIndexerWithJoystick());
  // }
}
