/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import org.team2168.RobotMap;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Indexer extends Subsystem {
  // Puts methods for controlling this subsystem
  // here. Call these from Commands.
  private final boolean INDEXER_MOTOR_INVERTED = false;
  private TalonSRX _motor;
  private static DigitalInput entranceLineBreak;
  private static DigitalInput exitLineBreak;
  private static Indexer _instance = null;

  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20; //amps
  private final double TRIGGER_THRESHOLD_LIMIT = 30; //amp
  private final double TRIGGER_THRESHOLD_TIME = 0.2; //s

  private Indexer(){
    _motor = new TalonSRX(RobotMap.INDEXER_MOTOR_PDP);
    entranceLineBreak = new DigitalInput(RobotMap.ENTRANCE_LINE_BREAK);
    exitLineBreak = new DigitalInput(RobotMap.EXIT_LINE_BREAK);
    _motor.setNeutralMode(NeutralMode.Brake);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    _motor.configSupplyCurrentLimit(talonCurrentLimit);

    _motor.setNeutralMode(NeutralMode.Brake);
    _motor.setInverted(INDEXER_MOTOR_INVERTED);
    _motor.configNeutralDeadband(0.05);

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
    // if(_INDEXER_MOTOR_REVERSED) {
    //   speed = speed * -1;
    // }
    _motor.set(ControlMode.PercentOutput, speed);
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

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new DriveIndexerWithJoystick());
  }
}
