/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem class for the Hopper
 */
public class Hopper extends Subsystem {
  public static final boolean HOPPER_MOTOR_REVERSE = false;
  private TalonSRX hopperMotor;

  private static Hopper _instance = null;

  private Hopper() {
    hopperMotor = new TalonSRX(RobotMap.HOPPER_MOTOR_PDP);
    //The methods for the voltage and currents do not yet exist.
  }

  /**
   * Returns a singleton instance of the Hopper
   */
  public static Hopper getInstance() {
    if(_instance == null) {
      _instance = new Hopper();
    }
    return _instance;
  }

  /**
   * Drives the hopper motor at the specified speed
   * @param speed is a double from -1 to 1; positive is heading towards the shooter
   */
  public void drive(double speed) {
    if(HOPPER_MOTOR_REVERSE) {
      speed = -speed;
    }
    hopperMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
