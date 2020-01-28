/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.Robot;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class HoodAdjust extends Subsystem {

  private DoubleSolenoid _hoodSolenoid;
  private DoubleSolenoid _pancakeSolenoid;

  private static HoodAdjust _instance;

  private HoodAdjust()
  {
    _hoodSolenoid = new DoubleSolenoid(RobotMap.PCM_CAN_ID_SHOOTER, RobotMap.HOOD_SOLENOID_IN, RobotMap.HOOD_SOLENOID_OUT);
    _pancakeSolenoid = new DoubleSolenoid(RobotMap.PCM_CAN_ID_SHOOTER, RobotMap.PANCAKE_SOLENOID_IN, RobotMap.PANCAKE_SOLENOID_OUT);
  }

  public static HoodAdjust getInstance() {
    if(_instance == null)
      _instance = new HoodAdjust();
    return _instance;
  }

  public void extendHood() 
  {
      _hoodSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractHood()
  {
      _hoodSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void extendPancake()
  {
    _pancakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractPancake()
  {
    _pancakeSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean isHoodExtended()
  {
      return _hoodSolenoid.get()==DoubleSolenoid.Value.kForward;
  }
  public boolean isHoodRetracted()
  {
      return _hoodSolenoid.get()==DoubleSolenoid.Value.kReverse;
  }

  public boolean isPancakeExtended()
  {
    return _pancakeSolenoid.get()==DoubleSolenoid.Value.kForward;
  }

  public boolean isPancakeRetracted()
  {
    return _pancakeSolenoid.get()==DoubleSolenoid.Value.kReverse;
  }

  @Override
  public void initDefaultCommand() {
    
  }
}
