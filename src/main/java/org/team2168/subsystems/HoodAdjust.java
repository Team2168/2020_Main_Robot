/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

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
  public HoodPosition shooterPosition;

  private HoodAdjust()
  {
    _hoodSolenoid = new DoubleSolenoid(RobotMap.PCM_CAN_ID_SHOOTER, RobotMap.HOOD_SOLENOID_ENGAGE, RobotMap.HOOD_SOLENOID_DISENGAGE);
    _pancakeSolenoid = new DoubleSolenoid(RobotMap.PCM_CAN_ID_SHOOTER, RobotMap.PANCAKE_SOLENOID_IN, RobotMap.PANCAKE_SOLENOID_OUT);
    shooterPosition = HoodPosition.WALL;
  }
  /**
   * Creates a new Instance of the HoodAdjust
   * @return - Returns the new instance of
   * the HoodAdjust, to be used by 
   * the commands
   */
  public static HoodAdjust getInstance() {
    if(_instance == null)
      _instance = new HoodAdjust();
    return _instance;
  }
  /**
   * The hood, controlled by a solenoid, 
   * is set to the foreward position
   */
  public void extendHood() 
  {
      _hoodSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  /**
   * The hood, controlled by a solenoid, 
   * is set to reverse, pulling it back 
   * into the shooter
   */
  public void retractHood()
  {
      _hoodSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * The hard stop, or pancake, is extended,
   * to create two more positions to shoot
   * from.
   */
  public void extendPancake()
  {
    _pancakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }
/**
   * The hard stop, or pancake, is retracted,
   * allowing the hood to move to the 
   * other two positions
   */
  public void retractPancake()
  {
    _pancakeSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
 /**
  * Checks to see if the hood is extended
  * @return - Returns if the hood is in the 
  * foreward position
  */
  public boolean isHoodExtended()
  {
      return _hoodSolenoid.get()==DoubleSolenoid.Value.kForward;
  }
   /**
  * Checks to see if the hood is retracted
  * @return - Returns if the hood is in the 
  * reversed position
  */
  public boolean isHoodRetracted()
  {
      return _hoodSolenoid.get()==DoubleSolenoid.Value.kReverse;
  }
   /**
  * Checks to see if the pancake
  * @return - Returns if the pancake is
  * extended, which would help the shooter 
  * stay in new locations
  */
  public boolean isPancakeExtended()
  {
    return _pancakeSolenoid.get()==DoubleSolenoid.Value.kForward;
  }
  /**
  * Checks to see if the pancake
  * @return - Returns if the pancake is
  * retracted, which would allow the shooter to 
  * switch between its two defaut positions
  */
  public boolean isPancakeRetracted()
  {
    return _pancakeSolenoid.get()==DoubleSolenoid.Value.kReverse;
  }

  public enum HoodPosition{
    WALL,
    WHITE_LINE,
    FRONT_TRENCH,
    BACK_TRENCH
  }

  public void setHoodPosition(HoodPosition newPosition){
    shooterPosition = newPosition;
  }



  @Override
  public void initDefaultCommand() {
    
  }
}
