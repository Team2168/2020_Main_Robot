//This is using PWM, falcon 500

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team2168.RobotMap;
import org.team2168.commands.balancer.DriveBalancerMotorWithJoystick;

import edu.wpi.first.wpilibj.DoubleSolenoid;



/**
 * Add your docs here.
 */
public class Balancer extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public static final boolean BALANCER_MOTOR_REVERSE = false;
  private CANSparkMax _balancerMotor;
  private DoubleSolenoid _brakes;

  private static Balancer instance = null;

  /**
   * As of the initial writing of this code, the balancer will require at least one motor controller and one double Solenoid
   */


  private Balancer()
  {
    _balancerMotor = new CANSparkMax(RobotMap.BALANCER_MOTOR_PDP, MotorType.kBrushless);
    
    //speed limit 60
    _balancerMotor.setSmartCurrentLimit(60);

    //control frame every 20ms
    _balancerMotor.setControlFramePeriodMs(20);

    //status frame every 500ms
    _balancerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
  }
  
  /**
   * Calls instance object and makes it a singleton object of type Balancer
   * 
   * @return Balancer object "instance"
   */
  public static Balancer getInstance()
  {
    if (instance == null)
      instance = new Balancer();
    
    return instance;
  }

  /**
   * Calls _balancerMotor and creates a local variable "speed" Refers to boolean in
   * Robot map and if true, speed = - speed Uses set() command to assign the new
   * speed to _balacnerMotor
   * 
   * @param double speed between -1 and 1 negative is reverse, positive if
   *        forward, 0 is stationary
   */
    public void driveMotor(double speed)
    {
      if (BALANCER_MOTOR_REVERSE)
        speed = -speed;

      _balancerMotor.set(speed);
    }
    

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveBalancerMotorWithJoystick());
  }
}
