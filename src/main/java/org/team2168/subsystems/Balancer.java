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

public class Balancer extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public static final boolean BALANCER_MOTOR_REVERSE = false;
  private CANSparkMax _balancerMotor;
  private static Balancer instance = null;

  /**
   * Basic constructors for Balancer
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
   * @return An isntance of the Balancer subsystem
   */

  public static Balancer getInstance()
  {
    if (instance == null)
      instance = new Balancer();
    
    return instance;
  }

  /**
   * 
   * @param double Sets speed of Balancer Motor, positive is
   * right, negative is left, 0 is stationary
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
    //setDefaultCommand(new DriveBalancerMotorWithJoystick());
  }
}
