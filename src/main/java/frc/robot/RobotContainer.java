// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controls.WristControls;
import frc.robot.subsystems.WristSubsystemMotorPower;


public class RobotContainer {

  /**
   * In this class we generally only declare and create subsystems, controls(like WristControls), and a few other things. 
   */

  //Create instance variables for the WristSubsystem and WristControls classes
  //see WristControls.java for an explanation on instance variables. 
  private final WristSubsystemMotorPower m_wrist; 

  private final WristControls m_wristControls; 

  //This is the constructor. See WristControls.java for an explanation on constructors. 
  public RobotContainer() {
    //assign m_wrist to a WristSubsystemMotorPower object. 
    m_wrist = new WristSubsystemMotorPower(); 

    /**
     * Give the m_wristControls instance variable a WristControls object to hold
     *     
     * Additionally, pass in m_wrist, the WristSubsystemMotorPower object(created above) 
     * 
     * The WristControls() class requires a WristSubsystemMotorPower object as a parameter. 
     * 
     */
    m_wristControls = new WristControls(m_wrist);
    
    //configure the button-command bindings
    m_wristControls.configureControls();
  }


}
