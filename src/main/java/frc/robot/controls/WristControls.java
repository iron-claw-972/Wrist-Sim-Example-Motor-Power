package frc.robot.controls;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.WristSubsystemMotorPower;

public class WristControls {

  /**
   * Below are instance variables. Think of them as the specific variables that we are going to need for THIS class
   * 
   * m_wrist is a variable of the type WristSubsystemMotorPower
   * m_button1, m_button1, and m_button3 are variables of the type Trigger
   * m_joy is a variable of the type Joystick
  */

  WristSubsystemMotorPower m_wrist; 

  Trigger m_button1;
  Trigger m_button2;
  Trigger m_button3;
  Joystick m_joy; 

  /**
   * This is the constructor for the WristControls class. We give the constructor a parameter -- an object(wristSubsystemMotorPower) of the type WristSubsystemMotorPower(a class). 
   * 
   * <p>
   * 
   * To satisfy this parameter, when we create an object of this controls class in RobotContainer.java we will pass in a WristSubsystemMotorPower object into it. 
   * 
   * <p> 
   * 
   * By doing m_wrist = wristSubsystemMotorPower(line 49), m_wrist now holds a WristSubsystemMotorPower class now. We can now access all the methods inside WristSubsystemMotorPower.java
   * 
   * <p>
   * 
   * For the other instance variables, we can just pass in objects of our desired classes to them by doing m_instanceVariable = new Class(); .We do this in the constructor, ex: m_joy = new Joystick(0); 
   * 
   * <p> 
   * 
   * @param wristSubsystem
   */
  public WristControls(WristSubsystemMotorPower wristSubsystemMotorPower){
    
    m_wrist = wristSubsystemMotorPower; 
    
    //create a joystick object and assign it to m_joy. this joystick and its buttons can be accessed in simulation and real life. 
    m_joy = new Joystick(0); 
    
    m_button1 = new JoystickButton(m_joy, 1);
    m_button2 = new JoystickButton(m_joy, 2);
    m_button3 = new JoystickButton(m_joy,3);
  }


  /**
   * The configureControls() method is used for control binding.
   */
  public void configureControls(){

    /**
     * When a button is pressed an InstantCommand() is created that, on button press,
     * sets the desired motorPower to the wrist by using the setMotorPower() method declared in WristSubsystemMotorPower.java
     */

    m_button1.onTrue(new InstantCommand(()-> m_wrist.setMotorPower(0.25)));
    m_button2.onTrue(new InstantCommand(()-> m_wrist.setMotorPower(-0.25)));


  }
}
