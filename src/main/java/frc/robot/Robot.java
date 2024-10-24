// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //public static final Solenoid BREAK = new Solenoid(PneumaticsModuleType.CTREPCM, 5);
  private final double baseSpeed = 0.7; //Base robot speed
  private final double creepSpeed = 0.5; // Speed of the robot in creep mode
  private DifferentialDrive driveInstance; // Create a differenetial drive object
  private XboxController controllerInput; // Create a controller input
  double speedMultiplier = 1; // Speed multiplier for flip drive
  double creepMultiplier; 
  double leftAxisController;
  double rightAxisController;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Motor setup start
    // PWMSparkMax leftMotor = new PWMSparkMax(0); // PWM Motor motor setup
    // PWMSparkMax leftFollowerMotor = new PWMSparkMax(1);
    // PWMSparkMax rightMotor = new PWMSparkMax(2);
    // PWMSparkMax rightFollowerMotor = new PWMSparkMax(3);
    // leftMotor.addFollower(leftFollowerMotor); // make the leftfollower motor follow the main left motor
    // rightMotor.addFollower(rightFollowerMotor);
    CANSparkMax leftMotor = new CANSparkMax(4, MotorType.kBrushed); // CAN motor setup
    CANSparkMax leftFollowerMotor = new CANSparkMax(3, MotorType.kBrushed);
    CANSparkMax rightMotor = new CANSparkMax(1, MotorType.kBrushed);
    CANSparkMax rightFollowerMotor = new CANSparkMax(2, MotorType.kBrushed);
    leftFollowerMotor.follow(leftMotor); // make the leftfollower motor follow the main left motor
    rightFollowerMotor.follow(rightMotor);
    rightMotor.setInverted(true); // Reverse right motors
    driveInstance = new DifferentialDrive(leftMotor, rightMotor); // Drive instance with differential drive using both motors.
    // Motor setup end
    controllerInput = new XboxController(0); // Input for the controller, port 0 on driver station
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //BREAK.set(true);
  }
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    speedMultiplier = controllerInput.getRightBumperPressed() ? (speedMultiplier * -1.0) : speedMultiplier; // An If statement for flip drive
    creepMultiplier = controllerInput.getRightTriggerAxis() > 0.5 ? creepSpeed : baseSpeed; // An If statement for creep drive
    double controllerY = controllerInput.getLeftY() * creepMultiplier * speedMultiplier; // Get Y input from left joystick
    double controllerX = controllerInput.getRightX() * creepMultiplier; // Get X input from right joystick
    leftAxisController = (controllerY + controllerX); // Driving math for left motors
    rightAxisController = -(controllerY - controllerX); // Driving math for right motors
    driveInstance.tankDrive(leftAxisController, rightAxisController); // Use the tank drive class to send the final outputs to the motors
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    //BREAK.set(false);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
