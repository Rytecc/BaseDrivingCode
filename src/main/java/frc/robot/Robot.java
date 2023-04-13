// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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
  private final boolean isControllerMode = true;
  private final boolean hasLights = true;

  private final double kCreepSpeed = 0.6;
  private final double kBaseSpeed = 0.7;
  private Lights lightInstance;
  private DifferentialDrive driveInstance;
  private XboxController input_controller;
  private Joystick input;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /*
    CANSparkMax upLeft = new CANSparkMax(3, MotorType.kBrushless);
    CANSparkMax downLeft = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax upRight = new CANSparkMax(5, MotorType.kBrushless);
    CANSparkMax downRight = new CANSparkMax(6, MotorType.kBrushless);
    */
    PWMSparkMax upLeft = new PWMSparkMax(0);
    PWMSparkMax downLeft = new PWMSparkMax(1);
    PWMSparkMax upRight = new PWMSparkMax(2);
    PWMSparkMax downRight = new PWMSparkMax(3);

    MotorControllerGroup left = new MotorControllerGroup(upLeft, downLeft);
    MotorControllerGroup right = new MotorControllerGroup(upRight, downRight);
    driveInstance = new DifferentialDrive(left, right);

    if(isControllerMode) {
      input_controller = new XboxController(0);
      if(hasLights) lightInstance = new Lights(input_controller, 9, 56);
      return;
    }
    
    input = new Joystick(0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if(hasLights) {
      lightInstance.lightsPeriodic();
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //BREAK.set(true);
  }
  double multiplier = 1;
  double creepMultiplier;
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(isControllerMode) {
      multiplier = input_controller.getRightBumperPressed() ? (multiplier * -1.0) : multiplier;
      creepMultiplier = input_controller.getLeftTriggerAxis() > 0.5 ? kCreepSpeed : kBaseSpeed;

      double inX = input_controller.getLeftX() * creepMultiplier;
      double inY = input_controller.getLeftY() * creepMultiplier * multiplier;
      driveInstance.arcadeDrive(inX, inY);
      return;
    }

    multiplier = input.getRawButtonPressed(2) ? (multiplier * -1.0) : multiplier;
    creepMultiplier = input.getRawButton(1) ? kCreepSpeed : kBaseSpeed;
    double inX = input.getX() * creepMultiplier;
    double inY = input.getY() * creepMultiplier * multiplier;
    driveInstance.arcadeDrive(inX, inY);
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
