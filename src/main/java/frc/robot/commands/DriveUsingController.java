/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.RobotPreferences.RIGHT_TRIGGER_SCALAR;

import frc.robot.subsystems.Swerve;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.math.util.MathUtil;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class DriveUsingController extends Command {

  private static final double DEADBAND = 0.08;

  protected final Swerve drivetrain;
  protected final CommandXboxController xboxController;

  private double powerScalar;

  /** Creates a command which allows a human to drive the robot using an Xbox controller. */
  public DriveUsingController(Swerve drivetrain, CommandXboxController xboxController) {
    this.drivetrain = drivetrain;
    this.xboxController = xboxController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -xboxController.getLeftY();
    double ySpeed = -xboxController.getLeftX();

    // The `powerScalar` linearly scales the robot's drive power from 1.0 (when the right trigger is
    // not pressed) down to RIGHT_TRIGGER_SCALAR (when the right trigger is fully depressed).
    powerScalar = (RIGHT_TRIGGER_SCALAR.getValue() - 1.0) * xboxController.getRightTrigger() + 1.0;

    // Applies deadbands to the x, y, and rotation joystick values and then multiplies all speeds by
    // the powerScalar, which allows finer driving control.
    xSpeed = MathUtil.applyDeadband(xSpeed, DEADBAND) * powerScalar;
    ySpeed = MathUtil.applyDeadband(ySpeed, DEADBAND) * powerScalar;

    drivetrain.drive(xSpeed, ySpeed, calculateRotationSpeed(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  protected double calculateRotationSpeed() {
    double rSpeed = -xboxController.getRightX();
    rSpeed = MathUtil.applyDeadband(rSpeed, DEADBAND) * powerScalar;
    return rSpeed;
  }
}
