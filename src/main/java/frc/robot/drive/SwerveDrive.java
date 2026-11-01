/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.drive;

import frc.robot.parameters.SwerveDriveParameters;
import frc.robot.util.MatchUtil;
import java.util.function.Supplier;
import org.wpilib.datalog.DataLog;
import org.wpilib.datalog.DoubleLogEntry;
import org.wpilib.drive.RobotDriveBase;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.system.DataLogManager;

/** SwerveDrive implements swerve drive control. */
public class SwerveDrive extends RobotDriveBase {
  private static final DataLog LOG = DataLogManager.getLog();

  private static final ChassisVelocities ZERO_SPEEDS = new ChassisVelocities();

  private final SwerveModule[] modules;
  private final SwerveDriveKinematics kinematics;
  private final Supplier<Rotation2d> orientationSupplier;
  private final double maxDriveSpeed;
  private final double maxRotationalSpeed;
  private final ChassisVelocities chassisSpeeds = new ChassisVelocities();

  // The current supplied state updated by the periodic method.
  private Rotation2d orientation;

  private final DoubleLogEntry xSpeedLog = new DoubleLogEntry(LOG, "/SwerveDrive/xSpeed");
  private final DoubleLogEntry ySpeedLog = new DoubleLogEntry(LOG, "/SwerveDrive/ySpeed");
  private final DoubleLogEntry omegaSpeedLog = new DoubleLogEntry(LOG, "/SwerveDrive/omegaSpeed");

  private final SwerveModuleVelocity[] moduleStates = new SwerveModuleVelocity[4];
  private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

  /**
   * constructs the swerve drive
   *
   * @param parameters A {@link SwerveDriveParameters} object providing information on the physical
   *     swerve drive characteristics.
   * @param modules An array of four {@link SwerveModule} objects in the order: front left, front
   *     right, back left, back right.
   * @param kinematics A {@link SwerveDriveKinematics} object used to convert chassis velocity into
   *     individual module states.
   * @param orientationSupplier Supplies the robot orientation relative to the field.
   */
  public SwerveDrive(
      SwerveDriveParameters parameters,
      SwerveModule[] modules,
      Supplier<Rotation2d> orientationSupplier) {
    this.modules = modules;
    this.kinematics = parameters.getKinematics();
    this.orientationSupplier = orientationSupplier;
    this.maxDriveSpeed = parameters.getMaxDriveSpeed();
    this.maxRotationalSpeed = parameters.getMaxRotationalSpeed();

    initializeSuppliedState();
  }

  /** Initializes the supplied state. */
  private void initializeSuppliedState() {
    updateSuppliedState();
  }

  /**
   * Updates the supplied state.
   *
   * <p>This method **MUST* be called by the {@link #periodic()} method to ensure the supplied state
   * is up to date for subsequent use.
   */
  private void updateSuppliedState() {
    orientation = orientationSupplier.get();
  }

  /**
   * Sets the swerve module states.
   *
   * @param states An array of four {@link SwerveModuleVelocity} objects in the order: front left,
   *     front right, back left, back right
   */
  public void setModuleStates(SwerveModuleVelocity[] states) {
    states = SwerveDriveKinematics.desaturateWheelVelocities(states, maxDriveSpeed);

    for (int i = 0; i < modules.length; ++i) {
      modules[i].setState(states[i]);
    }

    // Reset the motor watchdog timer.
    feedWatchdog();
  }

  @Override
  public void stopMotor() {
    setChassisVelocities(ZERO_SPEEDS);

    for (SwerveModule module : modules) {
      module.disable();
    }
  }

  @Override
  public String getDescription() {
    return "SwerveDrive";
  }

  /**
   * Returns the orientation of the robot frame relative to the field.
   *
   * @return The orientation of the robot frame.
   */
  public Rotation2d getOrientation() {
    return orientation;
  }

  /**
   * Drives the robot based on joystick inputs.
   *
   * @param xSpeed Speed of the robot in the x direction.
   * @param ySpeed Speed of the robot in the y direction.
   * @param rSpeed Rotation speed of the robot.
   * @param fieldRelative Whether the x and y values are relative to field.
   */
  public void drive(double xSpeed, double ySpeed, double rSpeed, boolean fieldRelative) {
    // multiples all values with max speed.
    xSpeed *= m_maxOutput * maxDriveSpeed;
    ySpeed *= m_maxOutput * maxDriveSpeed;
    rSpeed *= m_maxOutput * maxRotationalSpeed;

    if (!fieldRelative) {
      chassisSpeeds.vx = xSpeed;
      chassisSpeeds.vy = ySpeed;
      chassisSpeeds.omega = rSpeed;
      setChassisVelocities(chassisSpeeds);
    } else {

      if (MatchUtil.isRedAlliance()) {
        xSpeed *= -1.0;
        ySpeed *= -1.0;
      }

      double orientationRadians = -orientation.getRadians();

      double cosOrientation = Math.cos(orientationRadians);
      double sinOrientation = Math.sin(orientationRadians);

      chassisSpeeds.vx = (xSpeed * cosOrientation) - (ySpeed * sinOrientation);
      chassisSpeeds.vy = (xSpeed * sinOrientation) + (ySpeed * cosOrientation);
      chassisSpeeds.omega = rSpeed;

      setChassisVelocities(chassisSpeeds);
    }
  }

  /**
   * Returns the current module state describing the wheel velocity and angle.
   *
   * @return The current module state.
   */
  public SwerveModuleVelocity[] getModuleStates() {
    for (int i = 0; i < modules.length; i++) {
      moduleStates[i] = modules[i].getState();
    }

    return moduleStates;
  }

  /**
   * Sets the current module's states based on the chassis speed.
   *
   * @param speeds The chassis speeds.
   */
  public void setChassisVelocities(ChassisVelocities speeds) {
    xSpeedLog.append(speeds.vx);
    ySpeedLog.append(speeds.vy);
    omegaSpeedLog.append(Math.toDegrees(speeds.omega));

    SwerveModuleVelocity[] states = kinematics.toSwerveModuleVelocities(speeds);

    setModuleStates(states);
  }

  /**
   * Returns the current chassis speed.
   *
   * @return The chassis speed.
   */
  public ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleStates());
  }

  /**
   * Returns the swerve module positions.
   *
   * @return Swerve module positions.
   */
  public SwerveModulePosition[] getModulesPositions() {
    for (int i = 0; i < modules.length; i++) {
      modulePositions[i] = modules[i].getPosition();
    }

    return modulePositions;
  }

  /**
   * This method is called periodically by the SwerveSubsystem. It is used to update drive-specific
   * state.
   */
  public void periodic() {
    updateSuppliedState();

    for (SwerveModule module : modules) {
      module.periodic();
    }
  }
}
