/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.drive;

import com.nrg948.actuator.MotorController;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardRadialGauge;
import com.nrg948.dashboard.annotations.DashboardTextDisplay;
import frc.robot.parameters.SwerveDriveParameters;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.datalog.DataLog;
import org.wpilib.datalog.DoubleLogEntry;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.system.DataLogManager;

/**
 * Manages the drive and steering motors of a single swerve drive module.
 *
 * <p>This class uses a combination feedback (i.e. PID) and feedforward control to achieve desired
 * translational (i.e. drive) and rotational (i.e. steering) velocities. For more information, see
 * the <a href=
 * "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">
 * Introduction to PID</a> and <a href=
 * "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html">
 * Introduction to DC Motor Feedforward</a> articles of the WPILib documentation.
 */
@DashboardDefinition
public final class SwerveModule {
  private static final DataLog LOG = DataLogManager.getLog();

  private final MotorController driveMotor;
  private final DoubleSupplier positionSupplier;
  private final MotorController steeringMotor;
  private final DoubleSupplier velocitySupplier;
  private final Supplier<Rotation2d> wheelAngleSupplier;
  private final DoubleSupplier wheelAngleVelocitySupplier;
  private final String name;

  // models motors mathematically, calculates voltage needed
  private final SimpleMotorFeedforward driveFeedForward;
  private final SimpleMotorFeedforward steeringFeedForward;

  private final PIDController drivePID;
  private final ProfiledPIDController steeringPID;

  // The last motor voltages applied.
  private double driveVoltage;
  private double steeringVoltage;

  // The current supplied state updated by the periodic method.
  private final SwerveModuleVelocity state = new SwerveModuleVelocity();
  private final SwerveModulePosition position = new SwerveModulePosition();

  private final DoubleLogEntry driveSpeedLog;
  private final DoubleLogEntry positionLog;
  private final DoubleLogEntry wheelAngleLog;
  private final DoubleLogEntry wheelAngleVelocityLog;
  private final DoubleLogEntry stateVelocityLog;
  private final DoubleLogEntry stateWheelAngleLog;
  private final DoubleLogEntry driveVoltageLog;
  private final DoubleLogEntry steeringVoltageLog;

  /**
   * Constructs the swerve module.
   *
   * @param parameters A {@link SwerveDriveParameters} object providing information on the physical
   *     swerve drive characteristics.
   * @param driveMotor The drive motor controller.
   * @param position Supplies the position in meters.
   * @param velocity Supplies velocity in meters per second.
   * @param steeringMotor The steering motor controller.
   * @param wheelAngle Supplies the wheel angle.
   * @param wheelAngleVelocity Supplies the wheel angle velocity in radians per second.
   * @param name The name of the module.
   */
  public SwerveModule(
      SwerveDriveParameters parameters,
      MotorController driveMotor,
      DoubleSupplier position,
      DoubleSupplier velocity,
      MotorController steeringMotor,
      Supplier<Rotation2d> wheelAngle,
      DoubleSupplier wheelAngleVelocity,
      String name) {
    this.driveMotor = driveMotor;
    this.steeringMotor = steeringMotor;
    this.wheelAngleSupplier = wheelAngle;
    this.wheelAngleVelocitySupplier = wheelAngleVelocity;
    this.positionSupplier = position;
    this.velocitySupplier = velocity;
    this.name = name;

    this.driveSpeedLog =
        new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/driveSpeed", name));
    this.positionLog = new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/position", name));
    this.wheelAngleLog =
        new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/wheelAngle", name));
    this.wheelAngleVelocityLog =
        new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/wheelAngleVelocity", name));
    this.stateVelocityLog =
        new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/stateVelocity", name));
    this.stateWheelAngleLog =
        new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/stateWheelAngle", name));
    this.driveVoltageLog =
        new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/driveVoltage", name));
    this.steeringVoltageLog =
        new DoubleLogEntry(LOG, String.format("SwerveModule/%s/steeringVoltage", name));

    initializeSuppliedState();

    this.driveFeedForward =
        new SimpleMotorFeedforward(
            parameters.getDriveKs(), parameters.getDriveKv(), parameters.getDriveKa());
    this.steeringFeedForward =
        new SimpleMotorFeedforward(
            parameters.getSteeringKs(), parameters.getSteeringKv(), parameters.getSteeringKa());

    this.drivePID = new PIDController(0, 0, 0.0);

    this.steeringPID = new ProfiledPIDController(3.5, 0, 0.0, parameters.getSteeringConstraints());
    this.steeringPID.enableContinuousInput(-Math.PI, Math.PI);
    this.steeringPID.setTolerance(Math.toRadians(1.0));
    this.steeringPID.reset(getPosition().angle.getRadians());
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
    Rotation2d wheelAngle = wheelAngleSupplier.get();
    double velocity = velocitySupplier.getAsDouble();
    double wheelAngleVelocity = wheelAngleVelocitySupplier.getAsDouble();
    double position = positionSupplier.getAsDouble();

    this.position.distance = position;
    this.position.angle = wheelAngle;

    this.state.velocity = velocity;
    this.state.angle = wheelAngle;

    driveSpeedLog.append(velocity);
    positionLog.append(position);
    wheelAngleLog.append(wheelAngle.getDegrees());
    wheelAngleVelocityLog.append(wheelAngleVelocity);
    driveMotor.logTelemetry();
    steeringMotor.logTelemetry();
  }

  /** Returns the name of the module. */
  public String getName() {
    return name;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param newState The desired state w/ speed and angle
   */
  public void setState(SwerveModuleVelocity newState) {
    // Optimize the state to avoid spinning further than 90 degrees
    Rotation2d currentAngle = getWheelRotation2d();
    newState = newState.optimize(currentAngle);
    newState = newState.cosineScale(currentAngle);

    stateVelocityLog.append(newState.velocity);
    stateWheelAngleLog.append(newState.angle.getDegrees());

    // Calculate the drive motor voltage using PID and FeedForward
    double driveOutput = drivePID.calculate(state.velocity, newState.velocity);
    double driveFeedForward = this.driveFeedForward.calculate(newState.velocity);

    // Calculate the steering motor voltage using PID and FeedForward
    double steeringOutput =
        steeringPID.calculate(currentAngle.getRadians(), newState.angle.getRadians());
    double steeringFeedForward =
        this.steeringFeedForward.calculate(steeringPID.getSetpoint().velocity);

    // Sets voltages of motors
    driveVoltage = driveOutput + driveFeedForward;
    steeringVoltage = steeringOutput + steeringFeedForward;

    setMotorVoltages(driveVoltage, steeringVoltage);
  }

  /**
   * Sets the drive and steering motor voltages.
   *
   * @param driveVoltage The drive motor voltage.
   * @param steeringVoltage The steering motor voltage.
   */
  public void setMotorVoltages(double driveVoltage, double steeringVoltage) {
    this.driveMotor.setVoltage(driveVoltage);
    this.steeringMotor.setVoltage(steeringVoltage);
    this.driveVoltageLog.append(driveVoltage);
    this.steeringVoltageLog.append(steeringVoltage);
  }

  /** Stops the drive and steering motors. */
  public void disable() {
    driveMotor.disable();
    steeringMotor.disable();
  }

  /**
   * Returns the current module state describing the wheel velocity and angle.
   *
   * @return The current module state.
   */
  public SwerveModuleVelocity getState() {
    return state;
  }

  /**
   * The position of the wheel on its axis of travel.
   *
   * @return The position of the swerve module.
   */
  public SwerveModulePosition getPosition() {
    return position;
  }

  /**
   * Returns the current wheel orientation.
   *
   * @return The current wheel orientation.
   */
  public Rotation2d getWheelRotation2d() {
    return position.angle;
  }

  /**
   * This method is called periodically by the {@link Swerve}. It is used to update module-specific
   * state.
   */
  public void periodic() {
    updateSuppliedState();
  }

  /** Returns the wheel angle in degrees. */
  @DashboardRadialGauge(
      title = "Wheel Angle (deg)",
      startAngle = -180,
      endAngle = 180,
      min = -180,
      max = 180,
      numberOfLabels = 0,
      wrapValue = true)
  public double getAngleDegrees() {
    return getState().angle.getDegrees();
  }

  /** Returns the wheel velocity in meters per second. */
  @DashboardTextDisplay(title = "Wheel Velocity (m/s)")
  public double getVelocity() {
    return getState().velocity;
  }
}
