/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.MotorController;
import frc.robot.util.MotorDirection;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.NullMotorAdapter;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;
import java.util.Optional;
import org.ejml.simple.UnsupportedOperation;

/** A enum representing the properties of a specific motor type. */
public enum MotorParameters {
  /**
   * A null motor that can be used for testing or as a placeholder when no motor is needed.
   *
   * <p>It has the same properties as a bag motor, but does not actually control any hardware.
   */
  NullMotor(DCMotor.getBag(1)),

  /**
   * A VEX PRO <a href="https://www.vexrobotics.com/217-6515.html">Falcon 500</a> motor with
   * integrated Talon FX motor controller and encoders.
   */
  Falcon500(DCMotor.getFalcon500(1)),

  /**
   * A WestCoast Products <a href="https://docs.wcproducts.com/kraken-x60/kraken-x60-motor">Kraken
   * X60</a> motor with integrated Talon FX motor controller and encoders.
   */
  KrakenX60(DCMotor.getKrakenX60(1)),

  /**
   * A WestCoast Products <a
   * href="https://docs.wcproducts.com/welcome/electronics/kraken-x44">Kraken X44</a> motor with
   * integrated Talon FX motor controller and encoders.
   */
  KrakenX44(DCMotor.getKrakenX44(1)),

  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1650/">NEO Brushless Motor V1.1</a>
   * with integrated encoder.
   */
  NeoV1_1(DCMotor.getNEO(1)),

  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1652/">NEO Vortex Brushless
   * Motor</a> with <a href="https://www.revrobotics.com/rev-11-2158/">SparkMax</a> motor
   * controller.
   */
  NeoVortexMax(DCMotor.getNeoVortex(1)),

  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1652/">NEO Vortex Brushless
   * Motor</a> with <a href="https://www.revrobotics.com/rev-11-2159/">SparkFlex</a> motor
   * controller.
   */
  NeoVortexFlex(DCMotor.getNeoVortex(1)),

  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1651/">NEO 550 Brushless Motor</a>
   * with integrated encoder.
   */
  Neo550(DCMotor.getNeo550(1));

  private static final Optional<MethodHandle> talonFxAdapterConstructor;
  private static final Optional<MethodHandle> sparkMaxAdapterConstructor;
  private static final Optional<MethodHandle> sparkFlexAdapterConstructor;

  /**
   * Finds the constructor of the motor adapter class using reflection and returns a {@link
   * MethodHandle} to it.
   *
   * @param className The fully qualified name of the motor adapter class.
   * @return An {@link Optional} containing the {@link MethodHandle} if found, or an empty {@code
   *     Optional} if not.
   */
  private static final Optional<MethodHandle> findMotorAdapterConstructor(String className) {
    try {
      var lookup = MethodHandles.lookup();
      return Optional.of(
          lookup.findConstructor(
              Class.forName(className),
              MethodType.methodType(
                  void.class,
                  String.class,
                  int.class,
                  MotorDirection.class,
                  MotorIdleMode.class,
                  double.class)));
    } catch (NoSuchMethodException | IllegalAccessException | ClassNotFoundException e) {
      return Optional.empty();
    }
  }

  static {
    talonFxAdapterConstructor = findMotorAdapterConstructor("frc.robot.util.TalonFXAdapter");
    sparkMaxAdapterConstructor = findMotorAdapterConstructor("frc.robot.util.SparkMaxAdapter");
    sparkFlexAdapterConstructor = findMotorAdapterConstructor("frc.robot.util.SparkFlexAdapter");
  }

  private final DCMotor motor;

  /**
   * Constructs an instance of this enum.
   *
   * @param motor The motor properties.
   */
  MotorParameters(DCMotor motor) {
    this.motor = motor;
  }

  /**
   * @return the DCMotor.
   */
  public DCMotor getDCMotor() {
    return motor;
  }

  /**
   * Returns the free speed RPM of the motor.
   *
   * @return The free speed RPM.
   */
  public double getFreeSpeedRPM() {
    return this.motor.freeSpeedRadPerSec / (2 * Math.PI) * 60;
  }

  /**
   * @return The stall torque in Nm.
   */
  public double getStallTorque() {
    return this.motor.stallTorqueNewtonMeters;
  }

  /** Returns the voltage required to overcome the internal resistance of the motor. */
  public double getKs() {
    return this.motor.rOhms * this.motor.freeCurrentAmps;
  }

  /**
   * Returns a new {@link MotorController} implementation for this motor type.
   *
   * @param logPrefix The prefix for the log entries.
   * @param deviceID The CAN device ID.
   * @param direction The direction the motor rotates when a positive voltage is applied.
   * @param idleMode The motor behavior when idle (i.e. brake or coast mode).
   * @param distancePerRotation The distance the attached mechanism moves per rotation of the motor
   *     output shaft.
   *     <p>The unit of measure depends on the mechanism. For a mechanism that produces linear
   *     motion, the unit is typically in meters. For a mechanism that produces rotational motion,
   *     the unit is typically in radians.
   * @return A new MotorController implementation.
   */
  public MotorController newController(
      String logPrefix,
      int deviceID,
      MotorDirection direction,
      MotorIdleMode idleMode,
      double distancePerRotation) {
    switch (this) {
      case NullMotor:
        return new NullMotorAdapter();

      case Falcon500:
      case KrakenX44:
      case KrakenX60:
        return invokeConstructor(
            talonFxAdapterConstructor,
            logPrefix,
            deviceID,
            direction,
            idleMode,
            distancePerRotation);

      case NeoV1_1:
      case NeoVortexMax:
      case Neo550:
        return invokeConstructor(
            sparkMaxAdapterConstructor,
            logPrefix,
            deviceID,
            direction,
            idleMode,
            distancePerRotation);

      case NeoVortexFlex:
        return invokeConstructor(
            sparkFlexAdapterConstructor,
            logPrefix,
            deviceID,
            direction,
            idleMode,
            distancePerRotation);

      default:
        throw new UnsupportedOperation("Unknown Motor Type");
    }
  }

  /**
   * Invokes the constructor of the motor adapter class using the provided MethodHandle.
   *
   * @param constructor The MethodHandle of the constructor.
   * @param logPrefix The prefix for the log entries.
   * @param deviceID The CAN device ID.
   * @param direction The direction the motor rotates when a positive voltage is applied.
   * @param idleMode The motor behavior when idle (i.e. brake or coast mode).
   * @param distancePerRotation The distance the attached mechanism moves per rotation of the motor
   *     output shaft.
   * @return A new MotorController implementation.
   * @throws RuntimeException if the constructor cannot be invoked or if the motor adapter class is
   *     not found.
   */
  private MotorController invokeConstructor(
      Optional<MethodHandle> constructor,
      String logPrefix,
      int deviceID,
      MotorDirection direction,
      MotorIdleMode idleMode,
      double distancePerRotation) {
    return (MotorController)
        constructor
            .map(
                c -> {
                  try {
                    return c.invoke(logPrefix, deviceID, direction, idleMode, distancePerRotation);
                  } catch (Throwable e) {
                    throw new RuntimeException(e);
                  }
                })
            .orElseThrow(
                () ->
                    new RuntimeException(
                        new ClassNotFoundException("Motor adapter class not found")));
  }
}
