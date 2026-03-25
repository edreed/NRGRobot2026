/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/** A motor controller implementation based on the REV Robotics {@link SparkMax} controllers. */
public class SparkMaxAdapter extends SparkAdapter {
  /**
   * Constructs a SparkMaxAdapter.
   *
   * @param logPrefix The prefix for the log entries.
   * @param deviceID The device ID of the SparkMax motor controller.
   * @param direction The direction the motor rotates when a positive voltage is applied.
   * @param idleMode The motor behavior when idle (i.e. brake or coast mode).
   * @param distancePerRotation The distance the attached mechanism moves per rotation of the motor
   *     output shaft.
   *     <p>The unit of measure depends on the mechanism. For a mechanism that produces linear
   *     motion, the unit is typically in meters. For a mechanism that produces rotational motion,
   *     the unit is typically in radians.
   */
  public SparkMaxAdapter(
      String logPrefix,
      int deviceID,
      MotorDirection direction,
      MotorIdleMode idleMode,
      double distancePerRotation) {
    super(
        logPrefix,
        new SparkMax(deviceID, MotorType.kBrushless),
        direction,
        idleMode,
        distancePerRotation);
  }
}
