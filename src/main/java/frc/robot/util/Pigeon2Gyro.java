/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Angle;

/** A gyro implementation based on the Pigeon 2. */
public final class Pigeon2Gyro implements Gyro {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;

  public Pigeon2Gyro(int canID) {
    pigeon = new Pigeon2(canID, new CANBus(CANPort.CAN_D0));
    yaw = pigeon.getYaw();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
  }

  @Override
  public double getYaw() {
    return Math.toRadians(yaw.refresh().getValueAsDouble());
  }

  @Override
  public double getPitch() {
    return Math.toRadians(pitch.refresh().getValueAsDouble());
  }

  @Override
  public double getRoll() {
    return Math.toRadians(roll.refresh().getValueAsDouble());
  }

  @Override
  public void reset() {
    pigeon.reset();
  }

  @Override
  public void close() throws Exception {
    pigeon.close();
  }
}
