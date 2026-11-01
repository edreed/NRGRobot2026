/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import com.nrg948.preferences.EnumPreference;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.tunable.Selectable;

/**
 * AprilTag field layout selections for REBUILT.
 *
 * <p>The AprilTag location may vary depending on the field perimeter in use.
 *
 * <p>This enum can be used with a {@link Selectable} or {@link EnumPreference} to select the field
 * layout to match.
 */
public enum AprilTagFieldParameters {

  /** The field layout using the welded perimeter. */
  k2026RebuiltWelded(Fields.FRC_2026_REBUILT_WELDED),

  /** The field layout using the AndyMark perimeter. */
  k2026RebuiltAndymark(Fields.FRC_2026_REBUILT_ANDY_MARK);

  private Fields aprilTagField;

  /** Constructs a variant of this enum. */
  private AprilTagFieldParameters(Fields aprilTagField) {
    this.aprilTagField = aprilTagField;
  }

  /** Gets the {@link Fields} variant that can be used to load the correct field layout. */
  public Fields getAprilTagField() {
    return aprilTagField;
  }

  /** Gets the {@link Field} for this variant. */
  public Field loadAprilTagFieldLayout() {
    return Field.loadField(aprilTagField);
  }
}
