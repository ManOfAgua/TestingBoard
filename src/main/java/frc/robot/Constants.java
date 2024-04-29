// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Meters;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class ControllerConstants { // for playstation 5 controller
    public static final int driver = 0,
                    operator = 1,
                    b_SQR = 1,
                    b_X = 2,
                    b_O = 3,
                    b_TRI = 4,
                    b_L1 = 5,
                    b_R1 = 6,
                    b_L2 = 7,
                    b_R2 = 8,
                    b_PIC = 9,
                    b_MEN = 10,
                    b_LJOY = 11,
                    b_RJOY = 12,
                    b_LOG = 13,
                    b_PAD = 14,
                    b_MIC = 15;
}

     public static class VisionConstants {
                /**
                 * Array of PhotonVision camera names. The values here match
                 * ROBOT_TO_CAMERA_TRANSFORMS for the camera's location.
                 */
                public static final String[] APRILTAG_CAMERA_NAME = { "PhotonCamera" };

                /**
                 * Physical location of the apriltag cameras on the robot, relative to the
                 * center of the robot.
                 * The values here math APRILTAG_CAMERA_NAME for the camera's name.
                 */
                public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = {
                                new Transform3d(
                                                new Translation3d(inchesToMeters(14), inchesToMeters(14),
                                                                inchesToMeters(7.75)),
                                                new Rotation3d(0, degreesToRadians(0), degreesToRadians(0))),
                                new Transform3d(
                                                new Translation3d(inchesToMeters(21.5), inchesToMeters(8),
                                                                inchesToMeters(10.5)),
                                                new Rotation3d(degreesToRadians(0), degreesToRadians(30),
                                                                degreesToRadians(0)))
                };

                public static final Measure<Distance> FIELD_LENGTH = Meters.of(16.54175);
                public static final Measure<Distance> FIELD_WIDTH = Meters.of(8.0137);

                /**
                 * Minimum target ambiguity. Targets with higher ambiguity will be discarded.
                 * Not appliable when multiple tags are
                 * in view in a single camera.
                 */
                public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.35;
        }
}
