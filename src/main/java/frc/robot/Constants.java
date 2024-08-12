package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class ModuleConstants {
        //The diameter of the wheel on your swerve drive
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.94); 
        //The gear ratio for an SDS MK4 module with speed ratio of L2
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        //Ratio for an SDS MK4 turning motor
        public static final double kTurningMotorGearRatio = 1 / 12.80;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        //The P value for the turning PID loop
        public static final double kPTurning = 0.2;
    }

    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(22);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(22);


        // Measured from the center of the robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),    /*FL*/
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),   /*FR*/
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),   /*BL*/
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); /*BR*/

        //CAN IDs For the drive motor spark maxes on the swerve modules
        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 5;
        public static final int kBackLeftDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 7;

        //CAN IDs for the turning motor spark maxes on the swerve modules
        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kFrontRightTurningMotorPort = 6;
        public static final int kBackLeftTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 8;

        //A boolean to control the inversion of the direction of the motor gievn a positive value
        //Positive values muest result in a counter-clockwise movment
        //Note: if you are using the Mk4i (Inverted) Swerve Modules, you may need to change this to "true"
        //Hint: if there is a big oscillation when you turn, try inverting these.
        public static final boolean kFrontLeftTurningMotorReversed = false;
        public static final boolean kFrontRightTurningMotorReversed = false;
        public static final boolean kBackLeftTurningMotorReversed = false;
        public static final boolean kBackRightTurningMotorReversed = false;

        //A boolean to control the inversion of the direction of the motor gievn a positive value
        //Positive values must result in a forward movement
        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = true;
        public static final boolean kBackLeftDriveMotorReversed = true;
        public static final boolean kBackRightDriveMotorReversed = true;

        //The CAN id's for the CANcoder's on the swerve modules
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        //Inversion of the direction of the CANcoder
        //Positive values must result in a counter-clockwise movement
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //Position must be counter-clockwise from the positive YAw
        public static final boolean kGyroInverted = true;

        //The offset of the CANcoder's position from the zero position (Straight forward)
        //Measure this by rotating all the modules to the forward position and reading the CANcoder's value
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(77.1);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(-149.45);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(-125.4);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(107.7);

        // in m/s, based on MK4 L2 speed of 14.5 ft/s
        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(14.5);  
        // Robot turning speed
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 5 * Math.PI;

        // If you want to slow down the robot during TeleOp, adjust these values
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;  // Slowed down for testing
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        //gyro offset in degrees (in case NavX is facing different direction on robot)
        public static double kGyroOffset = 0;
    }

    public static final class AutoConstants {
        // If you want to slow down the robot during Autonomous, adjust these values
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond /2; 
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
        // The P value of the PID controller used in auto for the X and Y directions
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        // The P value of the PID controller used in auto for the theta (rotation) direction
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        // Port for the driver's controller (Thrust Master)
        public static final int kDriverControllerPort = 0;
        // Axis used for the X, Y, Rotation, and Throttle
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;
        public static final int kDriverThrottleAxis = 3;
        // Deadband for the controller
        public static final double kRotDeadband = .25; //0.05;
        public static final double kDriveDeadband = .05;
        // Button used to enable robot orientation driving
        // public static final int kDriverFieldOrientedButtonIdx = 5;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        // Button used to enable slow turning
        // public static final int kDriverSlowTurnButtonIdx=1;
        public static final int kDriverSlowTurnButtonIdx=5;
        // Button used to reset the gyro to 0
        public static final int kDriverResetGyroButtonIdx=2;


    }
}
