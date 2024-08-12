package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveSubsystem extends SubsystemBase {
    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftTurningMotorReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            "Front Left");

    public final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightTurningMotorReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            "Front Right");

    public final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftTurningMotorReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            "Back Left");

    public final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightTurningMotorReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            "Back Right");

            Joystick driverJoytick = RobotContainer.getDriverJoytick();

            static SwerveDriveKinematics k = Constants.DriveConstants.kDriveKinematics;

    // AHRS is NavX gryo module
    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), 
            getModulePositions());

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

//PathPlanner Test Code Start

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::swerveDriveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );

//PathPlanner Test Code End

    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return ((DriveConstants.kGyroInverted?-1.0:1.0) * Math.IEEEremainder(gyro.getAngle(), 360)) + DriveConstants.kGyroOffset;
    }

  
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] positions = {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };

        return positions;

    }

    public void resetOdometry(Pose2d pose) {  //resetPose() on other swerve codes
        odometer.resetPosition(getRotation2d(), getModulePositions(),pose);
        }
    

    @Override
    public void periodic() {

        //drives swerve modules by updating values
        swerveDrive();

        //useful data outputted to SmartDashboard
//TODO - make sure to comment out stuff if we don't need it any more. We need stuff on SmartDashboard to be as concise as possible to help drive the robot
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Back Right angle", (Units.radiansToDegrees(backRight.getAbsoluteEncoderRad())));
        SmartDashboard.putNumber("Back Left angle", (Units.radiansToDegrees(backLeft.getAbsoluteEncoderRad())));
        SmartDashboard.putNumber("Front Right angle", (Units.radiansToDegrees(frontRight.getAbsoluteEncoderRad())));
        SmartDashboard.putNumber("Front Left angle", (Units.radiansToDegrees(frontLeft.getAbsoluteEncoderRad())));

        SmartDashboard.putString("Back Right position", (backRight.getPosition().toString()));
        SmartDashboard.putString("Back Left position", (backLeft.getPosition().toString()));
        SmartDashboard.putString("Front Right position", (frontRight.getPosition().toString()));
        SmartDashboard.putString("Front Left position", (frontLeft.getPosition().toString()));

        //Raw encoder values with no offset
        SmartDashboard.putNumber("Raw Back Right angle", (Units.radiansToDegrees(backRight.getRawEncoderValue())));
        SmartDashboard.putNumber("Raw Back Left angle", (Units.radiansToDegrees(backLeft.getRawEncoderValue())));
        SmartDashboard.putNumber("Raw Front Right angle", (Units.radiansToDegrees(frontRight.getRawEncoderValue())));
        SmartDashboard.putNumber("Raw Front Left angle", (Units.radiansToDegrees(frontLeft.getRawEncoderValue())));

        SmartDashboard.putNumber("X Axis", driverJoytick.getX());
        SmartDashboard.putNumber("Y Axis", driverJoytick.getY());
        SmartDashboard.putNumber("Twist Axis", driverJoytick.getTwist());

        SmartDashboard.putNumber("Gyro Heading", getHeading());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
        System.out.println("Swerve STOP done.");
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[]{
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

//TODO - test getRobotRelativeSpeeds() and drive() , these are supposedly needed for PathPlanner to work
//these may not even be needed, we'll have to see

    public ChassisSpeeds getRobotRelativeSpeeds() {  //returns current robot-relative chassisSpeeds
        // SwerveDriveKinematics k = new SwerveDriveKinematics();
        // ChassisSpeeds speeds = k.toChassisSpeeds(getModuleStates());
        // return speeds;
        SwerveModuleState[] states = new SwerveModuleState[4];
        states = getModuleStates();
        ChassisSpeeds speeds = k.toChassisSpeeds(states[0],states[1],states[2],states[3]);
        return speeds;
    }

    public void swerveDrive() {  //same drive function that was in periodic() before, just moved it to its own method
        odometer.update(getRotation2d(), getModulePositions());
    }

    public void swerveDriveRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states = k.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }
}
