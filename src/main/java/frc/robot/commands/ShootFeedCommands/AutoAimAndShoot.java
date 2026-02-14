// package frc.robot.commands.ShootFeedCommands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.LimelightHelpers;
// import frc.robot.ShotCompensation;
// import frc.robot.ShootingLookupTable;
// import frc.robot.Constants.AimConstants;
// import frc.robot.Constants.LimelightConstants;
// import frc.robot.Constants.MotorConstants;
// import frc.robot.Constants.ShootingConstants;
// import frc.robot.Constants.SwerveConstants;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.shooterSystem.Feeder;
// import frc.robot.subsystems.shooterSystem.Flywheel;
// import frc.robot.subsystems.shooterSystem.TopRoller;
// import frc.robot.subsystems.shooterSystem.Spindexer;

// /**
//  * Auto-aim and auto-shoot command using Limelight AprilTag tracking.
//  * When active (driver holds RT):
//  * - Reads Limelight for target AprilTag
//  * - Auto-rotates toward target (driver retains translational control)
//  * - Adjusts hood angle and flywheel RPM based on distance
//  * - Auto-feeds when all conditions are met (aimed, hood ready, flywheel ready)
//  */
// public class AutoAimAndShoot extends Command {
//     private final Swerve m_swerve;
//     private final Flywheel m_flywheel;
//     private final TopRoller m_hood;
//     private final Feeder m_feeder;
//     private final Spindexer m_spindexer;

//     private final DoubleSupplier m_translationXSupplier; // Forward/back (field-relative Y on joystick)
//     private final DoubleSupplier m_translationYSupplier; // Left/right (field-relative X on joystick)

//     private final PIDController m_aimPID;

//     private double m_lastTargetRPM = MotorConstants.kShooterTargetRPM;
//     private boolean m_feeding = false;

//     public AutoAimAndShoot(
//             Swerve swerve, Flywheel flywheel, TopRoller hood, Feeder feeder, Spindexer spindexer,
//             DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
//         m_swerve = swerve;
//         m_flywheel = flywheel;
//         m_hood = hood;
//         m_feeder = feeder;
//         m_spindexer = spindexer;
//         m_translationXSupplier = translationXSupplier;
//         m_translationYSupplier = translationYSupplier;

//         m_aimPID = new PIDController(AimConstants.kAimP, AimConstants.kAimI, AimConstants.kAimD);
//         m_aimPID.setTolerance(AimConstants.kAimToleranceDegrees);
//         m_aimPID.setSetpoint(0.0); // Target: zero tx offset

//         addRequirements(swerve, flywheel, hood, feeder, spindexer);
//     }

//     @Override
//     public void initialize() {
//         // Set Limelight to AprilTag pipeline
//         LimelightHelpers.setPipelineIndex(LimelightConstants.kLimelightName, LimelightConstants.kAprilTagPipeline);

//         // Filter to only track scoring target AprilTags
//         LimelightHelpers.SetFiducialIDFiltersOverride(
//                 LimelightConstants.kLimelightName, LimelightConstants.kTargetAprilTagIDs);

//         m_aimPID.reset();
//         m_feeding = false;

//         SmartDashboard.putBoolean("AutoAim Active", true);
//     }

//     @Override
//     public void execute() {
//         String ll = LimelightConstants.kLimelightName;

//         // Read Limelight data
//         boolean hasTarget = LimelightHelpers.getTV(ll);
//         double tx = LimelightHelpers.getTX(ll);
//         double ty = LimelightHelpers.getTY(ll);
//         double detectedID = LimelightHelpers.getFiducialID(ll);

//         // Get driver translation input, scale to m/s
//         double translationX = m_translationXSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
//         double translationY = m_translationYSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
//         Translation2d translation = new Translation2d(translationX, translationY);

//         // Validate that detected tag is in our valid list
//         boolean validTarget = hasTarget && LimelightConstants.isValidTagID((int) detectedID);

//         if (validTarget) {
//             // Calculate distance using trigonometry
//             double angleToTargetRad = Math.toRadians(LimelightConstants.kLimelightMountAngleDegrees + ty);
//             double distance = (LimelightConstants.kTargetHeightMeters - LimelightConstants.kLimelightMountHeightMeters)
//                     / Math.tan(angleToTargetRad);

//             // Clamp distance to valid range
//             distance = MathUtil.clamp(distance,
//                     ShootingConstants.kMinShootingDistanceMeters,
//                     ShootingConstants.kMaxShootingDistanceMeters);

//             // Velocity compensation for shoot-while-moving
//             ChassisSpeeds fieldVelocity = m_swerve.getFieldVelocity();
//             double headingDeg = m_swerve.getHeading().getDegrees();
//             ShotCompensation.CompensationResult compensation =
//                     ShotCompensation.calculate(fieldVelocity, headingDeg, tx, distance);

//             // Use compensated distance for hood/RPM lookups, clamped to valid range
//             double compensatedDistance = MathUtil.clamp(compensation.adjustedDistanceMeters,
//                     ShootingConstants.kMinShootingDistanceMeters,
//                     ShootingConstants.kMaxShootingDistanceMeters);

//             // Look up hood angle and RPM from compensated distance
//             double targetHoodAngle = ShootingLookupTable.getHoodAngle(compensatedDistance);
//             double targetRPM = ShootingLookupTable.getFlywheelRPM(compensatedDistance);
//             m_lastTargetRPM = targetRPM;

//             m_hood.setHoodAngle(targetHoodAngle);
//             m_flywheel.setFlywheelVelocity(targetRPM);

//             // Calculate auto-rotation from aim PID (tx -> rad/s)
//             // Setpoint is the aim lead angle (0 when stationary, offset when moving)
//             double aimSetpoint = compensation.aimLeadDegrees;
//             double aimOutput = m_aimPID.calculate(tx, aimSetpoint);
//             double rotationSpeed = MathUtil.clamp(aimOutput,
//                     -AimConstants.kMaxAutoRotationRadPerSec,
//                     AimConstants.kMaxAutoRotationRadPerSec);

//             // Drive: driver translation + auto rotation, field-relative
//             m_swerve.drive(translation, rotationSpeed, true);

//             // Check if all conditions are met (aimed = reached lead angle, not necessarily centered)
//             boolean aimed = Math.abs(tx - aimSetpoint) < AimConstants.kAimToleranceDegrees;
//             boolean hoodReady = m_hood.isAtTargetAngle(targetHoodAngle);
//             boolean flywheelReady = m_flywheel.isAtTargetSpeed(targetRPM);
//             boolean readyToFire = aimed && hoodReady && flywheelReady;

//             if (readyToFire) {
//                 // Auto-feed
//                 m_feeder.setSpeed(MotorConstants.kFeederSpeed);
//                 m_spindexer.setSpeed(MotorConstants.kSpindexerSpeed);
//                 m_feeding = true;
//             } else {
//                 m_feeder.stopFeederMotor();
//                 m_spindexer.stopSpindexerMotor();
//                 m_feeding = false;
//             }

//             // Telemetry
//             SmartDashboard.putNumber("AutoAim Distance", distance);
//             SmartDashboard.putNumber("AutoAim Target Hood", targetHoodAngle);
//             SmartDashboard.putNumber("AutoAim Target RPM", targetRPM);
//             SmartDashboard.putBoolean("AutoAim Aimed", aimed);
//             SmartDashboard.putBoolean("AutoAim HoodReady", hoodReady);
//             SmartDashboard.putBoolean("AutoAim FlywheelReady", flywheelReady);
//             SmartDashboard.putBoolean("AutoAim ReadyToFire", readyToFire);

//             // Velocity compensation telemetry
//             double robotSpeed = Math.hypot(fieldVelocity.vxMetersPerSecond,
//                     fieldVelocity.vyMetersPerSecond);
//             SmartDashboard.putBoolean("VComp Active", compensation.compensationActive);
//             SmartDashboard.putNumber("VComp Aim Lead (deg)", compensation.aimLeadDegrees);
//             SmartDashboard.putNumber("VComp Adjusted Dist", compensation.adjustedDistanceMeters);
//             SmartDashboard.putNumber("VComp Raw Dist", distance);
//             SmartDashboard.putNumber("VComp Flight Time (s)", compensation.flightTimeSec);
//             SmartDashboard.putNumber("VComp Lateral V (m/s)", compensation.lateralVelocityMps);
//             SmartDashboard.putNumber("VComp Radial V (m/s)", compensation.radialVelocityMps);
//             SmartDashboard.putNumber("VComp Robot Speed (m/s)", robotSpeed);
//         } else {
//             // No valid target: drive with zero rotation, keep flywheel spinning at last RPM
//             m_swerve.drive(translation, 0.0, true);
//             m_flywheel.setFlywheelVelocity(m_lastTargetRPM);
//             m_feeder.stopFeederMotor();
//             m_spindexer.stopSpindexerMotor();
//             m_feeding = false;

//             SmartDashboard.putBoolean("AutoAim Aimed", false);
//             SmartDashboard.putBoolean("AutoAim ReadyToFire", false);
//             clearVCompTelemetry();
//         }

//         // Common telemetry
//         SmartDashboard.putBoolean("AutoAim HasTarget", validTarget);
//         SmartDashboard.putNumber("AutoAim TX", tx);
//         SmartDashboard.putNumber("AutoAim TY", ty);
//         SmartDashboard.putBoolean("AutoAim Feeding", m_feeding);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         m_flywheel.stopFlywheelMotor();
//         m_hood.stopRollerMotor();
//         m_feeder.stopFeederMotor();
//         m_spindexer.stopSpindexerMotor();
//         // Swerve default command auto-resumes

//         SmartDashboard.putBoolean("AutoAim Active", false);
//         SmartDashboard.putBoolean("AutoAim Feeding", false);
//         SmartDashboard.putBoolean("AutoAim ReadyToFire", false);
//     }

//     @Override
//     public boolean isFinished() {
//         return false; // Runs until interrupted (button release or autonomous deadline)
//     }

//     /**
//      * Check if the command is currently feeding a game piece to the shooter.
//      * Used by autonomous compositions to detect when shooting is in progress.
//      */
//     public boolean isFeedingActive() {
//         return m_feeding;
//     }

//     /** Zeros all velocity-compensation telemetry when no target is visible. */
//     private void clearVCompTelemetry() {
//         SmartDashboard.putBoolean("VComp Active", false);
//         SmartDashboard.putNumber("VComp Aim Lead (deg)", 0.0);
//         SmartDashboard.putNumber("VComp Adjusted Dist", 0.0);
//         SmartDashboard.putNumber("VComp Raw Dist", 0.0);
//         SmartDashboard.putNumber("VComp Flight Time (s)", 0.0);
//         SmartDashboard.putNumber("VComp Lateral V (m/s)", 0.0);
//         SmartDashboard.putNumber("VComp Radial V (m/s)", 0.0);
//         SmartDashboard.putNumber("VComp Robot Speed (m/s)", 0.0);
//     }
// }
