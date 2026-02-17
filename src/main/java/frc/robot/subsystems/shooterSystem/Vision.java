package frc.robot.subsystems.shooterSystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class Vision extends SubsystemBase {
    private final String limelight = Constants.LIMELIGHT;
    public Vision() {

    }

    public double getDistanceFromClosestTarget() {
        return (LimelightHelpers.getTY(limelight)-26.6923)/(-.155218);
    }

    // public double getTarget() {
    //     LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelight);
    //     for (LimelightTarget_Fiducial target : results) {

    //     }
    //     double id = 
    // }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight/Distance from Target", getDistanceFromClosestTarget());
        SmartDashboard.putNumber("Limelight/TX", LimelightHelpers.getTX(limelight));
        SmartDashboard.putNumber("Limelight/TY", LimelightHelpers.getTY(limelight));
    }
}
