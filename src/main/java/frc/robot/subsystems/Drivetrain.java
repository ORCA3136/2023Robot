
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import javax.lang.model.util.ElementScanner6;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Drivetrain extends SubsystemBase {
    
private DifferentialDrive diffDrive;

private RelativeEncoder leftEncoder;
private RelativeEncoder rightEncoder;

private SparkMaxPIDController leftController;
private SparkMaxPIDController rightController;

private CANSparkMax leftLeader;
private CANSparkMax leftFollower;

private CANSparkMax rightLeader;
private CANSparkMax rightFollower;

private double afterEncoderReduction = 6.0; // Internal encoders

private RelativeEncoder leftInternalEncoder;
private RelativeEncoder rightInternalEncoder;

public void DRIVE(){
    
    leftLeader = new CANSparkMax(Constants.leaderLeftCAN, MotorType.kBrushless);
    leftFollower = new CANSparkMax(Constants.followerLeftCAN, MotorType.kBrushless);

    rightLeader = new CANSparkMax(Constants.leaderRightCAN, MotorType.kBrushless);
    rightFollower = new CANSparkMax(Constants.followerRightCAN, MotorType.kBrushless);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

}

public void driveForward(){
    leftLeader.set(Constants.kTestSpeed);
    rightLeader.set(-1 * Constants.kTestSpeed);
}

public void driveReverse(){
    leftLeader.set(-1 * Constants.kTestSpeed);
    rightLeader.set(Constants.kTestSpeed);
}

public void driveStop(){
    leftLeader.set(0);
    rightLeader.set(0);
}

}
