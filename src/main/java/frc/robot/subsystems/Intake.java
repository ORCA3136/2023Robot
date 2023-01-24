package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    DoubleSolenoid IntakeSoli = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 15);

    public void deployIntake() {
        IntakeSoli.set(DoubleSolenoid.Value.kForward);
      }
    
      public void retractIntake(){
        IntakeSoli.set(Value.kReverse);
      }
    
      public void off(){
        IntakeSoli.set(Value.kOff);
    }

}
