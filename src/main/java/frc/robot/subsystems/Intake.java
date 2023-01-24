package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
   PneumaticHub m_pH=new PneumaticHub(5);
    DoubleSolenoid IntakeSoli = m_pH.makeDoubleSolenoid(14, 15);

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
