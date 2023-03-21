package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;

public class ClawSubsystem extends SubsystemBase {

  public static Compressor Compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  public static DoubleSolenoid claw_solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      ClawConstants.claw_solenoid1_port_ileri, ClawConstants.claw_solenoid1_port_geri);
  public ClawSubsystem() {

  }

  @Override
  public void periodic() {

  }

  public void setDash() {
    SmartDashboard.putBoolean("Claw Piston", claw_solenoid1.isFwdSolenoidDisabled());
  }

  public static void claw_open() {
    claw_solenoid1.set(Value.kForward);
  }

  public static void claw_close() {
    claw_solenoid1.set(Value.kReverse);
  }


}
