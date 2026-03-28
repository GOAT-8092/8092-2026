package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sabitler.MotorSabitleri;
import frc.robot.Subsystems.TaretAltSistemi;

/**
 * Tareti limit switch'e kadar döndürür, switch tetiklenince durur ve
 * enkoderi sıfırlar. Normally closed switch varsayılır.
 */
public class TaretHomingKomutu extends Command {
    private final TaretAltSistemi taretAltSistemi;

    public TaretHomingKomutu(TaretAltSistemi taretAltSistemi) {
        this.taretAltSistemi = taretAltSistemi;
        addRequirements(taretAltSistemi);
    }

    @Override
    public void initialize() {
        // Zaten switch üzerindeyse sıfırla, homing gerekmez
        if (taretAltSistemi.limitSwitchTetiklendi()) {
            taretAltSistemi.enkoderiSifirla();
        }
    }

    @Override
    public void execute() {
        if (!taretAltSistemi.limitSwitchTetiklendi()) {
            taretAltSistemi.dondur(MotorSabitleri.TARET_HOMING_HIZI);
        }
    }

    @Override
    public boolean isFinished() {
        return taretAltSistemi.limitSwitchTetiklendi();
    }

    @Override
    public void end(boolean interrupted) {
        taretAltSistemi.durdur();
        if (!interrupted) {
            taretAltSistemi.enkoderiSifirla();
        }
    }
}
