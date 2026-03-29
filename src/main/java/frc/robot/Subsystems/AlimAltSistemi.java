package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.ModulSabitleri;
import frc.robot.Sabitler.MotorSabitleri;

public class AlimAltSistemi extends SubsystemBase {
    private PWMSparkMax alimCimMotoru;
    private PWMSparkMax yukariTasiyiciCimMotoru;
    private double sonAlimHizi = 0.0;
    private double sonTasiyiciHizi = 0.0;

    public AlimAltSistemi() {
        // Shuffleboard'da Ayarlama tab'inda gorunur
        SmartDashboard.setDefaultNumber("Ayarlama/AlimHizi", ModulSabitleri.ALIM_HIZI);
        SmartDashboard.setDefaultNumber("Ayarlama/TasiyiciHizi", ModulSabitleri.DEPO_ATICI_YUKARI_TASIYICI_HIZI);

        if (MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN) {
            alimCimMotoru = new PWMSparkMax(MotorSabitleri.ALIM_CIM_PWM_KANALI);
            yukariTasiyiciCimMotoru = new PWMSparkMax(MotorSabitleri.DEPO_ATICI_YUKARI_TASIYICI_CIM_PWM_KANALI);
            alimCimMotoru.setInverted(MotorSabitleri.ALIM_MOTOR_TERS);
            yukariTasiyiciCimMotoru.setInverted(MotorSabitleri.DEPO_ATICI_YUKARI_TASIYICI_TERS);
        } else {
            alimCimMotoru = null;
            yukariTasiyiciCimMotoru = null;
        }
    }

    public void al() {
        double hiz = SmartDashboard.getNumber("Ayarlama/AlimHizi", ModulSabitleri.ALIM_HIZI);
        sonAlimHizi = hiz;
        if (alimCimMotoru != null) {
            alimCimMotoru.set(hiz);
        }
    }

    public void geriAt() {
        double hiz = SmartDashboard.getNumber("Ayarlama/AlimHizi", ModulSabitleri.ALIM_HIZI);
        sonAlimHizi = -hiz;
        if (alimCimMotoru != null) {
            alimCimMotoru.set(-hiz);
        }
    }

    public void depodanAticiyaYukariTasimaBaslat() {
        double hiz = SmartDashboard.getNumber("Ayarlama/TasiyiciHizi", ModulSabitleri.DEPO_ATICI_YUKARI_TASIYICI_HIZI);
        sonTasiyiciHizi = hiz;
        if (yukariTasiyiciCimMotoru != null) {
            yukariTasiyiciCimMotoru.set(hiz);
        }
    }

    public void depodanAticiyaYukariTasimaTersBaslat() {
        double hiz = SmartDashboard.getNumber("Ayarlama/TasiyiciHizi", ModulSabitleri.DEPO_ATICI_YUKARI_TASIYICI_HIZI);
        sonTasiyiciHizi = -hiz;
        if (yukariTasiyiciCimMotoru != null) {
            yukariTasiyiciCimMotoru.set(-hiz);
        }
    }

    public void depodanAticiyaYukariTasimaDurdur() {
        sonTasiyiciHizi = 0.0;
        if (yukariTasiyiciCimMotoru != null) {
            yukariTasiyiciCimMotoru.set(0.0);
        }
    }

    public void durdur() {
        sonAlimHizi = 0.0;
        sonTasiyiciHizi = 0.0;
        if (alimCimMotoru != null) {
            alimCimMotoru.set(0.0);
        }
        if (yukariTasiyiciCimMotoru != null) {
            yukariTasiyiciCimMotoru.set(0.0);
        }
    }

    public double getSonAlimHizi() {
        return sonAlimHizi;
    }

    public double getSonTasiyiciHizi() {
        return sonTasiyiciHizi;
    }

    /** PWM kanallarini serbest birakir — test ortaminda @AfterAll ile cagrilmali */
    public void close() {
        if (alimCimMotoru != null) {
            alimCimMotoru.close();
            alimCimMotoru = null;
        }
        if (yukariTasiyiciCimMotoru != null) {
            yukariTasiyiciCimMotoru.close();
            yukariTasiyiciCimMotoru = null;
        }
    }
}
