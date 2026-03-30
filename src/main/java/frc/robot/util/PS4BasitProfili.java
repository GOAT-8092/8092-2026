package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * PS4 controller profile where Axis 4/5 are not available.
 * Atış kontrolleri D-Pad ile organize edildi.
 *
 * Axis mapping:
 *   0 = Left X (strafe)
 *   1 = Left Y (forward/back)
 *   2 = Right X (turn)
 *
 * Button mapping:
 *   1 = Square    -> intake
 *   2 = Cross     -> reverse intake
 *   3 = Circle    -> conveyor reverse unjam (0.5 s)
 *   4 = Triangle  -> taşıyıcı (manuel)
 *   5 = L1        -> taret sola
 *   6 = R1        -> taret sağa
 *   7 = L2 digital -> turret homing
 *   8 = R2 digital -> (boş)
 *   9 = Share     -> gyro reset
 *   10 = Options  -> (boş)
 *   11 = PS       -> (boş)
 *   12 = Touchpad -> (boş)
 *
 * D-Pad (POV) mapping:
 *   0° (Up)    -> Yakın atış (~1.2m → 2750 RPM)
 *   90° (Right)-> Orta atış (~2.8m → 3700 RPM)
 *   180° (Down)-> Uzak atış (~4.4m → 4490 RPM)
 */
public class PS4BasitProfili extends KontrolcuProfili {

    private static final int SOL_X_EKSEN    = 0;
    private static final int SOL_Y_EKSEN    = 1;
    private static final int SAG_X_EKSEN    = 2;

    private static final int ALIM_BTN       = 1;   // Square
    private static final int GERI_AT_BTN    = 2;   // Cross
    private static final int UNJAM_BTN      = 3;   // Circle
    private static final int TASIYICI_BTN   = 4;   // Triangle
    private static final int TARET_SOL_BTN  = 5;   // L1
    private static final int TARET_SAG_BTN  = 6;   // R1
    private static final int HOMING_BTN     = 7;   // L2 digital
    private static final int BUTON_8        = 8;   // R2 digital (bos)
    private static final int GYRO_RESET_BTN = 9;   // Share
    private static final int BUTON_10       = 10;  // Options (bos)
    private static final int BUTON_11       = 11;  // PS (bos)
    private static final int BUTON_12       = 12;  // Touchpad (bos)

    // D-Pad POV açıları
    private static final int POV_UP    = 0;
    private static final int POV_RIGHT = 90;
    private static final int POV_DOWN  = 180;

    public PS4BasitProfili(GenericHID hid) {
        super(hid);
    }

    @Override public double yanal()     { return eksenGuvenliOku(SOL_X_EKSEN); }
    @Override public double ileriGeri() { return eksenGuvenliOku(SOL_Y_EKSEN); }
    @Override public double donus()     { return eksenGuvenliOku(SAG_X_EKSEN); }

    @Override public boolean yakinAtisBasili()   { return povOku() == POV_UP; }
    @Override public boolean ortaAtisBasili()    { return povOku() == POV_RIGHT; }
    @Override public boolean uzakAtisBasili()    { return povOku() == POV_DOWN; }

    @Override public boolean alimBasili()         { return dugmeGuvenliOku(ALIM_BTN); }
    @Override public boolean geriAtBasili()       { return dugmeGuvenliOku(GERI_AT_BTN); }
    @Override public boolean tasiyiciBasili()     { return dugmeGuvenliOku(TASIYICI_BTN); }
    @Override public boolean tasiyiciTersBasili() { return dugmeGuvenliOku(UNJAM_BTN); }

    @Override public boolean taretSolaBasili()   { return dugmeGuvenliOku(TARET_SOL_BTN); }
    @Override public boolean taretSagaBasili()   { return dugmeGuvenliOku(TARET_SAG_BTN); }
    @Override public boolean taretHomingBasili() { return dugmeGuvenliOku(HOMING_BTN); }

    @Override public boolean gyroSifirlaBasili() { return dugmeGuvenliOku(GYRO_RESET_BTN); }

    @Override public boolean gecikmeliAtisBasili() { return dugmeGuvenliOku(BUTON_10); }

    /** D-Pad (POV) açısını güvenli şekilde okur. Merkezde değilse -1 döner. */
    private int povOku() {
        int port = hid.getPort();
        if (!DriverStation.isJoystickConnected(port)) return -1;
        return hid.getPOV();
    }
}

