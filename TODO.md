# TODO.md — 2026 FRC Cross-Check Findings

> Kaynak: 2026 sezonu GitHub repoları, WPILib ornekleri, Chief Delphi, REV ornekleri ve Andymark dokumanları ile capraz kontrol.

---

## 1. KRITIK — Hemen Duzeltilmeli

### 1.1 NavX Yaw Negation Bug ✅ TAMAMLANDI
- **Dosya:** `SurusAltSistemi.java` (~satir 579, `getHeading()`)
- **Sorun:** NavX CW-pozitif, WPILib CCW-pozitif kullanir. `Rotation2d.fromDegrees(navx.getYaw())` negatif almadan donduruluyor.
- **Etki:** Alan-yonlu surus ve odometri ters yonde calisiyor.
- **Duzeltme:**
  ```java
  // Yanlis:
  return Rotation2d.fromDegrees(navx.getYaw());
  // Dogru:
  return Rotation2d.fromDegrees(-navx.getYaw());
  ```
- **Kaynak:** AdvantageKit resmi NavX entegrasyonu, Mechanical Advantage

### 1.2 REVLib ve SparkMax Firmware Guncellemesi ✅ TAMAMLANDI (REVLib; SparkMax firmware fiziksel)
- **Mevcut:** REVLib `2026.0.1`
- **Hedef:** REVLib `2026.0.5` + SparkMax firmware `spline-26.1.3`
- **Dosya:** `vendordeps/REVLib.json` ve REV Hardware Client
- **Neden:** 26.1.3 hiz ortalamasi, aci offseti geri yukleme ve LED yonuge hatalarini duzeltiyor.

---

## 2. YUKSEK — Performansi Etkiliyor

### 2.1 SparkMax IdleMode Eksik ✅ TAMAMLANDI
- **Dosya:** `SurusAltSistemi.java`, `AticiAltSistemi.java`, `TaretAltSistemi.java`
- **Sorun:** hicbir SparkMax konfigurasyonunda `idleMode` belirtilmemis.
- **Duzeltme:**
  - Surus motorleri → `IdleMode.kBrake` (aninda durma)
  - Atici motoru → `IdleMode.kCoast` (atak momentumuyla serbest donus)
  - Taret motoru → `IdleMode.kBrake` (pozisyon tutma)
  ```java
  reversedConfig.idleMode(IdleMode.kBrake);
  ```

### 2.2 Vision: MegaTag2 ve LimelightHelpers Gecisi ✅ TAMAMLANDI
- **Dosya:** `GorusAltSistemi.java`, `SurusAltSistemi.java`
- **Sorunlar:**
  - Ham `botpose_wpiblue` dizisi okunuyor (MegaTag1)
  - `SetRobotOrientation()` hic cagrilmiyor → MegaTag2 kullanilmiyor
  - `addVisionMeasurement()` standart sapma parametresiz cagriliyor
- **2026 Standart:** Tum rekabetci takimlar MegaTag2 kullaniyor (jiro destekli, daha kararli)
- **Duzeltme plani:**
  1. `LimelightHelpers` v1.12 ekle
  2. Her dongude `SetRobotOrientation(name, gyroYaw, ...)` cagir
  3. `getBotPoseEstimate_wpiBlue_MegaTag2()` kullan
  4. Standart sapmalari gec:
     ```java
     VecBuilder.fill(0.1, 0.1, 10.0)  // x, y std dev kucuk; theta std dev buyuk (jiro daha guvenilir)
     ```
- **Kaynak:** `wcpllc/2026CompetitiveConcept`, `frc1678/C2025-Public`

### 2.3 Vision Filtreleri Eksik ✅ TAMAMLANDI
- **Sorun:** Hicbir goruntu olcumu reddedilmiyor.
- **Eklenecek filtreler:**
  - **Tag sayisi:** 2+ tag VEYA 1 buyuk tag (tek kucuk tag reddet)
  - **Donus hizi:** Robot >360 deg/s donuyorsa olcumu reddet
  - **Mesafe olcekleme:** `avgTagDist` ile standart sapmalari carp (uzak tag = az guven)
- **Kaynak:** `FRCTeam3255/2025_Robot_Code`, `NewTechProgrammers/2026Rebuilt`

### 2.4 Atici: Velocity PID + Feedforward ✅ TAMAMLANDI
- **Dosya:** `AticiAltSistemi.java`
- **Mevcut:** `aticiMotoru.set(0.90)` — yuzde cikti, RPM geri bildirimi yok
- **Hedef:** SparkMax kapali dongu hiz kontrolü
- **Duzeltme:**
  ```java
  // SparkMaxConfig'a ekle:
  config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.0001)
      .i(0)
      .d(0)
      .outputRange(-1, 1)
      .feedForward.kV(12.0 / 5767);  // REV resmi kV degeri
  ;
  // Calistirma:
  closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  ```
- **Onemli:** `feedbackSensor` acikca belirtilmeli. `ControlType.kVelocity` kullanilmali (`kMAXVelocity` DEGIL).
- **Kaynak:** `REVrobotics/REVLib-Examples`, `Earl-Of-March-FRC/2026-7476-Rebuilt`, Chief Delphi FRC4327 konusu

---

## 3. ORTA — Kaliteyi Artirir

### 3.1 Taret: SparkMax Donanim Soft Limitleri ✅ TAMAMLANDI
- **Dosya:** `TaretAltSistemi.java`
- **Mevcut:** Yazilim seviyesinde aci sinirlama (`Math.min/max`)
- **Sorun:** RoboRIO kodu cokerse sinirlar kalkar
- **Duzeltme:** SparkMax `softLimit` kullan:
  ```java
  config.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit((float) degreesToMotorRotations(90))
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit((float) degreesToMotorRotations(-90));
  ```
- **Kaynak:** `wavelength3572/Robot-2026`

### 3.2 Taret: MAXMotion Trapezoidal Profil ✅ TAMAMLANDI
- **Mevcut:** Sadece P * hata kontrolü
- **Oneri:** SparkMax `ControlType.kMAXMotionPositionControl` ile cruise hiz/ivme sinirlari
- **Fayda:** Daha yumusak tareti hareket, daha az asinma
- **Kaynak:** `Nanuet-Knightronz/2026-Codebase`

### 3.3 Taret: Absolute Encoder ile Baslangic Referansi ⏭️ ATLANDI (donanım yok)
- **Mevcut:** Limit switch ile homing (her mac basi)
- **Oneri:** Baslangicta relative enkoderi absolute enkoderden seed et → homing gereksiz
- **Kaynak:** `wavelength3572/Robot-2026`, `Nanuet-Knightronz/2026-Codebase`

### 3.4 Taret: kI Terimi Eksik ✅ TAMAMLANDI
- **Mevcut:** `KP = 0.01`, kI = 0, kD = 0
- **Sorun:** Statik suretme hatasi olusabilir (kucuk hatada motor surtunmeyi yenemez)
- **Oneri:** Kucuk bir kI degeri ekle (ornegin 0.001)
- **Kaynak:** `wavelength3572/Robot-2026` TODO yorumu

### 3.5 Surus: Velocity PID + Feedforward (Otonom icin) ✅ TAMAMLANDI
- **Dosya:** `SurusAltSistemi.java`
- **Mevcut:** `motor.set(percent)` — batarya voltajina bagli, otonom yolu takip hatasi buyuk
- **Oneri:** `SimpleMotorFeedforward` + `PIDController` ile `setVoltage()` kullan
- **Alternatif:** SparkMax onboard velocity PID
- **Kaynak:** WPILib `mecanumbot` ornegi

### 3.6 Surus: PoseEstimator Standart Sapmalari ✅ TAMAMLANDI
- **Dosya:** `SurusAltSistemi.java`
- **Mevcut:** Default constructor (std dev parametresiz)
- **Oneri:**
  ```java
  new MecanumDrivePoseEstimator(
      kinematics, getHeading(), getWheelPositions(), initialPose,
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),  // state std devs
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))    // vision std devs
  );
  ```
- **Kaynak:** WPILib `mecanumdriveposeestimator` ornegi

### 3.7 Voltage Compensation ✅ TAMAMLANDI
- **Dosya:** Tum SparkMax konfigürasyonları
- **Oneri:** `.voltageCompensation(12)` ekle → batarya voltaj dususunda tutarli davranis
- **Kaynak:** REV resmi 2025 Starter Bot

---

## 4. DUSUK — Gelecek Planlama

### 4.1 SmartDashboard → Elastic Gecisi ✅ TAMAMLANDI
- **Sorun:** SmartDashboard 2027'de kaldirilacak (WPILib 2026 changelog)
- **Oneri:** Mevcut `Elastic.java` alt yapisini kullanarak kademeli gecis yap
- **Aciliyet:** 2026 sezonu icin zorunlu degil, 2027 oncesi planlanmali

### 4.2 Atici: Mesafe-RPM Polinom Egrisi
- **Oneri:** Farkli mesafeler icin hedef RPM belirleme
- **Ornek:** Team 7476 → `RPM = 43.5*d² + 119*d + 2372`
- **Kaynak:** `Earl-Of-March-FRC/2026-7476-Rebuilt`

### 4.3 Atis-Hareket Entegrasyonu (Shoot-While-Moving)
- **Oneri:** Robot hizi ve ivmesini hesaba katarak ongorulu atis
- **Kaynak:** `Earl-Of-March-FRC/2026-7476-Rebuilt` LaunchHelpers.java

### 4.4 Victor SPX / CIM PWM Notlari
- **Mevcut:** `PWMSparkMax` ile PWM kontrolu — dogru yaklasim
- **Not:** CTRE Phoenix kutuphanesi PWM Victor SPX icin gereksiz. Mevcut yapi dogru.

---

## 5. Mekanik Degisiklikler (Takip)

### 5.1 Robot Yonelimi 180° Takas ✅ TAMAMLANDI
- **Degisiklik:** Taret + Limelight on tarafa tasindi; alim arka tarafa alindi.
- **Kod etkisi:**
  - `SurusKomutu`: yanal eksen negasyonu kaldirildi (`hamY` artik terslenmez)
  - `Sabitler`: `TARET_ARKA_OFFSET_DERECE = 180.0` → `TARET_ON_OFFSET_DERECE = 0.0`
  - `PozTabanliTaretKomutu`: offset sabiti guncellendi
  - `TaretHomingKomutu`: yoni degismedi (-90° hala limit switch tarafi)
- **Otomatik taret devre disi:** Kablo guvenligi icin buton 8 (R2) iptal edildi.
  Yeniden etkinlestirmek icin `RobotKapsayici.java` 254. satira bak.

---

## Referans Repoları

| Repo | Konu | URL |
|---|---|---|
| WPILib MecanumBot | Resmi mecanum ornegi | `wpilibsuite/allwpilib` |
| WPILib MecanumPoseEstimator | Vision-fused odometry | `wpilibsuite/allwpilib` |
| REVLib-Examples | SparkMax velocity PID | `REVrobotics/REVLib-Examples` |
| wavelength3572/Robot-2026 | Turret (abs encoder + soft limit) | GitHub |
| Nanuet-Knightronz/2026-Codebase | Turret (MAXMotion) | GitHub |
| Earl-Of-March-FRC/2026-7476-Rebuilt | Tam atis sistemi | GitHub |
| wcpllc/2026CompetitiveConcept | MegaTag2 vision | GitHub |
| frc1678/C2025-Public | Mesafe-olcekli vision std dev | GitHub |
| FRCTeam3255/2026_Robot_Code | Cok-kamerali vision oncelik | GitHub |
| FRC-Team-3140/2026-Rebuilt-Bot | Turret + onguru atis | GitHub |
| FRC4048/Java_2026 | Cift-slot PID turret | GitHub |
| REV 2025 Starter Bot | SparkMaxConfig ornekleri | `REVrobotics/2025-REV-ION-FRC-Starter-Bot` |

---

*Son guncelleme: 2026-03-29*