# Robot Buton Referansi

## Driver Joystick (Port 0) - PS4 Controller

### Suruş Eksenleri

| PS4 Ekseni | Axis No | Java Sabiti | Fonksiyon |
|------------|---------|-------------|-----------|
| Sol Stick Y | 0 | `DRIVER_Y_AXIS` | Ileri/Geri suruş |
| Sol Stick X | 1 | `DRIVER_X_AXIS` | Strafe (Sag/Sol) |
| Sag Stick X | 2 | `DRIVER_Z_AXIS` | Donduruculuk (Z-axis, kodda -1 ile carpiliyor) |

**Not:** Robot odakli suruş kullanilmaktadir. NavX jiroskop suruş modunda kullanilmaz.

### Buton Atamalari

#### Buton 1 (Square): On Sol Motor Test (CAN ID 2)
```java
// Dosya: RobotContainer.java, configureBindings()
new JoystickButton(driverJoystick, 1)
    .whileTrue(new RunCommand(() -> driveSubsystem.runFrontLeftMotor(0.3), driveSubsystem));
```
**Kullanim:**
- On Sol NEO motorunu test eder
- CAN ID: 2
- Motor invert: FALSE
- Guc: %30

#### Buton 2 (Cross): Arka Sol Motor Test (CAN ID 3)
```java
// Dosya: RobotContainer.java, configureBindings()
new JoystickButton(driverJoystick, 2)
    .whileTrue(new RunCommand(() -> driveSubsystem.runRearLeftMotor(0.3), driveSubsystem));
```
**Kullanim:**
- Arka Sol NEO motorunu test eder
- CAN ID: 3
- Motor invert: FALSE
- Guc: %30

#### Buton 3 (Circle): Arka Sag Motor Test (CAN ID 1)
```java
// Dosya: RobotContainer.java, configureBindings()
new JoystickButton(driverJoystick, 3)
    .whileTrue(new RunCommand(() -> driveSubsystem.runRearRightMotor(0.3), driveSubsystem));
```
**Kullanim:**
- Arka Sag NEO motorunu test eder
- CAN ID: 1
- Motor invert: TRUE
- Guc: %30

#### Buton 4 (Triangle): On Sag Motor Test (CAN ID 4)
```java
// Dosya: RobotContainer.java, configureBindings()
new JoystickButton(driverJoystick, 4)
    .whileTrue(new RunCommand(() -> driveSubsystem.runFrontRightMotor(0.3), driveSubsystem));
```
**Kullanim:**
- On Sag NEO motorunu test eder
- CAN ID: 4
- Motor invert: TRUE
- Guc: %30

#### Buton 5 (L1): Vision Pozisyon Sifirla
```java
// Dosya: RobotContainer.java, configureBindings()
new JoystickButton(driverJoystick, 5)
    .onTrue(new InstantCommand(() -> driveSubsystem.resetPoseFromVision(), driveSubsystem));
```
**Kullanim:**
- AprilTag ile odometry sifirlar
- InstantCommand olarak calisir (bir kez tetiklenir)
- Limelight'in bir AprilTag gormesi gerekir

**Gereksinimler:**
- Limelight'in bir AprilTag gormesi (`Vision/HasTarget` = true)
- Pozisyonun gecerli olmasi (`Vision/PoseValid` = true)
- Ambiguity < 0.2

#### Buton 6 (R1): AlignToAprilTagCommand
```java
// Dosya: RobotContainer.java, configureBindings()
new JoystickButton(driverJoystick, 6)
    .whileTrue(new AlignToAprilTagCommand(driveSubsystem, visionSubsystem, 1, 1.0));
```
**Kullanim:**
- Basili tutuldugu surece Tag 1'e hizalanir
- Tag 1'in 1.0 metre onunde durur
- Birakinca durur

#### Buton 7 (L2): RotateToAprilTag360Command
```java
// Dosya: RobotContainer.java, configureBindings()
new JoystickButton(driverJoystick, 7)
    .onTrue(new RotateToAprilTag360Command(driveSubsystem, visionSubsystem, 1));
```
**Kullanim:**
- Tag 1'e bakacak sekilde bir kez doner
- Donuşu tamamlayip Tag yonunde kilitlenir
- Tek sefer tetiklenir

#### Buton 8 (R2): TrackAprilTagCommand
```java
// Dosya: RobotContainer.java, configureBindings()
new JoystickButton(driverJoystick, 8)
    .whileTrue(new TrackAprilTagCommand(driveSubsystem, visionSubsystem, 1, 1.0));
```
**Kullanim:**
- Basili tutuldugu surece Tag 1'i surekli takip eder
- 1.0 metre mesafe korur
- Birakinca durur

#### Buton 9-12: Atanmamis

---

## SmartDashboard Motor Test Toggle'lari

Disabled periodique'de DriveSubsystem tarafindan yonetilir.

### MotorTest/FrontLeft
- **Motor**: On Sol (CAN ID 2)
- **Invert**: FALSE
- **DISABLED modda**: + veya - ile test
- **Guc**: %20

### MotorTest/FrontRight
- **Motor**: On Sag (CAN ID 4)
- **Invert**: TRUE
- **DISABLED modda**: + veya - ile test
- **Guc**: %20

### MotorTest/RearLeft
- **Motor**: Arka Sol (CAN ID 3)
- **Invert**: FALSE
- **DISABLED modda**: + veya - ile test
- **Guc**: %20

### MotorTest/RearRight
- **Motor**: Arka Sag (CAN ID 1)
- **Invert**: TRUE
- **DISABLED modda**: + veya - ile test
- **Guc**: %20

---

## CAN ID Eslesmesi

Fiziksel CAN kablo baglantilari (sahada dogrulanmis):

| CAN ID | Pozisyon | Motor Invert |
|--------|----------|--------------|
| 1 | Arka Sag (Rear Right) | TRUE |
| 2 | On Sol (Front Left) | FALSE |
| 3 | Arka Sol (Rear Left) | FALSE |
| 4 | On Sag (Front Right) | TRUE |

---

## Motor Test Akisi

### Pit Alaninda Motor Testi

1. **Robot DISABLED modda olduguna emin olun**
2. **SmartDashboard'i acin**
3. **Motor Test toggle'larini gorun**
4. **Test motorunu secin** (ornegin MotorTest/FrontLeft)
5. **+ tusuna basin** (SmartDashboard'da)
6. **Motor donmeli** - %20 gucle
7. **Tekerlek yonunu kontrol edin**
8. **- tusuna basin** - Motor diger yone doner
9. **Tum motorlar icin tekrarlayin**

### Buton ile Motor Testi

```
1. Robot DISABLED
2. Buton 1 (Square) basili tut -> On Sol motor doner (CAN ID 2)
3. Buton 2 (Cross) basili tut  -> Arka Sol motor doner (CAN ID 3)
4. Buton 3 (Circle) basili tut -> Arka Sag motor doner (CAN ID 1)
5. Buton 4 (Triangle) basili tut -> On Sag motor doner (CAN ID 4)
6. Butonu birak -> Motor durur
```

---

## Vision Reset Detaylari

### Ne Zaman Kullanilmali?

Kullanin:
- Mac oncesi baslangic pozisyonu icin
- Odometry drift supehsi oldugunda
- Autonomous oncesi pozisyon dogrulama icin
- Pit alaninda vision testi icin

Kullanmayin:
- Mac ortasinda (eger pozisyonunuz dogruysa)
- AprilTag gorunmuyorken
- Robot hareket halindeyken

### Vision Reset Sonrasi Kontrol

SmartDashboard'da su degerleri kontrol edin:

1. **`Robot X`**: 0-16.54 arasi (saha boyunca)
2. **`Robot Y`**: 0-8.21 arasi (saha genisliginde)
3. **`Robot Heading`**: -180 ila 180 derece
4. **`Vision/TagID`**: Hangi tag kullanildi (1-16)
5. **`Drive/VisionUpdate`**: TRUE olmali

### Hata Mesajlari

| Mesaj | Anlami | Cozum |
|-------|--------|-------|
| "Pose reset from AprilTag X..." | Basarili | - |
| (Mesaj yok) | Reset basarisiz | Tag'i kontrol edin |
| "No valid vision result" | Tag gorunmuyor | Limelight'i cevirin |

---

## Guvenlik Notlari

### Test Gucleri

- **Buton testi**: %30 guc
- **Toggle testi**: %20 guc
- **Guvenli seviyeler** - patinaj yapmaz

### Test Sirasinda

1. Sadece DISABLED modda calistirin
2. Robot yerinde sabit olmali
3. Tekerleklerin cevresinde bosluk birakın
4. Gerekirse robotu yukari kaldirin

---

## Referanslar

- **Motor ID'leri**: Constants.java (MotorConstants class)
- **Buton baglari**: RobotContainer.java (configureBindings)
- **Vision kodu**: VisionSubsystem.java, DriveSubsystem.java
- **Komutlar**: Commands/ dizini (AlignToAprilTagCommand, RotateToAprilTag360Command, TrackAprilTagCommand)
