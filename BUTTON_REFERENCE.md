# Robot Buton Referansı

## Driver Joystick (Port 0) - PS4 Controller

### Sürüş Eksenleri

| PS4 Ekseni | Axis No | Java Sabiti | Fonksiyon |
|------------|---------|-------------|-----------|
| Sol Stick Y | 0 | `DRIVER_Y_AXIS` | İleri/Geri sürüş |
| Sol Stick X | 1 | `DRIVER_X_AXIS` | Strafe (Sağ/Sol) |
| Sağ Stick X | 2 | `DRIVER_Z_AXIS` | Döndürme (Z-axis) |

### Buton Atamaları

#### Button 1: Tüm Motorları Test
```java
// Dosya: RobotContainer.java, Satır 60
new JoystickButton(driverJoystick, 1)
    .whileTrue(new RunCommand(() -> driveSubsystem.runAllMotors(0.3), driveSubsystem));
```
**Kullanım:**
- ✅ DISABLED modda çalışır
- ✅ Tüm 4 motoru %30 güçle ileri çalıştırır
- ❌ ENABLED modda nothing

**Test Amaçları:**
- Motor bağlantılarını doğrulama
- CAN ID eşleşmesini kontrol etme
- Tüm motorların aynı yönde döndüğünü görme

#### Button 2: Arka Sol Motor Test (ID 4)
```java
// Dosya: RobotContainer.java, Satır 64
new JoystickButton(driverJoystick, 2)
    .whileTrue(new RunCommand(() -> driveSubsystem.runRearLeftMotor(0.3), driveSubsystem));
```
**Kullanım:**
- ✅ DISABLED modda çalışır
- Arka sol NEO motorunu test eder
- CAN ID: 4
- Motor invert: TRUE

#### Button 3: Arka Sağ Motor Test (ID 2)
```java
// Dosya: RobotContainer.java, Satır 68
new JoystickButton(driverJoystick, 3)
    .whileTrue(new RunCommand(() -> driveSubsystem.runRearRightMotor(0.3), driveSubsystem));
```
**Kullanım:**
- ✅ DISABLED modda çalışır
- Arka sağ NEO motorunu test eder
- CAN ID: 3
- Motor invert: FALSE

#### Button 4: Ön Sağ Motor Test (ID 3)
```java
// Dosya: RobotContainer.java, Satır 72
new JoystickButton(driverJoystick, 4)
    .whileTrue(new RunCommand(() -> driveSubsystem.runFrontRightMotor(0.3), driveSubsystem));
```
**Kullanım:**
- ✅ DISABLED modda çalışır
- Ön sağ NEO motorunu test eder
- CAN ID: 2
- Motor invert: FALSE

#### Button 5: Vision Pozisyon Reset
```java
// Dosya: RobotContainer.java, Satır 76
new JoystickButton(driverJoystick, 5)
    .onTrue(new RunCommand(() -> driveSubsystem.resetPoseFromVision(), driveSubsystem));
```
**Kullanım:**
- ✅ DISABLED modda çalışır
- ✅ ENABLED modda da çalışır
- AprilTag ile robot pozisyonunu resetler

**Nasıl Çalışır:**
1. VisionSubsystem'dan güncel AprilTag pozisyonunu alır
2. DriveSubsystem odometry'sini o pozisyona resetler
3. DriverStation'a mesaj yazar

**Gereksinimler:**
- Limelight'ın bir AprilTag görmesi (`Vision/HasTarget` = true)
- Pozisyonun geçerli olması (`Vision/PoseValid` = true)
- Ambiguity < 0.2

**Örnek Kullanım:**
```
Maç öncesi:
1. Robotu sahada başlangıç pozisyonuna koyun
2. Yakın bir AprilTag bulun (1-3 metre)
3. Limelight'ı tag'e doğru çevirin
4. Button 5'e basın
5. DriverStation: "Pose reset from AprilTag 1 at (3.61, 3.39)"
6. Robot pozisyonu güncellendi ✓
```

## SmartDashboard Motor Test Toggle'ları

### MotorTest/FrontLeft
- **Motor**: Ön Sol (ID 1)
- **Invert**: TRUE
- **DISABLED modda**: + veya - butonu ile test
- **Güç**: %20

### MotorTest/FrontRight
- **Motor**: Ön Sağ (ID 2)
- **Invert**: FALSE
- **DISABLED modda**: + veya - butonu ile test
- **Güç**: %20

### MotorTest/RearLeft
- **Motor**: Arka Sol (ID 4)
- **Invert**: TRUE
- **DISABLED modda**: + veya - butonu ile test
- **Güç**: %20

### MotorTest/RearRight
- **Motor**: Arka Sağ (ID 3)
- **Invert**: FALSE
- **DISABLED modda**: + veya - butonu ile test
- **Güç**: %20

## CAN ID Eşleşmesi

Fiziksel CAN kablo bağlantıları:

| CAN ID | Pozisyon | Motor Invert |
|--------|----------|--------------|
| 1 | Ön Sol | TRUE |
| 2 | Arka Sağ | FALSE |
| 3 | Ön Sağ | FALSE |
| 4 | Arka Sol | TRUE |

## Motor Test Akışı

### Pit Alanında Motor Testi

1. **Robot DISABLED modda olduğundan emin olun**
2. **SmartDashboard'ı açın**
3. **Motor Test toggle'larını görün**
4. **Test motorunu seçin** (örneğin MotorTest/FrontLeft)
5. **+ tuşuna basın** (SmartDashboard'da)
6. **Motor dönmeli** - %20 güçle
7. **Tekerlek yönünü kontrol edin**
8. **- tuşuna basın** - Motor diğer yöne döner
9. **Tüm motorlar için tekrarlayın**

### Buton ile Test Akışı

```
1. Robot DISABLED
2. Button 1'ye basılı tut
3. Tüm motorlar %30 güçle döner
4. Tekerlek yönlerini kontrol edin:
   - Ön Sol ve Arka Sol: AYNI YÖN
   - Ön Sağ ve Arka Sağ: AYNI YÖN
   - Sol ve Sağ: ZIT YÖN
5. Button 1'i bırak
6. Motorlar durur
```

## Vision Reset Detayları

### Ne Zaman Kullanılmalı?

**✅ Kullanın:**
- Maç öncesi başlangıç pozisyonu için
- Odometry drift şüphesi olduğunda
- Autonomous öncesi pozisyon doğrulama için
- Pit alanında vision testi için

**❌ Kullanmayın:**
- Maç ortasında (eğer pozisyonunuz doğruysa)
- AprilTag görünmüyorken
- Robot hareket halindeyken

### Vision Reset Sonrası Kontrol

SmartDashboard'da şu değerleri kontrol edin:

1. **`Robot X`**: 0-16.54 arası (saha boyunca)
2. **`Robot Y`**: 0-8.21 arası (saha genişliğinde)
3. **`Robot Heading`**: -180 ila 180 derece
4. **`Vision/TagID`**: Hangi tag kullanıldı (1-16)
5. **`Drive/VisionUpdate`**: TRUE olmalı

### Hata Mesajları

| Mesaj | Anlamı | Çözüm |
|-------|--------|-------|
| "Pose reset from AprilTag X..." | Başarılı ✓ | - |
| (Mesaj yok) | Reset başarısız | Tag'i kontrol edin |
| "No valid vision result" | Tag görünmüyor | Limelight'ı çevirin |

## Güvenlik Notları

### Test Güçleri

- **Buton testi**: %30 güç
- **Toggle testi**: %20 güç
- **Güvenli seviyeler** - patinaj yapmaz

### Test Sırasında

1. ⚠️ **Sadece DISABLED modda**
2. ⚠️ **Robot yerinde sabit kalmalı**
3. ⚠️ **Tekerleklerin çevresinde boşluk**
4. ⚠️ **Gerekirse robotu yukarı kaldırın**

---

## Referanslar

- **Motor ID'leri**: Constants.java (MotorConstants class)
- **Buton bağları**: RobotContainer.java (configureBindings)
- **Vision kodu**: VisionSubsystem.java, DriveSubsystem.java
- **FMAP bilgis**: AprilTagFieldLayout.java
