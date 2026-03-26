# FRC 2026 Robot Kullanim Rehberi

Bu dosya, onceki asagidaki dokumanlarin birlestirilmis halidir:
- APRILTAG_ALIGN_TEST_GUIDE.md
- APRILTAG_TEST_PLAN.md
- BUTTON_REFERENCE.md
- QUICK_REFERENCE.md

## 1) Kontroller

### Surucu kontrolcusu (Port 0)
- Sol analog Y: ileri/geri
- Sol analog X: yanal
- Sag analog X: donus

### Buton atamalari (guncel)
- Button 1: PWM test motoru seviye secimi
- Button 2: PWM test motoru seviye secimi
- Button 3: PWM test motoru seviye secimi
- Button 4: PWM test motoru seviye secimi (en yuksek oncelik)
- L1 (Button 5): CAN ID 3 motorunu %10 geri test ettirir
- R1 (Button 6): CAN ID 3 motorunu %10 ileri test ettirir
- R2 (Button 8): AprilTag surekli takip komutu

Not: L1/R1 birakildiginda motor durdurulur.

## 2) PWM Test Motoru Davranisi

PWM test motoru periodic dongude butonlara gore surulur:
- 4 basiliysa: 1.00
- degilse 3 basiliysa: 0.90
- degilse 2 basiliysa: 0.80
- degilse 1 basiliysa: 0.75
- hicbiri degilse: 0.00

## 3) AprilTag Kullanim Ozeti

### R2 ile takip
- R2 basili tutulurken AprilTag takip calisir
- hedef: Tag ID 1
- takip mesafesi: 1.5m

### Vision durum gostergeleri
- Vision/HasTarget
- Vision/PoseValid
- Vision/TagID
- Vision/RobotX
- Vision/RobotY
- Vision/RobotYaw
- Drive/VisionUpdate

## 4) Hizli Test Akisi

1. Robot baglantisini dogrula (DriverStation + SmartDashboard).
2. Vision/FmapTagCount ve Vision/HasTarget kontrol et.
3. DISABLED modda L1/R1 ile CAN ID 3 yon testini yap.
4. ENABLED modda temel surus eksenlerini test et.
5. R2 ile AprilTag takip davranisini dogrula.

## 5) Mekanizma Donanim Guncellemesi (2026-03-26)

- Intake sistemi: 1 adet CIM motor
- Depodan aticiya yukari tasima: 1 adet CIM motor
- Shooter sistemi: NEO + Spark Max
- Turret sistemi: NEO + Spark Max

Kod yansimalari:
- IntakeSubsystem iki CIM hatti yonetir (alim + yukari tasiyici)
- ShootCommand atis baslangicinda yukari tasiyiciyi da calistirir

## 6) Sorun Giderme

- Robot hareket etmiyorsa: E-Stop, enable durumu ve joystick baglantisini kontrol et.
- Vision yoksa: Limelight gorus acisi, tag mesafesi ve isik durumunu kontrol et.
- Motor testinde tepki yoksa: ilgili CAN/PWM baglantisini ve ters yon ayarini kontrol et.

## 7) Ilgili Dokumanlar

- ROBOT_SETUP.md: donanim/CAN/PWM haritasi ve kurulum notlari
- HARDWARE_VALIDATION.md: dogrulama artifact akisi
