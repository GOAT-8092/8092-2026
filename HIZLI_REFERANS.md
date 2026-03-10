# Robot Hizli Referans Karti

## PS4 Surucü Kontrolcüsü (Port 0)

```
                   +-----------------+
                   |                 |
        L Stick    |    o o  * o o   |    R Stick
     Ileri/Geri    |                 |    Döndürme
       + Strafe    |    o o  * o o   |
                   |                 |
                   +-----------------+

        1  2  3  4  5  6  7  8
       Sq  X  O  Tr L1  R1 L2  R2
```

## Buton Fonksiyonlari

| Buton | PS4 Tus | Fonksiyon | Not |
|-------|---------|-----------|-----|
| 1 | Square | On Sol motoru calistir (CAN ID 2, %30) | Test - DISABLED modda |
| 2 | Cross | Arka Sol motoru calistir (CAN ID 3, %30) | Test - DISABLED modda |
| 3 | Circle | Arka Sag motoru calistir (CAN ID 1, %30) | Test - DISABLED modda |
| 4 | Triangle | On Sag motoru calistir (CAN ID 4, %30) | Test - DISABLED modda |
| 5 | L1 | Vision pose reset (AprilTag ile odometry sifirla) | ENABLED modda |
| 6 | R1 | AprilTag'e hizala (Tag 1, 1.0m) - basili tut | ENABLED modda |
| 7 | L2 | AprilTag'e 360 derece don (Tag 1) - bir kez bas | ENABLED modda |
| 8 | R2 | AprilTag surekli takip (Tag 1, 1.0m) - basili tut | ENABLED modda |
| 9-12 | - | Atanmamis | - |

## Surusu

**Sol Stick** = Ileri/Geri (Eksen 0) + Strafe Sag/Sol (Eksen 1)
**Sag Stick** = Döndürme (Eksen 2, kodda ters cevrilmis)

**Suruus Modu**: Robot yonune gore hareket eder (Robot Oriented)
- Stick yonu robotun kendi yonune göredir, saha yonune degil
- NavX suruus kontrolünde kullanilmaz

## CAN ID Motor Eslesmesi

| CAN ID | Motor Konumu | Ters Cevrilmis |
|--------|-------------|----------------|
| 1 | Arka Sag (Rear Right) | Evet |
| 2 | On Sol (Front Left) | Hayir |
| 3 | Arka Sol (Rear Left) | Hayir |
| 4 | On Sag (Front Right) | Evet |

## Vision Komutlari

```
Buton 5 (L1) - Pose Reset:
1. Robotu bir AprilTag'in önüne koyun
2. Limelight tag'i görsün (Vision/HasTarget = true)
3. L1'e basin
4. DriverStation mesaji: "Pose reset from AprilTag X..."

Buton 6 (R1) - Tag'e Hizala:
- Basili tutarken robot Tag 1'e hizalanir (1.0m mesafe)

Buton 7 (L2) - 360 Derece Don:
- Bir kez basin, robot Tag 1'e dogru döner

Buton 8 (R2) - Surekli Takip:
- Basili tutarken robot Tag 1'i takip eder (1.0m mesafe)
```

## SmartDashboard - Önemli Göstergeler

| Gösterge | Ne Demek |
|----------|----------|
| `Vision/HasTarget` | AprilTag görünüyor |
| `Vision/PoseValid` | Pozisyon dogru |
| `Vision/RobotX` | Sahada X (m) |
| `Vision/RobotY` | Sahada Y (m) |
| `Robot Heading` | Robot acisi (derece) |
| `Drive/VisionUpdate` | Vision calisiyor |

## SmartDashboard Motor Toggle'lari (Sadece DISABLED)

| Toggle | Motor | CAN ID |
|--------|-------|--------|
| `MotorTest/FrontLeft` | On Sol | 2 |
| `MotorTest/FrontRight` | On Sag | 4 |
| `MotorTest/RearLeft` | Arka Sol | 3 |
| `MotorTest/RearRight` | Arka Sag | 1 |

## Ag Adresleri

| Cihaz | Adres |
|-------|-------|
| roboRIO | 10.80.92.2 |
| Radyo | 10.80.92.1 |
| Limelight | 10.80.92.200 |
| Limelight Web | http://10.80.92.200:5801 |

## Acil Durum

- **E-Stop** = Kirmizi buton
- **Space** = Robot disable (klavyede)
- **Enter** = Robot enable (klavyede)

## Önemli Degerler

- **Maksimum Hiz**: ~3.0 m/s
- **Deadband**: 0.08 (stick hassasiyeti)
- **Vision Update**: 20Hz (her 50ms)
- **AprilTag Sayisi**: 32 (16 her alliance)

## Sorun?

| Sorun | Cözüm |
|-------|-------|
| Robot gitmiyor | E-Stop kontrol edin |
| Vision yok | Tag'e dogru cevirin |
| Yanlis pozisyon | L1 (Buton 5) reset |
| Motor calismıyor | CAN ID ve kablo kontrol |

---

**Mac Öncesi Kontrol**: Vision/FmapTagCount = 32, motor toggle testleri tamam
