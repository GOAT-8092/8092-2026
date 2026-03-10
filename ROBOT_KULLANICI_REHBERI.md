# FRC 2026 Robot Kullanici Rehberi

## Icindekiler

1. [Kontrolcüler](#kontrolcüler)
2. [Suruus Kontrolleri](#suruus-kontrolleri)
3. [Test Butonlari](#test-butonlari)
4. [Vision/AprilTag Kullanimi](#visionapriltag-kullanimi)
5. [SmartDashboard Göstergeleri](#smartdashboard-göstergeleri)
6. [Güvenlik Ipuclari](#güvenlik-ipuclari)
7. [Sorun Giderme](#sorun-giderme)

---

## Kontrolcüler

### Driver (Sürücü) - PS4 Controller (Port 0, Model: CUH-ZCT2U)

- **Sol Stick**: Ileri/Geri ve Strafe (Sag/Sol)
- **Sag Stick**: Döndürme (Z ekseni)
- **Buton 1 (Square)**: On Sol motoru test
- **Buton 2 (Cross)**: Arka Sol motoru test
- **Buton 3 (Circle)**: Arka Sag motoru test
- **Buton 4 (Triangle)**: On Sag motoru test
- **Buton 5 (L1)**: Vision pose reset
- **Buton 6 (R1)**: AprilTag'e hizala (basili tut)
- **Buton 7 (L2)**: AprilTag'e 360 derece don
- **Buton 8 (R2)**: AprilTag surekli takip (basili tut)

### Operator (Operatör) - Joystick (Port 1)

- Su an hicbir fonksiyona atanmamistir
- IntakeSubsystem, ShooterSubsystem ve TurretSubsystem RobotContainer'da henuz olusturulmamistir
- Gelecekte intake/shooter/turret kontrolü icin planlanmaktadir

---

## Suruus Kontrolleri

### Mekanum Suruus Modu

Robot 4 tekerlekli mekanum suruus sistemine sahiptir - her yöne hareket edebilir.

| Eksen | Fonksiyon | Axis No | Aciklama |
|-------|-----------|---------|----------|
| Y Ekseni | Ileri/Geri | 0 | Sol stick ileri/geri |
| X Ekseni | Strafe | 1 | Sol stick sag/sol |
| Z Ekseni | Döndurme | 2 | Sag stick sag/sol (kodda ters cevrilmis) |

### Robot Yönüne Gore Suruus (Robot Oriented)

Robot kendi yönüne gore hareket eder:

- Stick ileri = Robotun baktigi yöne ileri gider
- Stick saga = Robotun sag tarafina strafe yapar
- NavX jiroscop suruus kontrolünde kullanilmaz

### Önemli Ayarlar

| Ayar | Deger | Aciklama |
|------|-------|----------|
| Deadband | 0.08 | Stick "ölü bölgesi" |
| Maksimum Hiz | ~3.0 m/s | NEO motorlarla |

---

## Test Butonlari

UYARI: Motor test butonlari (1-4) ROBOT DISABLED modundayken calisir.

### Buton 1 (Square): On Sol Motor (CAN ID 2)

- On sol NEO motorunu %30 gucle calistirir
- CAN ID dogrulamasi icin kullanin

### Buton 2 (Cross): Arka Sol Motor (CAN ID 3)

- Arka sol NEO motorunu %30 gucle calistirir
- CAN ID dogrulamasi icin kullanin

### Buton 3 (Circle): Arka Sag Motor (CAN ID 1)

- Arka sag NEO motorunu %30 gucle calistirir
- CAN ID dogrulamasi icin kullanin

### Buton 4 (Triangle): On Sag Motor (CAN ID 4)

- On sag NEO motorunu %30 gucle calistirir
- CAN ID dogrulamasi icin kullanin

### Buton 5 (L1): Pozisyon Reset (Vision)

- AprilTag ile robot pozisyonunu resetler
- Kullanim:
  1. Robotu bir AprilTag'in önüne koyun
  2. L1'e basin
  3. DriverStation'da "Pose reset from AprilTag X..." mesaji görünür
  4. Robot pozisyonu o tag'e göre güncellenir

### Buton 6 (R1): AprilTag'e Hizala

- Basili tutarken robot Tag 1'e hizalanir (1.0m mesafe)
- Birakinca hareket durur

### Buton 7 (L2): AprilTag'e 360 Derece Don

- Bir kez basin, robot Tag 1'e dogru döner
- Komut tamamlaninca durur

### Buton 8 (R2): AprilTag Surekli Takip

- Basili tutarken robot Tag 1'i 1.0m mesafede takip eder
- Birakinca hareket durur

---

## Vision/AprilTag Kullanimi

### Limelight 3 AprilTag Sistemi

Robot 2026 FRC sahasindaki 32 AprilTag'i görebilir ve pozisyonunu belirleyebilir.

### Vision Göstergeleri (SmartDashboard)

| Gösterge | Anlami | Ideal Deger |
|----------|--------|------------|
| `Vision/HasTarget` | AprilTag görünüyor mu | true (tag göründügünde) |
| `Vision/PoseValid` | Pozisyon gecerli mi | true |
| `Vision/RobotX` | Sahada X pozisyonu (m) | 0-16.54 |
| `Vision/RobotY` | Sahada Y pozisyonu (m) | 0-8.21 |
| `Vision/RobotYaw` | Robot basligi (derece) | -180 ila 180 |
| `Vision/TagID` | Görülen tag ID'si | 1-16 |
| `Vision/Ambiguity` | Algilama belirsizligi | < 0.2 (dusuk iyi) |
| `Vision/FmapTagCount` | FMAP tag sayisi | 32 |
| `Drive/VisionUpdate` | Vision güncellemesi | true (sik sik) |

### Vision ile Pozisyon Alma

Otomatik Odometry Güncellemeleri:
- Robot her 50ms'de bir vision ölcümü alir
- Odometry drift'i otomatik düzeltilir
- Hem autonomous hem teleop'ta calisir

Manuel Pozisyon Reset (L1 / Buton 5):
```
1. Robotu bir AprilTag'in 1-3 metre önüne koyun
2. Limelight'in tag'i gördügünden emin olun (Vision/HasTarget = true)
3. L1'e basin
4. Robot pozisyonu o tag'e göre ayarlanir
```

### Sahadaki AprilTag'lar

2026 FRC Andymark Sahasi:
- Toplam: 32 AprilTag (16 mavi, 16 kirmizi)
- Tag Boyutu: 6.5 inc (165.1mm)
- Tag Ailesi: 36h11

---

## SmartDashboard Göstergeleri

### Robot Pozisyonu

| Gösterge | Aciklama |
|----------|----------|
| `Robot X` | Sahada X pozisyonu (metre) |
| `Robot Y` | Sahada Y pozisyonu (metre) |
| `Robot Heading` | Robot basligi (derece) |

### Suruus Motorlari

| Gösterge | Aciklama |
|----------|----------|
| `Drive/FrontLeftOutput` | On sol motor gucu (% -1.0 ila 1.0) |
| `Drive/FrontRightOutput` | On sag motor gucu |
| `Drive/RearLeftOutput` | Arka sol motor gucu |
| `Drive/RearRightOutput` | Arka sag motor gucu |

### Motor Test Toggle'lari (Sadece DISABLED)

| Toggle | Motor | CAN ID |
|--------|-------|--------|
| `MotorTest/FrontLeft` | On Sol | 2 |
| `MotorTest/FrontRight` | On Sag | 4 |
| `MotorTest/RearLeft` | Arka Sol | 3 |
| `MotorTest/RearRight` | Arka Sag | 1 |

### NavX Test Göstergeleri

| Gösterge | Aciklama |
|----------|----------|
| `NavXTest/Run` | Test baslatmak icin true yapın (sadece DISABLED modda) |
| `NavXTest/Status` | Test durumu |
| `NavXTest/ErrorCode` | Hata kodu (varsa, NAVX_E001-E007) |

### PathPlanner

| Gösterge | Aciklama |
|----------|----------|
| `Auto Chooser` | Otonom modu secici |
| `2026 FRC Field` | Saha görselleştirmesi |

---

## Güvenlik Ipuclari

### Robot Baslatma Öncesi

1. Alan Kontrolü:
   - Robotun etrafinda bosluk oldugundan emin olun
   - Tüm kablolar güvenli bir sekilde baglanmis olmali

2. Güvenli Bölge:
   - Robot en az 2 metre radius'lik bos alana ihtiyac duyar
   - Cocuklar ve izleyiciler güvenli mesafede kalmali

3. Acil Durum:
   - E-Stop butonuna her zaman hazirlikli olun
   - Space tusu = Robot disable

### Suruus Sirasinda

1. Hiz Kontrolü:
   - Ilk kullanimda yavas baslayin
   - Deadband (0.08) nedeniyle kücük stick hareketleri görülmeyebilir

2. Mekanum Suruus:
   - Strafe (yana kayma) suruusü farkli hissedebilir
   - Robot kendi yönüne göre hareket eder, saha yönüne göre degil

3. Vision Kullanimi:
   - AprilTag'ler net görünmeli
   - Günes isigi vision'i etkileyebilir

### Pit Alaninda

1. Motor Test:
   - Sadece DISABLED modda kullanin
   - %30 guc güvenli test seviyesidir

2. Pozisyon Reset:
   - L1 (Buton 5) ile ENABLED modda yapin
   - AprilTag'in net göründügünden emin olun

---

## Sorun Giderme

### Robot Hareket Etmiyor

| Belirti | Olasi Sebep | Cözüm |
|---------|-------------|-------|
| Tüm motorlar calismıyor | E-Stop basili | E-Stop'u serbest birakin, robot'u yeniden enable edin |
| Tek motor calismıyor | CAN baglatisi | Motor ID'sini ve kablolari kontrol edin |
| Tekerlekler dönüyor ama robot gitmiyor | Tekerlek kayması | Zeminde yeterli traction yok |
| Garip suruus davranisi | Odometry drift'i | L1 ile vision reset yapin |

### Vision Calismıyor

| Belirti | Olasi Sebep | Cözüm |
|---------|-------------|-------|
| `Vision/HasTarget` = false | Tag görünmüyor | Limelight'i tag'e dogru cevirin |
| `Vision/PoseValid` = false | Ambiguity yüksek | Tag'i daha yakindan/net görün |
| `Vision/FmapTagCount` ≠ 32 | FMAP yüklenmemis | Limelight web arayüzünden FMAP yükleyin |
| Yanlis pozisyon | Kamera ayarlari | Camera height/pitch ayarlarini kontrol edin |

### SmartDashboard Verileri Yok

| Belirti | Olasi Sebep | Cözüm |
|---------|-------------|-------|
| Göstergeler görünmüyor | NetworkTables baglantisi | Robot baglantisini kontrol edin |
| Eski veriler | Robot disable | Robot'u enable edin |
| NaN degerler | Sensör hatasi | NavX ve encoder'lari kontrol edin |

### Suruus Davranisi Sorunlari

| Belirti | Olasi Sebep | Cözüm |
|---------|-------------|-------|
| Stick tepki vermiyor | Deadband cok yüksek | Constants.java'dan deadband'i düsürün |
| Yavas hiz | Cok dusuk guc | Drive scale ayarlarini kontrol edin |

---

## Hizli Baslangic Kontrol Listesi

### Mac Öncesi

- [ ] Robot gücünü acin
- [ ] DriverStation'dan baglanin
- [ ] SmartDashboard'i acin
- [ ] `Vision/FmapTagCount` = 32 kontrol edin
- [ ] `Robot X`, `Robot Y` degerlerini görün
- [ ] Motor test toggle'larini DISABLED modda test edin
- [ ] Robot'u enable edin
- [ ] Hafif suruus testi yapin
- [ ] L1 ile pozisyon reset test edin

### Mac Sirasi

- [ ] Autonomous öncesi: L1 ile pozisyon reset
- [ ] Autonomous: Auto Chooser'dan modu secin
- [ ] Teleop baslangicinida: Vision calistigından emin olun
- [ ] Suruus sirasinda: `Drive/VisionUpdate` = true kontrol edin

### Mac Sonrasi

- [ ] Robot'u disable edin
- [ ] Gucu kapatin
- [ ] Pil durumunu kontrol edin
- [ ] Hasar kontrolü yapin

---

## Ag Adresleri

| Cihaz | Adres |
|-------|-------|
| roboRIO | 10.80.92.2 |
| Radyo | 10.80.92.1 |
| Limelight | 10.80.92.200 |
| Limelight Web Arayüzü | http://10.80.92.200:5801 |

---

## Destek

Sorun yasarsaniz:

1. DriverStation Loglari: Hata mesajlarini kontrol edin
2. SmartDashboard: Vision ve drive verilerini inceleyin
3. Limelight Web: http://10.80.92.200:5801
4. RIO Log: Rio log dosyalarini kontrol edin

---

## Ek Kaynaklar

- **ROBOT_KURULUM.md**: Donanim kurulum detaylari
- **CLAUDE.md**: Proje genel bakis
- **WPILib Dokümantasyonu**: https://docs.wpilib.org/
- **Limelight Dokümantasyonu**: https://docs.limelightvision.io/

---

**Versiyon**: 2026 Sezonu
**Son Güncelleme**: Mart 2026
**Robot**: Mekanum Suruus + AprilTag Vision
