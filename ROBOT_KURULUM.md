# Robot Kurulum Notları

## Sürüş Sistemi

- 4 adet NEO fırçasız motor (REV-21-1650, NEO V1.1)
- 4 adet AndyMark 6" Mecanum tekerlek (0.1524 m çap)
- Motor sürücüler: REV Spark MAX (CAN bus)
- Şanzıman: Toughbox Mini Classic 12.75:1
- Maksimum hız: 3.0 m/s (teorik 3.55 m/s, kayıplar ile)

## CAN ID - Motor Eşleşmesi (Sürüş, Doğrulanmış)

| CAN ID | Konum                     | Inverted |
|--------|---------------------------|----------|
| 1      | Arka Sağ (Rear Right)     | true     |
| 2      | Ön Sol (Front Left)       | false    |
| 3      | Arka Sol (Rear Left)      | false    |
| 4      | Ön Sağ (Front Right)      | true     |

**Not:** Mecanum sürüşünde SAĞ taraf motorları (CAN ID 1 ve 4) SOL tarafa göre ters çevrilmiştir. Bu, rulo yönü farkından kaynaklanır. Pozitif voltaj verildiğinde tüm tekerlekler ileri yönde döner.

## Ek Motorlar (Su An Devre Dışı)

`ENABLE_NON_DRIVE_MOTORS = false` olduğundan bu motorlar çağrıları no-op yapar.

| CAN ID | Alt Sistem     | Inverted |
|--------|----------------|----------|
| 5      | Intake         | false    |
| 6      | Shooter Sol    | false    |
| 7      | Shooter Sağ    | true     |
| 8      | Shooter Üst    | false    |
| 9      | Turret         | false    |

Etkinleştirmek için `Constants.java` dosyasında `ENABLE_NON_DRIVE_MOTORS = true` yapın.

## NEO Motor Özellikleri

- Model: REV-21-1650 (NEO V1.1)
- Nominal Voltaj: 12 V
- Serbest Hız: 5676 RPM
- Serbest Akım: 1.8 A
- Stall Akımı (ampirik): 105 A
- Stall Torku (ampirik): 2.6 Nm
- Tepe Çıkış Gücü: 406 W
- Hall Encoder: 42 sayım/tur
- Mil: 8 mm anahtarlı, 35 mm uzunluk
- Ağırlık: 0.425 kg

## Akım Limitleri (Constants.java)

- Stall akım limiti: 60 A (NEO fiziksel stall 105 A, limit koruma sağlar)
- Serbest akım limiti: 40 A
- Akım eşiği: 40 RPM

## Sürüş Kontrol Eksenleri (Doğrulanmış)

PS4 fiziksel eksen eşleşmesi:

| Sabit          | Değer | PS4 Ekseni         | İşlev              |
|----------------|-------|--------------------|--------------------|
| DRIVER_Y_AXIS  | 0     | Sol Stick Y        | İleri / Geri       |
| DRIVER_X_AXIS  | 1     | Sol Stick X        | Strafe (yanlara)   |
| DRIVER_Z_AXIS  | 2     | Sağ Stick X        | Döndürme           |

**Önemli:** İleri hareket Y ekseninde negatif değerle tetiklenir (joystick ileri itildiğinde değer düşer, bu WPILib standardıdır).

## Kontrolcü

- Driver: Sony PS4 (CUH-ZCT2U), port 0
- Operator: Joystick, port 1

## Sürüş Modu

**ROBOT ORIENTED (Robot Odaklı) sürüş kullanılmaktadır.**

NavX jiroskop yalnızca yaw doğrulama testi için kullanılır. Sürüş komutları robot referans çerçevesinde çalışır; robot hangi yönü gösteriyorsa o yön ileridir. Alan odaklı sürüş aktif değildir.

## Sürüş Kontrol Ayarları (DriveControlConstants)

| Parametre              | Değer | Açıklama                                           |
|------------------------|-------|----------------------------------------------------|
| DEADBAND               | 0.08  | Minimum joystick girişi (titreşim önleme)          |
| Y_AXIS_SENSITIVITY     | 0.6   | İleri/Geri hassasiyet katsayısı                    |
| X_AXIS_SENSITIVITY     | 0.65  | Strafe hassasiyet katsayısı (mecanum kaybı için)   |
| Z_AXIS_SENSITIVITY     | 0.6   | Dönüş hassasiyet katsayısı                         |
| TRANSLATION_SLEW_RATE  | 1.0   | İleri/Strafe ivme limiti (yavaş = yumuşak)         |
| ROTATION_SLEW_RATE     | 2.0   | Dönüş ivme limiti                                  |
| TRANSLATION_SCALE      | 0.5   | İleri/Geri güç ölçeği                              |
| STRAFE_SCALE           | 0.6   | Strafe güç ölçeği                                  |
| ROTATION_SCALE         | 0.5   | Dönüş güç ölçeği                                  |

**Not:** Translation scale 0.5 ve Rotation scale 0.5 olarak ayarlanmıştır. Önceki sürümlerde 0.90 ve 0.75 olan değerler kasıtlı olarak düşürülmüştür; bu ayarlar daha yavaş ve kontrollü bir sürüş sağlar.

## Robot Boyutları

- Track Width (tekerlek arası genişlik): 0.52 m
- Wheel Base (ön-arka tekerlek arası): 0.575 m

## Ana Kontrol Birimi (roboRIO)

- Model: NI roboRIO 1.0
- Hostname: roboRIO-8092-FRC
- DNS: roboRIO-8092-FRC.lan
- IP (eth0): 10.80.92.2/24
- MAC (eth0): 00:80:2F:24:AB:01
- Gateway: 10.80.92.4
- DNS Sunucusu: 10.80.92.1
- IPv4 Modu: DHCP veya Link Local
- USB (usb0): Yalnızca DHCP
- Seri No: 031885C1
- Firmware: 25.5.0f112
- OS Image: FRC_roboRIO_2026_v1.2
- İşletim Sistemi: NI Linux Real-Time ARMv7-A 4.14.146-rt67
- CPU: 2 çekirdek / 2 mantıksal işlemci
- Doğrulama tarihi: 2026-03-05

## Güç Dağıtımı

- PDP: CTRE Power Distribution Panel
- VRM: CTRE Voltage Regulator Module (12 V ve 5 V regüle çıkış)
- Ana Sigorta: OptiFuse 120 A Main Breaker

## Ağ Yapılandırması

| Cihaz     | Model                  | IP           |
|-----------|------------------------|--------------|
| Radyo     | Vivid-Hosting VH-109   | 10.80.92.11  |
| roboRIO   | NI roboRIO 1.0         | 10.80.92.2   |
| Limelight | Limelight 3            | 10.80.92.200 |

Takım numarası: 8092

## Sensörler

### NavX Jiroskop

- Bağlantı: roboRIO MXP SPI
- Kullanım: Yalnızca yaw doğrulama testi (sürüşte kullanılmıyor)
- Test tetikleme: SmartDashboard `NavXTest/Run = true` (sadece Disabled modunda)

#### NavX Test Parametreleri

| Parametre                  | Değer   |
|----------------------------|---------|
| TEST_TURN_OUTPUT           | 0.3     |
| MIN_EXPECTED_DELTA_DEG     | 45.0 °  |
| TURN_PHASE_TIMEOUT_SEC     | 5.0 s   |
| ZERO_SETTLE_MS             | 500 ms  |
| MAX_ABS_YAW_JUMP_DEG       | 180.0 ° |

#### NavX SmartDashboard Alanları

- `NavXTest/Run` - Testi başlat (true yap)
- `NavXTest/Status` - Son test sonucu
- `NavXTest/ErrorCode` - Hata kodu
- `NavXTest/LastYawStartDeg` - Başlangıç yaw değeri
- `NavXTest/LastYawEndDeg` - Bitiş yaw değeri
- `NavXTest/DeltaDeg` - Ölçülen yaw değişimi

#### NavX Hata Kodları

| Kod        | Anlam                                        |
|------------|----------------------------------------------|
| OK         | Test başarıyla geçti                         |
| NAVX_E001  | NavX bağlı değil veya okuma geçersiz         |
| NAVX_E002  | Saat yönü dönüş yön uyuşmazlığı              |
| NAVX_E003  | Saat yönü tersi dönüş yön uyuşmazlığı        |
| NAVX_E004  | Yaw değişimi beklenen eşiğin altında         |
| NAVX_E005  | Faz zaman aşımı                              |
| NAVX_E006  | Disabled modu dışında test tetikleme girişimi|
| NAVX_E007  | Yaw sıçraması / aykırı değer                 |

Test tamamlandığında `NavXTest/Run` otomatik olarak `false` yapılır.

### Limelight 3 (Görüntüleme)

- Kamera: OV5647, 640x480 @ 90 FPS
- Görüş Açısı: 62.5° yatay, 48.9° dikey
- IP: 10.80.92.200
- Web Arayüzü: http://10.80.92.200:5801
- Montaj Yüksekliği: 0.50 m
- Montaj Açısı: 25°
- Pipeline: 0 (AprilTag modu)
- FMAP: FRC2026_ANDYMARK.fmap (32 tag)
- Hedef Tag ID'leri: 12 ve 15

#### Limelight Sabitleri (VisionConstants)

- LIMELIGHT_NAME: "limelight"
- DESIRED_PIPELINE: 0
- DESIRED_CAM_MODE: 0 (Vision processor)
- DESIRED_LED_MODE: 0 (Pipeline kontrolü)
- CAMERA_HEIGHT_METERS: 0.5
- CAMERA_PITCH_RADIANS: Math.toRadians(25.0)
- APRILTAG_SIZE_METERS: 0.1651 (6.5 inç)
- SPEAKER_TAG_HEIGHT_METERS: 0.889

## MXP SPI Bağlantıları

- NavX Jiroskop: roboRIO MXP SPI portu

## CAN Bus Özeti

| CAN ID | Cihaz                    | Tip       |
|--------|--------------------------|-----------|
| 1      | Arka Sağ Motor (Spark MAX)| Sürücü   |
| 2      | Ön Sol Motor (Spark MAX)  | Sürücü   |
| 3      | Arka Sol Motor (Spark MAX)| Sürücü   |
| 4      | Ön Sağ Motor (Spark MAX)  | Sürücü   |
| 5      | Intake (Spark MAX)        | Devre dışı|
| 6      | Shooter Sol (Spark MAX)   | Devre dışı|
| 7      | Shooter Sağ (Spark MAX)   | Devre dışı|
| 8      | Shooter Üst (Spark MAX)   | Devre dışı|
| 9      | Turret (Spark MAX)        | Devre dışı|
