# Donanım Doğrulama Paketi

Bu proje, robot donanımının durumunu kayıt altına alan yapılandırılmış bir doğrulama akışı içerir.

Her kontrol sonucu aşağıdaki bilgileri kaydeder:

- Durum: PASS / FAIL
- UTC zaman damgası
- Robot derleme hash'i
- Kontrol ayrıntıları

## Artifact Formatı

`build/hardware-validation/results.txt` dosyasındaki her satır:

```
STATUS | CHECK_NAME | TIMESTAMP | BUILD_HASH | DETAILS
```

Örnek:

```
PASS | MOTOR_CAN_ID_1 | 2026-03-10T14:32:00Z | cc8a29d | Rear Right, inverted=true, current=2.1A
FAIL | LIMELIGHT_PING | 2026-03-10T14:32:05Z | cc8a29d | 10.80.92.200 erisilemedi
```

## Dogrulama Aracini Calistirma

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run-hardware-validation.ps1 -BuildHash "<git-hash>"
```

Git hash almak icin:

```bash
git rev-parse --short HEAD
```

## Dogrulama Proseduru (Adim Adim)

1. Driver Station'da robotu Disabled moduna alin.
2. Battery voltajinin 12V uzerinde oldugunu kontrol edin (minimum 10.0V).
3. CAN bus hata sayisini dogrulayin (sifir olmali).
4. Her surucu motoru icin bireysel motor testlerini calistirin (SmartDashboard motor toggle).
5. Her motorun encoder yonunu ve akim davranisini dogrulayin.
6. roboRIO sagligini kontrol edin (brownout yok, CPU normal).
7. Limelight pipeline / camMode / ledMode ayarlarini dogrulayin.
8. FMAP tag sayisini kontrol edin (32 tag bekleniyor).
9. NavX yaw dogrulama testini calistirin (SmartDashboard NavXTest/Run = true).
10. Artifact'i roboRIO'da kullanilan derleme hash'i ile kaydedin.

## Motor Kontrol Listesi (CAN Bus)

### Surucu Motorlar (Spark MAX, NEO V1.1)

| CAN ID | Konum          | Inverted | Kontrol Durumu |
|--------|----------------|----------|----------------|
| 1      | Arka Sag       | true     | Aktif          |
| 2      | On Sol         | false    | Aktif          |
| 3      | Arka Sol       | false    | Aktif          |
| 4      | On Sag         | true     | Aktif          |

Dogrulama: Her motora ayri ayri kisa sureli pozitif voltaj verin. Robot ileri hareket ettiginde tum motorlar dogru yonde donmelidir. Ters donme varsa ilgili motorun inverted ayarini kontrol edin.

### Ek Motorlar (ENABLE_NON_DRIVE_MOTORS = false, su an devre disi)

| CAN ID | Alt Sistem     | Inverted | Durum     |
|--------|----------------|----------|-----------|
| 5      | Intake         | false    | Devre disi|
| 6      | Shooter Sol    | false    | Devre disi|
| 7      | Shooter Sag    | true     | Devre disi|
| 8      | Shooter Ust    | false    | Devre disi|
| 9      | Turret         | false    | Devre disi|

Bu motorlari etkinlestirmek icin `Constants.java` dosyasinda `ENABLE_NON_DRIVE_MOTORS = true` yapin.

## Akim ve Voltaj Kontrolleri

### NEO Motor Akim Limitleri (Constants.java)

- Stall akim limiti: 60 A (fiziksel stall akimi 105 A, limit koruma saglar)
- Serbest akim limiti: 40 A
- Akim esigi: 40 RPM

### Voltaj Kontrolleri

- Minimum calisma voltaji: 10.0 V (brownout esigi altinda robot devre disi olur)
- Normal calisma voltaji: 12 V - 12.8 V
- Tam yuk altinda beklenen voltaj dususu: 0.5 V - 1.0 V

### Beklenen Akim Degerleri (Normal Kosullarda)

- Hareketsiz (idle): ~2 A per motor
- Hafif yukte surucu: ~5-15 A per motor
- Agir manevra: ~20-40 A per motor
- Stall (asiri yuk - kacinin): 60 A limit devreye girer

## Limelight Kontrol Listesi

- IP erisimi: `ping 10.80.92.200`
- Web arayuzu: `http://10.80.92.200:5801`
- Pipeline: 0 (AprilTag)
- camMode: 0 (Vision processor)
- ledMode: 0 (Pipeline kontrolu)
- FMAP: FRC2026_ANDYMARK.fmap (32 tag)
- Hedef taglar: 12 ve 15
- Kamera cozunurlugu: 640x480 @ 90 FPS
- Goruntu acisi: 62.5 yatay, 48.9 dikey
- Kamera yuksekligi: 0.50 m
- Kamera acisi: 25 derece

SmartDashboard'da `Vision/HasTarget` degerini kontrol edin. Hedef gorunuyor olmalidir.

## NavX Kontrol Listesi

- Baglanti: roboRIO MXP SPI
- Kullanim: Yaw dogrulama testi (suruşte kullanılmıyor)
- Test calistirma: SmartDashboard `NavXTest/Run = true` (sadece Disabled modunda)

### NavX Test Parametreleri

- TEST_TURN_OUTPUT: 0.3 (motor cikis gucu)
- MIN_EXPECTED_DELTA_DEG: 45 derece
- TURN_PHASE_TIMEOUT_SEC: 5.0 saniye
- ZERO_SETTLE_MS: 500 ms

### NavX Hata Kodlari

| Kod        | Anlam                                  |
|------------|----------------------------------------|
| OK         | Test gecti                             |
| NAVX_E001  | NavX bagli degil / okuma gecersiz      |
| NAVX_E002  | Saat yonu yon uyusmazligi              |
| NAVX_E003  | Saat yonu tersi yon uyusmazligi        |
| NAVX_E004  | Yaw degisimi beklenen esik altinda     |
| NAVX_E005  | Faz zaman asimi                        |
| NAVX_E006  | Disabled disi tetikleme girişimi       |
| NAVX_E007  | Yaw atlama / aykiri deger              |

## roboRIO Kontrol Listesi

- Hostname: roboRIO-8092-FRC
- IP: 10.80.92.2
- Baglanti: `ping 10.80.92.2`
- Firmware: 25.5.0f112
- Image: FRC_roboRIO_2026_v1.2
- Seri No: 031885C1
- CPU: 2 cekirdek
- Driver Station baglantisi: USB veya ag uzerinden

## Ag Kontrol Listesi

| Cihaz      | IP           | Model                  |
|------------|--------------|------------------------|
| roboRIO    | 10.80.92.2   | NI roboRIO 1.0         |
| Radyo      | 10.80.92.11  | Vivid-Hosting VH-109   |
| Limelight  | 10.80.92.200 | Limelight 3            |

Takim numarasi: 8092
