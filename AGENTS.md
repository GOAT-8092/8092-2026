# AGENTS.md

Bu dosya, bu depoda calisirken gelistirici yardimcisina hizli teknik baglam verir.

## Proje Ozeti

- Dil: Java (WPILib command-based)
- Robot: Mecanum surus + AprilTag vision + taret
- Ana siniflar:
  - `Robot.java`
  - `RobotKapsayici.java`
  - `Sabitler.java`

## Robot Fiziksel Yonelimi

- **On taraf:** Taret + Limelight 3 (atis/hedefleme yonu)
- **Arka taraf:** Alim (intake)
- Surus yaziliminda 180° donuk koordinat sistemi uygulanmistir

## Guncel Donanim

### Surus
- 4x NEO + Spark Max (CAN)
- CAN: 3 (on sol), 4 (on sag), 1 (arka sag), 2 (arka sol)

### Mekanizma
| Alt Sistem | Motor | Baglanti | CAN/PWM | Konum | Sabit |
|---|---|---|---|---|---|
| Alim (intake) | CIM | PWM | 9 | Arka | `ALIM_CIM_PWM_KANALI` |
| Yukari tasiyici | CIM | PWM | 8 | Arka | `DEPO_ATICI_YUKARI_TASIYICI_CIM_PWM_KANALI` |
| Atici (shooter) | NEO | CAN | 5 | On | `ATICI_MOTOR_ID` |
| Taret (turret) | NEO | CAN | 6 | On | `TARET_MOTOR_ID` |

### Sensor
- NavX (MXP SPI)
- Limelight 3 — robotun on tarafinda, taret ile ayni yuzde

## Kod Yapisi

### Alt Sistemler (`Subsystems/`)
- `SurusAltSistemi` — mecanum surus, odometri, NavX, MegaTag2 poz fuzyonu
- `GorusAltSistemi` — Limelight 3, AprilTag MegaTag2 poz tahmini
- `AlimAltSistemi` — alim CIM + yukari tasiyici CIM
- `AticiAltSistemi` — atici NEO, velocity PID + feedforward (4000 RPM hedef)
- `TaretAltSistemi` — taret NEO, ±90° yazilim + donanim soft limit, MAXMotion trapezoidal

### Komutlar (`Commands/`)
- `SurusKomutu` — joystick ile mecanum surus
- `AlimKomutu` — alim calistir
- `AtisKomutu` — atis + tasiyici
- `TaretHomingKomutu` — tareti -90° limit switch'e suruklep enkoderi sifirlar
- `TaretTakipKomutu` — Limelight tx ile taret takibi *(simdilik baglanmamis)*
- `PozTabanliTaretKomutu` — odometri + hub koordinati ile taret *(simdilik devre disi)*
- `HedefeHizalamaKomutu` — tareti hedefe hizala *(OtonomAtisKomutu icinde)*
- `OtonomAtisKomutu` — hizala + at siralisi
- `AprilTagaHizalamaKomutu` / `AprilTagTakipKomutu` — vision tabanli surus (PathPlanner)

## Buton Yerlesimi (PS4, Port 0)

| Buton | PS4 | Eylem | Motor | Davranis |
|---|---|---|---|---|
| 1 | Kare | Alim | PWM 9 | whileTrue |
| 2 | Carpi | Geri at | PWM 9 | whileTrue |
| 3 | Daire | Yukari tasiyici | PWM 8 | whileTrue |
| 4 | Ucgen | Atici | CAN 5 | whileTrue |
| 5 | L1 | Taret sola | CAN 6 | whileTrue |
| 6 | R1 | Taret saga | CAN 6 | whileTrue |
| 7 | L2 | Taret homing (DIO 9) | CAN 6 | toggleOnTrue |
| 8 | R2 | **Devre disi** (otomatik taret iptal) | — | — |
| 9 | Paylas | Tasiyici ters (0.5 s) | PWM 8 | onTrue |
| 10 | Secenekler | Atici + 1 s sonra tasiyici | CAN 5 + PWM 8 | whileTrue |

## Calisma Hizlari (`Sabitler.ModulSabitleri`)

| Sabit | Deger | Aciklama |
|---|---|---|
| `ALIM_HIZI` | 0.5 | Alim/geri-at hizi |
| `DEPO_ATICI_YUKARI_TASIYICI_HIZI` | 0.5 | Tasiyici hizi |
| `ATICI_HEDEF_RPM` | 4000 | Atici hedef hizi (velocity PID) |
| `TARET_HIZI` | 0.04 | Manuel taret hizi |

## Taret Sinirlari ve Homing

- **Konum:** Robotun on tarafinda; 0° = robotun onu, -90° = limit switch (baslangic)
- Yazilim siniri + SparkMax donanim soft limit: **±90°** (toplam 180° hareket alani)
- Kablo nedeniyle tam tur donusu yapilmaz
- Limit switch: DIO 9, normally closed — -90° pozisyonunda kurulu
- L2 (buton 7) → `TaretHomingKomutu`: tareti -90°'ye suruklep enkoderi sifirlar
- `TARET_ON_OFFSET_DERECE = 0.0` (poz tabanli hesaplamada front-mount offseti)
- Otomatik taret takibi (buton 8) simdilik **devre disi** — kablo guvenligi icin

## Derleme ve Deploy

```bash
./gradlew build
./gradlew test
./gradlew deploy
```

## Onemli Notlar

- Surus disi alt sistemleri fiziksel olarak etkinlestirmek icin:
  - `Sabitler.MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN = true`
- Taret enkoderi goreli (relative) calisir; her mactan once L2 ile homing yapilmalidir.
- Limelight MegaTag2 kullaniliyor: her dongude `setRobotOrientation()` cagriliyor.
- Atici velocity PID ile calisir; `isHizaUlasti()` ±200 RPM toleransla hazirlik kontrol eder.

## Referans Dokuman

- `ROBOT.md` — donanim haritasi, buton yerlesimi, test akisi, sorun giderme
