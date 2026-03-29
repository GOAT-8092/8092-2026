# FRC 2026 Robot Dokumantasyonu

## 1) Donanim Haritasi

### Robot Fiziksel Yonelimi

| Taraf | Bilesim |
|---|---|
| **On** | Taret (CAN 6) + Limelight 3 — atis ve hedefleme yonu |
| **Arka** | Alim (PWM 9) + Yukari Tasiyici (PWM 8) |

Surus yaziliminda 180° donuk koordinat sistemi uygulanmistir.

### Surus — Mecanum (CAN)

| Sabit | CAN ID | Konum |
|---|---|---|
| `ON_SOL_MOTOR_ID` | 3 | On sol |
| `ON_SAG_MOTOR_ID` | 4 | On sag |
| `ARKA_SAG_MOTOR_ID` | 1 | Arka sag |
| `ARKA_SOL_MOTOR_ID` | 2 | Arka sol |

Sag taraf ters, sol taraf duz. Disli orani: 12.75:1. Maks hiz: ~3.0 m/s.

### Mekanizma

| Alt Sistem | Motor | Baglanti | ID | Konum | Kontrol |
|---|---|---|---|---|---|
| Alim | CIM | PWM | 9 | Arka | %50 |
| Yukari tasiyici | CIM | PWM | 8 | Arka | %50 |
| Atici | NEO + Spark Max | CAN | 5 | On | Velocity PID — 4000 RPM |
| Taret | NEO + Spark Max | CAN | 6 | On | MAXMotion + soft limit |

> **Taret:** 0° = robotun onu. Yazilim + donanim soft limit: **±90°** (toplam 180°).
> Kablo nedeniyle tam tur donusu yapilmaz.
> Limit switch: DIO 9, normally closed — **-90° baslangic pozisyonu**.
> Her mactan once L2 ile homing yapilmalidir.

### Sensor ve Ag

| Birim | Baglanti | Adres |
|---|---|---|
| NavX | MXP SPI | roboRIO uzerinde |
| Limelight 3 | Ethernet | 10.80.92.200 |
| roboRIO | — | 10.80.92.2 |
| Radio | — | 10.80.92.1 |

---

## 2) Kontroller (PS4, Port 0)

### Analog Eksenler — Surus

| Eksen | Hareket |
|---|---|
| Sol analog Y | Ileri / geri |
| Sol analog X | Yanal |
| Sag analog X | Donus |

### Buton Atamalari

| Buton | PS4 | Eylem | Motor | Davranis |
|---|---|---|---|---|
| 1 | Kare | Alim | PWM 9 | whileTrue |
| 2 | Carpi | Geri at | PWM 9 | whileTrue |
| 3 | Daire | Yukari tasiyici | PWM 8 | whileTrue |
| 4 | Ucgen | Atici | CAN 5 | whileTrue |
| 5 | L1 | Taret sola | CAN 6 | whileTrue |
| 6 | R1 | Taret saga | CAN 6 | whileTrue |
| 7 | L2 | Taret homing (DIO 9) | CAN 6 | toggleOnTrue: ilk basin baslar, ikinci basin durdurur |
| 8 | R2 | **Devre disi** (otomatik taret iptal) | — | Kablo guvenligi icin simdilik iptal |
| 9 | Paylas | Tasiyici ters (0.5 s) | PWM 8 | onTrue |
| 10 | Secenekler | Atici + 1 s sonra tasiyici | CAN 5 + PWM 8 | whileTrue |

---

## 3) Konfigurasyon

`Sabitler.MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN`

- `true` — alim, tasiyici, atici, taret motorlari fiziksel cikis verir
- `false` — bu alt sistemler simule modda calisir (varsayilan)

Calisma hizlari ve PID sabitleri `Sabitler.ModulSabitleri` icerisinde tanimlidir.

---

## 4) Test Akisi

1. DriverStation + SmartDashboard / Elastic baglantisini dogrula.
2. `Vision/HasTarget` ve `Vision/FmapTagCount` degerlerini kontrol et.
3. `SURUS_DISI_MOTORLARI_ETKIN = false` iken butonlarin SmartDashboard'a yazdigini dogrula.
4. `SURUS_DISI_MOTORLARI_ETKIN = true` yap, DISABLED modda her butonu teker teker test et.
5. ENABLED modda surus eksenlerini kontrol et.
6. L2 ile taret homing yap: taret -90°'ye gitmeli, enkoder sifirlanmali.
7. L1/R1 ile manuel taret dondurmeyi test et; ±90° sinirda durmasini dogrula.

---

## 5) Donanim Dogrulama Akisi

Her test oturumunda asagidaki kontroller yapilmalidir:

1. Batarya >= 10.0V, brownout yok.
2. Her motor kanali (CAN 1-6, PWM 8-9) birer kez test edildi.
3. Enkoder yonu ve akim davranisi beklendigi gibi.
4. Limelight pipeline / camMode / ledMode dogru; FMAP tag sayisi eslesiyir.
5. NavX yaw degerleri mantikli (robot saat yonunde donunce pozitif olmamali — WPILib CCW-pozitif).
6. Taret homing sonrasi enkoder -90° gostermeli; robotun onune bakinca 0°.

### Dogrulama Artifact Uretimi

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run-hardware-validation.ps1 -BuildHash "<git-hash>"
```

Cikti: `build/hardware-validation/results.txt`

Her satir formati:

```
DURUM | KONTROL_ADI | ZAMAN_DAMGASI | BUILD_HASH | DETAYLAR
```

---

## 6) Sorun Giderme

| Belirti | Kontrol Edilecek |
|---|---|
| Robot hareket etmiyor | E-Stop, enable durumu, joystick port 0 baglantisi |
| Vision calismıyor | Limelight IP, pipeline, tag mesafesi, LED modu |
| Motor tepki vermiyor | `SURUS_DISI_MOTORLARI_ETKIN`, CAN/PWM kablo baglantisi, ters yon ayari |
| Taret siniri calisiyor | Homing yapilmadi; L2 ile homing yap veya robotu kapat/ac |
| Atici RPM'e ulasmiyor | `isHizaUlasti()` false; PID sabitleri veya voltaj kompansasyonu kontrol et |
| Taret yanlıs yonde donuyor | `TARET_MOTOR_TERS` sabitini guncelle |
