# Elastic Dashboard Kurulum ve Kullanım Rehberi

## 1. Elastic Kurulumu

### İndirme
- GitHub: https://github.com/Gold872/elastic-dashboard/releases
- En güncel `.exe` / `.dmg` / `.AppImage` sürümünü indir.
- DriverStation bilgisayarına kur. Elastic ve DriverStation aynı anda çalışır.

### Layout Dosyasını Yükleme
1. Elastic'i aç.
2. Sağ üst köşe → **⚙ Settings** → **Load Layout**.
3. Şu dosyayı seç:
   ```
   src/main/deploy/elastic/robot-yeni-elastic-layout.json
   ```
4. Alternatif: dosyayı Elastic penceresine **sürükle-bırak**.
5. Layout deploy ile robota da kopyalanır; robot başladığında Elastic otomatik olarak aynı layout'u sunar.

### Robot Bağlantısı
- Elastic DriverStation ile entegre çalışır; ayrı IP girişi gerekmez.
- NT (NetworkTables) bağlantısı için DriverStation zaten 10.80.92.2'ye bağlı olmalıdır.

---

## 2. Tab Açıklamaları

### Tab 1 — Teleop  *(ana maç ekranı)*

| Widget | Tür | Açıklama |
|---|---|---|
| Field | Saha haritası | Robotun gerçek zamanlı pozisyonu + otonom yolları |
| Robot Heading | Gyro | NavX yaw açısı (CCW pozitif) |
| Robot X / Y | Sayı | Odometri konumu (metre) |
| Vision Update | Boolean | Limelight poz güncellemesi aktif mi? |
| Vision Tag ID | Metin | Görünen AprilTag ID'si |
| FMSInfo | FMS | Takım, ittifak, maç durumu |
| Match Time | Sayaç | Kalan süre — sarı:30s, kırmızı:15s |
| FL/FR/RL/RR Output | Sayı | Sürüş motoru çıkış % (debug) |
| **Atici Hazir** | Boolean | Yeşil = atıcı hedef RPM'de, atış yapılabilir |
| **Homing Tamam** | Boolean | Yeşil = taret enkoderi sıfırlanmış, oto mod aktif |
| **Atis Modu** | Metin | `Limelight_Otomatik` / `Manuel_YAKIN` / `Manuel_ORTA` / `Manuel_UZAK` |
| Aktuel RPM | Sayı | Atıcı anlık hızı |

---

### Tab 2 — Atıcı  *(atış sistemi detayı)*

| Widget | Tür | Açıklama |
|---|---|---|
| Atici Hazir | Boolean | Yeşil = ±200 RPM içinde, ateş kilidini açar |
| Atis Modu | Metin | Aktif atış modu |
| Homing Tamam | Boolean | Taret referansı alınmış mı? |
| **Manuel Mod Secici** | Seçici | YAKIN / ORTA / UZAK — Limelight olmadığında kullanılır |
| Aktuel RPM | Sayı | Motordaki gerçek hız |
| Hedef RPM (PID) | Sayı | SparkMax'a gönderilen setpoint |
| Hesaplanan RPM | Sayı | Mesafe tablosundan hesaplanan değer |
| Hedef Mesafe (m) | Sayı | Limelight ölçümü (manuel modda NaN) |
| RPM Ayar (Ayarlama) | Sayı + Gönder | Sabit RPM fallback değeri — manuel test için |

---

### Tab 3 — Kontrolcü  *(kol tanılama ve profil seçimi)*

| Widget | Tür | Açıklama |
|---|---|---|
| **Profil Secici** | Seçici | PS4 Tam / PS4 Basit / Xbox |
| Aktif Profil | Metin | O an yüklü profil sınıf adı |
| Kontrolcu Bagli | Boolean | Yeşil = joystick bağlı, kırmızı = bağlantı kesildi |
| Eksen 0–5 | Sayı | Ham eksen değerleri (-1…+1) |
| Btn 1–12 | Boolean | Ham buton durumları (debug) |
| POV | Sayı | D-Pad açısı (0/90/180/270, -1 = basılı değil) |

---

### Tab 4 — Vision

| Widget | Tür | Açıklama |
|---|---|---|
| Has Target | Boolean | Limelight hedef görüyor mu? |
| Pose Valid | Boolean | MegaTag2 poz geçerli mi? |
| Config OK | Boolean | Pipeline/LED/camMode doğru yapılandırıldı mı? |
| Horizontal Offset | Sayı | `tx` — taret oto açısı bu değerden türetilir |
| Vertical Offset | Sayı | `ty` |
| Target Area | Sayı | Hedef piksel alanı |
| Distance To Target | Sayı | Hesaplanan mesafe (m) |
| Robot X/Y/Yaw | Sayı | Vision poz tahmini |
| Tag ID / Ambiguity | Sayı | Görünen tag ve belirsizlik skoru |
| Config Status | Metin | Limelight hata mesajı (sorun varsa) |

---

### Tab 5 — Turret

| Widget | Tür | Açıklama |
|---|---|---|
| Turret Angle | Sayı | Mevcut açı (°) — 0=ön, -90=limit switch |
| Target Angle | Sayı | OtomatikTaretKomutu'nun hedefi |
| Angle Error | Sayı | Hedef - mevcut (°) |
| Oto Mod | Metin | `HOMING_GEREKLI` / Limelight / Odometri |
| Limit Switch | Boolean | Yeşil = switch tetiklenmiş (-90° pozisyonu) |
| Homing Tamam | Boolean | Enkoder sıfırlandı mı? |
| Blk: Switch / Min / Max | Boolean | Kırmızı = o yönde hareket engellendi |
| Command Speed | Sayı | Motor hız komutu |
| Motor Cikisi / Akim / Sicaklik / Voltaj | Sayı | SparkMax telemetrisi |
| Enkoder Rot. | Sayı | Ham rotasyon değeri (disli öncesi) |
| Taret Hizi | Sayı + Gönder | Manuel dönüş hızı (varsayılan 0.04) |

---

### Tab 6 — Auto

| Widget | Tür | Açıklama |
|---|---|---|
| Auto Chooser | Seçici | PathPlanner otonom rutini |
| Field | Saha haritası | Yol önizlemesi |
| NavX Run/Status/Error | — | NavX doğrulama testi |

---

### Tab 7 — Ayarlama  *(kalibrasyon)*

Tüm alanlar **Submit butonlu** — değeri yaz, Enter veya butona bas.

| Widget | Varsayılan | Açıklama |
|---|---|---|
| Intake Speed | 0.75 | Alım CIM hız oranı |
| Conveyor Speed | 0.75 | Taşıyıcı CIM hız oranı |
| Shooter RPM | 4000 | `at()` komutunun sabit RPM hedefi |
| Turret Speed | 0.04 | L1/R1 manuel taret hızı |

---

## 3. Maç Öncesi Kontrol Listesi

Her maçtan önce şu adımları sırayla yap:

### A — Kontrolcü Seçimi  *(Tab: Kontrolcü)*
1. **Profil Secici** → o maçta kullanılacak kolu seç:
   - `PS4 Tam (Axis 4/5 analog)` — analog triggerli PS4 (varsayılan)
   - `PS4 Basit (analog trigger yok)` — eski/yedek PS4
   - `Xbox` — Xbox One/360

2. **Kontrolcu Bagli** kutucuğu yeşil olduğunu doğrula.
3. İstersen **Eksen 0–5** ve **Btn 1–12** değerlerine basarak doğru profil okunduğunu kontrol et.

### B — Atış Modu Seçimi  *(Tab: Atıcı)*
Limelight hedefi görürse otomatik RPM hesaplanır, seçim gerekmez.
Limelight yoksa veya bozuksa:

1. **Manuel Mod Secici** → sahada beklenen mesafeye göre seç:

   | Seçenek | Mesafe | RPM |
   |---|---|---|
   | `Yakın (~1.2 m)` | < 1.5 m | 2900 |
   | `Orta (~2.8 m)` | 2–3.5 m | 3700 |
   | `Uzak (~4.4 m)` | > 3.5 m | 4450 |

### C — Otonom Seçimi  *(Tab: Auto)*
1. **Auto Chooser** → istenen rutin.
2. Field widget'ında yol önizlemesini onayla.

### D — Taret Homing  *(robotu enable etmeden önce)*
1. Robot DISABLED modda.
2. **Taret** tabında `Homing Tamam = FALSE` (kırmızı) olduğunu doğrula.
3. Robot ENABLED → joystick'te **L2 (PS4 Tam/Basit)** veya **Back (Xbox)** butonuna bas.
4. Taret -90°'ye kayar, enkoder sıfırlanır.
5. `Homing Tamam` kutucuğu **yeşile** döner.
6. `Turret Angle` ~ -90.0 göstermeli.

---

## 4. Joystick Buton Yerleşimi

### PS4 Tam Profili  *(Axis 4/5 analog mevcut — varsayılan)*

| Giriş | PS4 | Eylem | Davranış |
|---|---|---|---|
| Sol Analog Y | — | İleri/geri sürüş | Sürekli |
| Sol Analog X | — | Yanal sürüş | Sürekli |
| Sağ Analog X | — | Dönüş | Sürekli |
| **Axis 4 (L2 analog)** | L2 | Atıcı spin-up | Basılı tutulurken |
| **Axis 5 (R2 analog)** | R2 | Ateş kilidini aç | Atıcı hazırsa taşıyıcı çalışır |
| Buton 1 | ■ Kare | Alım | Basılı tutulurken |
| Buton 2 | ✕ Çarpı | Geri at | Basılı tutulurken |
| Buton 3 | ● Daire | Taşıyıcı ters unjam (0.5 s) | Tek basış |
| Buton 4 | ▲ Üçgen | Taşıyıcı yukarı manuel | Basılı tutulurken |
| Buton 5 | L1 | Taret sola (fallback) | Basılı tutulurken |
| Buton 6 | R1 | Taret sağa (fallback) | Basılı tutulurken |
| Buton 7 | L2 dijital | Taret homing | Toggle (1.basış=başla, 2.=durdur) |
| Buton 8 | R2 dijital | Gyro sıfırla | Tek basış |

**Atış akışı:**
1. L2 analog basılı tut → atıcı dönmeye başlar.
2. Kol titreşirse → atıcı hedef RPM'de.
3. R2 analog basılı tut → taşıyıcı çalışır, top atar.

---

### PS4 Basit Profili  *(Axis 4/5 YOK — eski kol)*

| Giriş | PS4 | Eylem | Davranış |
|---|---|---|---|
| Sol Analog Y | — | İleri/geri sürüş | Sürekli |
| Sol Analog X | — | Yanal sürüş | Sürekli |
| Sağ Analog X | — | Dönüş | Sürekli |
| Buton 1 | ■ Kare | Alım | Basılı tutulurken |
| Buton 2 | ✕ Çarpı | Geri at | Basılı tutulurken |
| Buton 3 | ● Daire | Taşıyıcı ters unjam (0.5 s) | Tek basış |
| Buton 4 | ▲ Üçgen | Taret homing | Toggle |
| Buton 5 | L1 | Taret sola (fallback) | Basılı tutulurken |
| Buton 6 | R1 | Taret sağa (fallback) | Basılı tutulurken |
| **Buton 7** | **L2 dijital** | **Atıcı spin-up** | Basılı tutulurken |
| **Buton 8** | **R2 dijital** | **Ateş kilidini aç** | Atıcı hazırsa taşıyıcı çalışır |
| Buton 9 | Share | Taşıyıcı yukarı manuel | Basılı tutulurken |
| Buton 10 | Options | Gyro sıfırla | Tek basış |

---

### Xbox Profili

| Giriş | Xbox | Eylem | Davranış |
|---|---|---|---|
| Sol Analog Y | — | İleri/geri sürüş | Sürekli |
| Sol Analog X | — | Yanal sürüş | Sürekli |
| Sağ Analog X | — | Dönüş | Sürekli |
| **Axis 2 (LT analog)** | LT | Atıcı spin-up | Basılı tutulurken |
| **Axis 3 (RT analog)** | RT | Ateş kilidini aç | Atıcı hazırsa taşıyıcı çalışır |
| Buton 1 | A | Alım | Basılı tutulurken |
| Buton 2 | B | Geri at | Basılı tutulurken |
| Buton 3 | X | Taşıyıcı ters unjam (0.5 s) | Tek basış |
| Buton 4 | Y | Taşıyıcı yukarı manuel | Basılı tutulurken |
| Buton 5 | LB | Taret sola (fallback) | Basılı tutulurken |
| Buton 6 | RB | Taret sağa (fallback) | Basılı tutulurken |
| Buton 7 | Back | Taret homing | Toggle |
| Buton 8 | Start | Gyro sıfırla | Tek basış |

---

## 5. Atış Kilidinin Çalışması

```
Spin-up (L2/LT)  ──►  Atıcı döner
                           │
                    ±200 RPM içine girdiyse
                           │
                    Kol titreşir (0.6)
                           │
Ateş (R2/RT) de  ──►  Taşıyıcı çalışır ──► TOP ATAR
basılıysa
```

- Atıcı henüz hıza ulaşmamışsa R2/RT basılı olsa bile taşıyıcı **çalışmaz**.
- Spin-up bırakılırsa atıcı durur, titreşim kesilir.

---

## 6. Taret Otomatik/Manuel Modu

```
Homing yapılmadı
    └─► Taret hareketsiz (güvenli)

Homing tamamlandı
    ├─► Limelight hedef görüyor  ──► tx * KP ile açı hesaplanır (oto)
    └─► Limelight kör            ──► Odometri + hub konumu ile açı (oto)

L1 / R1 basılı  ──► Manuel mod (oto taret komutu kesilir)
                      Bırakınca oto mod geri döner
```

Taret OtoMod değerleri (Turret tabı):
- `Limelight` — Limelight tx'e göre
- `Odometri` — NavX + saha koordinatına göre
- `HOMING_GEREKLI` — homing yapılmamış, bekliyor

---

## 7. Sorun Giderme

| Sorun | Kontrol |
|---|---|
| Profil Secici boş | Deploy tamamlandı mı? Robot enable edildi mi? NT bağlantısını kontrol et. |
| Kontrolcu Bagli kırmızı | USB/Bluetooth bağlantısı kopmuş, yeniden tak. |
| Atici Hazir hiç yeşil olmuyor | `Ayarlama/AticiRPM` değeri motor max RPM'i aşıyor olabilir. `SURUS_DISI_MOTORLARI_ETKIN = true` mı? |
| Atis Modu "Manuel_ORTA" ama Limelight var | `Vision/HasTarget` FALSE ise kod Limelight'ı görmüyor; pipeline/LED kontrol et. |
| Homing Tamam hiç yeşil olmuyor | L2 basılı tutuldu mu? Limit switch DIO 9'a bağlı mı? |
| Taret sadece bir yöne dönüyor | `TARET_MOTOR_TERS` sabitini değiştir, yeniden deploy et. |
| Manuel Mod Secici etkisiz | Limelight hedefi görüyor; otomatik mod önceliklidir. Hedef yokken test et. |
