# 🧪 AprilTag Test Planı

## Test Öncesi Hazırlık

### Robot Bağlantısı

- [ ] Robotu güç kaynağına bağlayın
- [ ] Robot'u açın (RIO power button)
- [ ] Bilgisayarı robot ağına bağlayın:
  - USB-C kablo ile RIO'ya, VEYA
  - Robot ağına (10.80.92.x)
- [ ] Ping testi: `ping 10.80.92.2`
- [ ] DriverStation'da robot bağlantısını doğrulayın

### Kod Deploy

```bash
# Kod derleyin
./gradlew build

# Robota deploy edin
./gradlew deploy

# Başarılı mesajını bekleyin: "BUILD SUCCESSFUL"
```

### Donanım Kontrolleri

- [ ] Limelight 3 güç alıyor (LED yanıyor)
- [ ] NavX jiroskop çalışıyor (mavi ışık)
- [ ] Motor kontrolörleri yeşil LED gösteriyor
- [ ] Pnömatik (varsa) basınçlı

---

## Adım 1: Temel Bağlantı Testi

### Amaç
Robot ve bilgisayar arasındaki tüm sistemlerin çalıştığını doğrulamak

### Test Adımları

1. **DriverStation'ı Açın**
   - Team Number: 8092
   - Robot bağlantısını bekleyin
   - "Communications": ✓ GREEN
   - "Robot Code": ✓ GREEN

2. **SmartDashboard'ı Açın**
   - Vision göstergelerini arayın:
     - `Vision/FmapTagCount` = 32
     - `Vision/FmapStatus` = "FMAP: 32 tags, Size: Known"

3. **Sonuçları Kaydedin**
   - [ ] DriverStation bağlantısı ✓
   - [ ] SmartDashboard göstergeleri ✓
   - [ ] FMAP yüklü (32 tags) ✓

---

## Adım 2: AprilTag Algılama Testi

### Amaç
Limelight'ın AprilTag'leri doğru algıladığını doğrulamak

### Test Adımları

1. **Robot DISABLED modda olsun**

2. **AprilTag'i Hazırlayın**
   - AprilTag basılı kağıdı/tableti
   - Tag boyutu: 6.5 inç (165.1mm)
   - Tag ID'sini bilin (örneğin: ID 1)

3. **Tag'i Limelight'a Gösterin**
   - Mesafe: 1-3 metre
   - Açı: Dik ( Limelight tag'e doğru bakmalı)
   - Işık: Yeterli aydınlatma

4. **SmartDashboard Göstergelerini İzleyin**

   | Gösterge | Beklenen Değer |
   |----------|----------------|
   | `Vision/HasTarget` | true |
   | `Vision/TagID` | Görülen tag ID (örn: 1) |
   | `Vision/HorizontalOffset` | -27 ila 27 derece |
   | `Vision/VerticalOffset` | -20.5 ila 20.5 derece |
   | `Vision/TargetArea` | 0 ila 1 (tag'e yakın = büyük) |
   | `Vision/PoseValid` | true |
   | `Vision/Ambiguity` | < 0.2 |

5. **Farklı Mesafelerde Test Edin**
   - 1 metre → TargetArea büyük (~0.5-0.8)
   - 2 metre → TargetArea orta (~0.2-0.4)
   - 3 metre → TargetArea küçük (~0.1-0.2)

6. **Sonuçları Kaydedin**
   - [ ] Tag algılanıyor (HasTarget = true) ✓
   - [ ] Tag ID doğru ✓
   - [ ] Ambiguity düşük (< 0.2) ✓
   - [ ] Farklı mesafelerde çalışıyor ✓

---

## Adım 3: Vision Pozisyon Testi

### Amaç
Limelight'ın doğru pozisyon hesapladığını doğrulamak

### Test Adımları

1. **Robotu Bilinen Pozisyona Koyun**
   - Örneğin: Sahanın sol alt köşesi
   - X ≈ 1.0m, Y ≈ 1.0m
   - Robot yaklaşık kuzeye bakmalı (yaw ≈ 0°)

2. **AprilTag'e Bakın**
   - Yakın bir tag bulun (1-2 metre)
   - SmartDashboard'da `Vision/PoseValid` = true bekleyin

3. **Pozisyon Değerlerini Okuyun**
   - `Vision/RobotX`: __.__ m
   - `Vision/RobotY`: __.__ m
   - `Vision/RobotYaw`: __°

4. **Beklenen Değerleri Kontrol Edin**
   - RobotX: 0-16.54 arası (saha boyu)
   - RobotY: 0-8.21 arası (saha genişliği)
   - RobotYaw: -180 ila 180 derece

5. **Gerçek Mesafeyi Ölçün**
   - Tag'e olan gerçek mesafeyi ölçün (metre cinsinden)
   - `getDistanceToTarget()` değerini kontrol edin

6. **Sonuçları Kaydedin**
   - [ ] Pozisyon mantıklı değerler ✓
   - [ ] X/Y saha sınırları içinde ✓
   - [ ] Yaw makul (robot gerçekten o yönde bakıyor mu) ✓

---

## Adım 4: Vision Reset Testi (Button 5)

### Amaç
Button 5 ile pozisyon reset fonksiyonunu test etmek

### Test Adımları

1. **Robot DISABLED Modda**
   - Güvenli bir konumda

2. **AprilTag'e Bakın**
   - Tag görünür olmalı
   - `Vision/HasTarget` = true
   - `Vision/PoseValid` = true

3. **Mevcut Pozisyonu Kaydedin**
   - `Robot X` (öncesi): _____
   - `Robot Y` (öncesi): _____
   - `Robot Heading` (öncesi): _____°

4. **Button 5'e Basın**
   - PS4 controller'da Button 5'e basın
   - DriverStation'da mesaj bekleyin:
     ```
     Pose reset from AprilTag X at (Y.ZZ, W.WW)
     ```

5. **Yeni Pozisyonu Kontrol Edin**
   - `Robot X` (sonrası): _____
   - `Robot Y` (sonrası): _____
   - `Robot Heading` (sonrası): _____°

6. **Pozisyon Değişti mi?**
   - Değerler güncellendiyse ✓
   - DriverStation mesajı varsa ✓
   - `Drive/VisionUpdate` = true ise ✓

7. **Sonuçları Kaydedin**
   - [ ] DriverStation mesajı göründü ✓
   - [ ] Pozisyon güncellendi ✓
   - [ ] Vision update çalışıyor ✓

---

## Adım 5: Motor Testleri (DISABLED Mod)

### Amaç
Motorların doğru çalıştığını doğrulamak

### Test Adımları

1. **Robot DISABLED Modda**
2. **Güvenli Alan**: Robot etrafında boşluk

3. **Button 1: Tüm Motorlar**
   - Button 1'e basılı tutun
   - Tüm 4 tekerlek dönmeli
   - Yön: İleri (tüm tekerlekler robotun "önüne" doğru)
   - Çıkış: Button 1'i bırakın
   - Sonuç: Tüm motorlar çalışıyor ✓

4. **Button 2: Arka Sol Motor (ID 4)**
   - Button 2'ye basılı tutun
   - Sadece arka sol tekerlek dönmeli
   - Sonuç: Arka sol motor çalışıyor ✓

5. **Button 3: Arka Sağ Motor (ID 3)**
   - Button 3'e basılı tutun
   - Sadece arka sağ tekerlek dönmeli
   - Sonuç: Arka sağ motor çalışıyor ✓

6. **Button 4: Ön Sağ Motor (ID 2)**
   - Button 4'e basılı tutun
   - Sadece ön sağ tekerlek dönmeli
   - Sonuç: Ön sağ motor çalışıyor ✓

---

## Adım 6: Sürüş Testi (ENABLED Mod)

### Amaç
Mekanum sürüş ve alan odaklı sürüşü test etmek

### Test Adımları

1. **Güvenlik**
   - Robot etrafında 2m boşluk
   - Acil durum butonuna hazırlıklı olun

2. **Robot ENABLED Edin**
   - DriverStation'da "Enable" butonu
   - Space tuşu (klavyede)

3. **Temel Sürüş**
   - Sol stick yukarı → Robot ileri gider
   - Sol stick aşağı → Robot geri gider
   - Sol stick sağa → Robot sağa strafe
   - Sol stick sola → Robot sola strafe
   - Sağ stick sağa → Robot saat yönünde döner
   - Sağ stick sola → Robot saat yönünün tersine döner

4. **Alan Odaklı Sürüş Testi**
   - Robotu 90° döndürün (sağ stick)
   - Sol stick yukarı → Robot hala aynı yöne (ileri) gider
   - Farklı başlıklar için test edin

5. **Vision Update Testi**
   - Sürerken bir AprilTag'e doğru gidin
   - `Drive/VisionUpdate` = true olmalı
   - Robot pozisyonu güncellenmeli

---

## Adım 7: Odometry Drift Testi

### Amaç
Vision'un odometry drift'ini düzelttiğini doğrulamak

### Test Adımları

1. **Başlangıç Pozisyonu**
   - Robotu bir başlangıç noktasına koyun
   - Button 5 ile pozisyon reset edin
   - `Robot X`, `Robot Y` değerlerini kaydedin

2. **Kare Yapın**
   - Robot yaklaşık 2m x 2m kare sürün
   - 4 köşe: (0,0) → (2,0) → (2,2) → (0,2) → (0,0)

3. **Son Pozisyonu Karşılaştırın**
   - `Robot X` (son): _____
   - `Robot Y` (son): _____
   - Hata: _____ metre

4. **Vision Düzeltmesini Kontrol Edin**
   - Sürerken `Drive/VisionUpdate` değerleri
   - Vision update sayısı (her 50ms'de bir)
   - Drift azaldı mı?

---

## Test Sonuçları Formu

### Test Tarihi: _____/_____/_____
### Testçi: _____________

| Test | Sonuç | Notlar |
|------|--------|-------|
| Robot Bağlantısı | ☐ Pass ☐ Fail | |
| FMAP Yüklü (32 tags) | ☐ Pass ☐ Fail | |
| AprilTag Algılama | ☐ Pass ☐ Fail | |
| Vision Pozisyon | ☐ Pass ☐ Fail | |
| Button 5 Reset | ☐ Pass ☐ Fail | |
| Motor Testleri | ☐ Pass ☐ Fail | |
| Temel Sürüş | ☐ Pass ☐ Fail | |
| Alan Odaklı Sürüş | ☐ Pass ☐ Fail | |
| Odometry Drift | ☐ Pass ☐ Fail | |

### Genel Değerlendirme: ☐ Başarılı ☐ Kısmen Başarılı ☐ Başarısız

### Bulunan Sorunlar:

1. _____________________________________________________
2. _____________________________________________________
3. _____________________________________________________

### Çözülen Sorunlar:

1. _____________________________________________________
2. _____________________________________________________
3. _____________________________________________________

---

## Sorun Giderme

### Sorun: Vision/HasTarget = false

**Çözümler:**
1. Tag'i daha yakına getirin (1-2m)
2. Limelight'ı tag'e doğru çevirin
3. Işık seviyesini kontrol edin
4. Tag'in düzgün basıldığından emin olun
5. Tag boyutunu kontrol edin (6.5 inç)

### Sorun: Vision/PoseValid = false

**Çözümler:**
1. Ambiguity değerini kontrol edin (< 0.2 olmalı)
2. Tag'i daha merkeze yerleştirin
3. Tag'i daha yakın veya net görün
4. Limelight web arayüzünden kontrol edin

### Sorun: Button 5 çalışmıyor

**Çözümler:**
1. DriverStation loglarını kontrol edin
2. Vision/HasTarget = true olmalı
3. Robot DISABLED modda olmalı
4. VisionSubsystem'in çalıştığını kontrol edin

### Sorun: Motorlar dönmüyor

**Çözümler:**
1. E-Stop kontrol edin
2. Robot enable edilmiş mi?
3. Motor ID'lerini kontrol edin
4. CAN kablolarını kontrol edin

---

## Sonraki Adımlar

Test başarılı olduktan sonra:

- [ ] Gerçek sahadaki AprilTag'leri test edin
- [ ] Otomatik modlarda vision kullanın
- [ ] Kamera ayarlarını robot üzerinde ölçün ve güncelleyin
- [ ] PID değerlerini tune edin (gerekirse)

---

**Test sırasında sorular için:**
- SmartDashboard göstergelerini takip edin
- DriverStation loglarını kontrol edin
- Limelight web arayüzü: http://10.80.92.200:5801
