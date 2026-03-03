# FRC 2026 Robot Kullanıcı Rehberi

## 📋 İçindekiler

1. [Kontrolcüler](#kontrolcüler)
2. [Sürüş Kontrolleri](#sürüş-kontrolleri)
3. [Test Butonları](#test-butonları)
4. [Vision/AprilTag Kullanımı](#visionapriltag-kullanımı)
5. [SmartDashboard Göstergeleri](#smartdashboard-göstergeleri)
6. [Güvenlik İpuçları](#güvenlik-ipuçları)
7. [Sorun Giderme](#sorun-giderme)

---

## 🎮 Kontrolcüler

### Driver (Sürücü) - PS4 Controller (Port 0)
- **Sol Stick**: İleri/Geri ve Strafe (Sağ/Sol)
- **Sağ Stick**: Döndürme (Z ekseni)
- **Butonlar 1-5**: Test fonksiyonları

### Operator (Operatör) - Joystick (Port 1)
- Şu anda kullanımda değil
- Gelecekte intake/shooter/turret kontrolü için

---

## 🚗 Sürüş Kontrolleri

### Mekanum Sürüş Modu

Robot 4 tekerlekli mekanum sürüş sistemine sahiptir - her yöne hareket edebilir!

| Eksen | Fonksiyon | Axis No | Açıklama |
|------|-----------|---------|----------|
| Y Ekseni | İleri/Geri | 0 | Sol stick ileri/geri |
| X Ekseni | Strafe | 1 | Sol stick sağ/sol |
| Z Ekseni | Döndürme | 2 | Sağ stick sağ/sol |

### Alan Odaklı Sürüş (Field-Oriented Drive)

✅ **Her zaman açık** - Robotun başlığı yönünü korur

- Robot ne şekilde olursa olsun, stick yönü sahadaki sabit yönü temsil eder
- Stick yukarı = Robot her zaman "ilere" gider
- NavX jiroskop tarafından sağlanır

### Önemli Ayarlar

| Ayar | Değer | Açıklama |
|------|-------|----------|
| Deadband | 0.08 | Stick "ölü bölgesi" |
| Maksimum Hız | ~3.0 m/s | NEO motorlarla |
| Dönme Hızı | Yavaş | Kontrollü dönmeler için |

---

## 🔘 Test Butonları

⚠️ **Not**: Bu butonlar ROBOT DISABLED modundayken test için kullanılır.

### Button 1: Tüm Motorları Çalıştır
- Tüm 4 sürüş motorunu %30 güçle ileri çalıştırır
- Motor bağlantılarını test etmek için

### Button 2: Arka Sol Motor (ID 4)
- Arka sol NEO motoru %30 güçle çalıştırır
- CAN ID doğrulaması için

### Button 3: Arka Sağ Motor (ID 2)
- Arka sağ NEO motoru %30 güçle çalıştırır
- CAN ID doğrulaması için

### Button 4: Ön Sağ Motor (ID 3)
- Ön sağ NEO motoru %30 güçle çalıştırır
- CAN ID doğrulaması için

### Button 5: Pozisyon Reset (Vision)
- **IMPORTANT**: AprilTag ile robot pozisyonunu resetler
- Kullanım:
  1. Robotu bir AprilTag'in önüne koyun
  2. Button 5'e basın
  3. DriverStation'da "Pose reset from AprilTag X..." mesajı görünür
  4. Robot pozisyonu güncellenir

---

## 👁️ Vision/AprilTag Kullanımı

### Limelight 3 AprilTag Sistemi

Robot 2026 FRC sahasındaki 32 AprilTag'ı görebilir ve pozisyonunu belirleyebilir.

### Vision Göstergeleri (SmartDashboard)

| Gösterge | Anlamı | İdeal Değer |
|----------|--------|------------|
| `Vision/HasTarget` | AprilTag görünüyor mu | true (tag göründüğünde) |
| `Vision/PoseValid` | Pozisyon geçerli mi | true |
| `Vision/RobotX` | Sahada X pozisyonu (m) | 0-16.54 |
| `Vision/RobotY` | Sahada Y pozisyonu (m) | 0-8.21 |
| `Vision/RobotYaw` | Robot başlığı (derece) | -180 ila 180 |
| `Vision/TagID` | Görülen tag ID'si | 1-16 |
| `Vision/Ambiguity` | Algılama belirsizliği | < 0.2 (düşük iyi) |
| `Vision/FmapTagCount` | FMAP tag sayısı | 32 |
| `Drive/VisionUpdate` | Vision güncellemesi | true (sık sık) |

### Vision ile Pozisyon Alma

**Otomatik Odometry Güncellemeleri:**
- Robot her 50ms'de bir vision ölçümü alır
- Odometry drift'i otomatik düzeltilir
- Hem autonomous hem teleop'ta çalışır

**Manuel Pozisyon Reset (Button 5):**
```
1. Robotu bir AprilTag'in 1-3 metre önüne koyun
2. Limelight'ın tag'i gördüğünden emin olun (Vision/HasTarget = true)
3. Button 5'e basın
4. Robot pozisyonu o tag'e göre ayarlanır
```

### Sahadaki AprilTag'lar

**2026 FRC Andymark Sahası:**
- **Toplam**: 32 AprilTag (16 mavi, 16 kırmızı)
- **Tag Boyutu**: 6.5 inç (165.1mm)
- **Tag Ailesi**: 36h11

Önemli tag pozisyonları:
- **Tag 1-8**: Blue alliance tarafı
- **Tag 9-16**: Orta saha ve blue tarafı
- Red alliance tag'leri simetrik

---

## 📊 SmartDashboard Göstergeleri

### Robot Pozisyonu

| Gösterge | Açıklama |
|----------|----------|
| `Robot X` | Sahada X pozisyonu (metre) |
| `Robot Y` | Sahada Y pozisyonu (metre) |
| `Robot Heading` | Robot başlığı (derece) |

### Sürüş Motorları

| Gösterge | Açıklama |
|----------|----------|
| `Drive/FrontLeftOutput` | Ön sol motor gücü (% -1.0 ila 1.0) |
| `Drive/FrontRightOutput` | Ön sağ motor gücü |
| `Drive/RearLeftOutput` | Arka sol motor gücü |
| `Drive/RearRightOutput` | Arka sağ motor gücü |

### Motor Test Toggle'ları (Disabled Only)

| Toggle | Motor | Kullanım |
|--------|-------|----------|
| `MotorTest/FrontLeft` | Ön Sol (ID 1) | + veya + butonu |
| `MotorTest/FrontRight` | Ön Sağ (ID 2) | + veya - butonu |
| `MotorTest/RearLeft` | Arka Sol (ID 4) | + veya - butonu |
| `MotorTest/RearRight` | Arka Sağ (ID 3) | + veya - butonu |

### NavX Test Göstergeleri

| Gösterge | Açıklama |
|----------|----------|
| `NavXTest/Run` | Test başlat |
| `NavXTest/Status` | Test durumu |
| `NavXTest/ErrorCode` | Hata kodu (varsa) |

### PathPlanner

| Gösterge | Açıklama |
|----------|----------|
| `Auto Chooser` | Otonom modu seçici |
| `2026 FRC Field` | Sahane görselleştirme |

---

## ⚠️ Güvenlik İpuçları

### Robot Başlatma Öncesi

1. **Alan Kontrolü:**
   - Robotun etrafında boşluk olduğundan emin olun
   - Tüm kablolar güvenli bir şekilde bağlanmış olmalı
   - Pnömatik basıncı kontrol edin (varsa)

2. **Güvenli Bölge:**
   - Robot en az 2 metre radius'lık boş alana ihtiyaç duyar
   - Çocuklar ve izleyiciler güvenli mesafede kalmalı

3. **Acil Durum:**
   - E-Stop butonuna her zaman hazırlıklı olun
   - Space tuşu = Robot disable

### Sürüş Sırasında

1. **Hız Kontrolü:**
   - İlk kullanımda yavaş başlayın
   - Deadband (0.08) nedeniyle küçük stick hareketleri görülmeyebilir

2. **Mekanum Sürüş:**
   - Strafe (yana kayma) sürüşü farklı hissettirebilir
   - Alan odaklı sürüş robotun başlığına göre değildir

3. **Vision Kullanımı:**
   - AprilTag'ler net görünmeli
   - Güneş ışığı vision'ı etkileyebilir

### Pit Alanında

1. **Motor Test:**
   - Sadece DISABLED modda kullanın
   - %30 güç güvenli test seviyesidir

2. **Pozisyon Reset:**
   - Button 5 ile sadece Disabled modda yapın
   - AprilTag'in net göründüğünden emin olun

---

## 🔧 Sorun Giderme

### Robot Hareket Etmiyor

| Belirti | Olası Sebep | Çözüm |
|---------|-------------|-------|
| Tüm motorlar çalışmıyor | E-Stop basılı | E-Stop'u serbest bırakın, robot'u yeniden enable edin |
| Tek motor çalışmıyor | CAN bağlantısı | Motor ID'sini ve kabloları kontrol edin |
| Tekerlekler dönüyor ama robot gitmiyor | Tekerlek kayması | Zeminde yeterli traction yok |
| Garip sürüş davranışı | Odometry drift'i | Button 5 ile vision reset yapın |

### Vision Çalışmıyor

| Belirti | Olası Sebep | Çözüm |
|---------|-------------|-------|
| `Vision/HasTarget` = false | Tag görünmüyor | Limelight'ı tag'e doğru çevirin |
| `Vision/PoseValid` = false | Ambiguity yüksek | Tag'i daha yakından/net görün |
| `Vision/FmapTagCount` ≠ 32 | FMAP yüklenmemiş | Limelight web arayüzünden FMAP yükleyin |
| Yanlış pozisyon | Kamera ayarları | Camera height/pitch ayarlarını kontrol edin |

### SmartDashboard Verileri Yok

| Belirti | Olası Sebep | Çözüm |
|---------|-------------|-------|
| Göstergeler görünmüyor | NetworkTables bağlantısı | Robot bağlantısını kontrol edin |
| Eski veriler | Robot disable | Robot'u enable edin |
| NaN değerler | Sensör hatası | NavX ve encoder'ları kontrol edin |

### Sürüş Davranışı Sorunları

| Belirti | Olası Sebep | Çözüm |
|---------|-------------|-------|
| Robot sürekli dönüyor | NavX kayması | NavX validation testini çalıştırın |
| Stick geri tepmiyor | Deadband çok yüksek | Constants.java'dan deadband'i düşürün |
| Yavaş hız | Çok düşük güç | Drive scale ayarlarını kontrol edin |

---

## 📝 Hızlı Başlangıç Kontrol Listesi

### Maç Öncesi

- [ ] Robot gücünü açın
- [ ] DriverStation'dan bağlanın
- [ ] SmartDashboard'ı açın
- [ ] `Vision/FmapTagCount` = 32 kontrol edin
- [ ] `Robot X`, `Robot Y` değerlerini görün
- [ ] Motor test toggle'larını DISABLED modda test edin
- [ ] Robot'u enable edin
- [ ] Hafif sürüş testi yapın
- [ ] Button 5 ile pozisyon reset test edin

### Maç Sırası

- [ ] Autonomous öncesi: Button 5 ile pozisyon reset
- [ ] Autonomous: Auto Chooser'dan modu seçin
- [ ] Teleop başlangıcında: Vision çalıştığından emin olun
- [ ] Sürüş sırasında: `Drive/VisionUpdate` = true kontrol edin

### Maç Sonrası

- [ ] Robot'u disable edin
- [ ] Gücü kapatın
- [ ] Pil durumunu kontrol edin
- [ ] Hasar kontrolü yapın

---

## 📞 Destek

Sorun yaşarsanız:

1. **DriverStation Logları**: Hata mesajlarını kontrol edin
2. **SmartDashboard**: Vision ve drive verilerini inceleyin
3. **Limelight Web**: http://10.80.92.200:5801
4. **RI_Log**: Rio log dosyalarını kontrol edin

---

## 📚 Ek Kaynaklar

- **ROBOT_SETUP.md**: Donanım kurulum detayları
- **CLAUDE.md**: Proje genel bakış
- **WPILib Dokümantasyonu**: https://docs.wpilib.org/
- **Limelight Dokümantasyonu**: https://docs.limelightvision.io/

---

**Versiyon**: 2026 Sezonu
**Son Güncelleme**: Mart 2026
**Robot**: Mekanum Sürüş + AprilTag Vision
