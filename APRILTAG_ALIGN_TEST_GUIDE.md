# 🧪 AprilTag Alignment & PathPlanner Test Rehberi

## 📋 Hazırlanan Özellikler

### 1. ✅ AlignToAprilTagCommand
Robot otomatik olarak bir AprilTag'e yönlendirir.

### 2. ✅ PathPlanner Basit Path
"Simple Line Test Path" - 6 metre düz çizgi.

---

## 🎮 Buton Atamaları

| Button | Komut | Açıklama |
|--------|-------|----------|
| 1 | Motor Test (All) | Tüm motorlar %30 |
| 2 | Motor Test (Rear Left) | Arka sol motor |
| 3 | Motor Test (Rear Right) | Arka sağ motor |
| 4 | Motor Test (Front Right) | Ön sağ motor |
| 5 | Vision Reset | Pozisyon reset (AprilTag) |
| **6** | **AprilTag Align** | **Tag'e yönel (NEW)** |

---

## 🧪 Test 1: AprilTag Alignment (Button 6)

### Amaç
Robotun bir AprilTag'e otomatik olarak yönlendiğini test etmek.

### Gereksinimler
- ✅ Robot ve AprilTag (ID 1) hazır
- ✅ Limelight tag'i görebilmeli
- ✅ Yeterli boşluk (robot hareket edecek)

### Adımlar

1. **Robot hazırlığı**
   - Robot DISABLED modda
   - AprilTag'i (ID 1) robotun önüne koyun (2-3 metre)

2. **Vision kontrolü**
   - Limelight tag'e doğru çevirin
   - SmartDashboard kontrolü:
     - `Vision/HasTarget` = true ✓
     - `Vision/TagID` = 1 ✓
     - `Vision/PoseValid` = true ✓

3. **Test başlatma**
   - Robot ENABLED moduna alın
   - **Button 6'ya basılı tutun**
   - Robot tag'e doğru gitmeli

4. **Beklenen davranış**
   - Robot tag'e doğru ilerlemeli
   - Tag'in önünde durmalı (1 metre mesafede)
   - Tag'e doğru dönmeli

5. **Test sonucu**
   - [ ] Robot tag'e ulaştı
   - [ ] Doğru mesafede durdu
   - [ ] Doğru yönde durdu

### Parametreler

```java
AlignToAprilTagCommand(
    driveSubsystem,
    visionSubsystem,
    1,      // Tag ID 1
    1.0     // 1 metre önünde dur
);
```

### Ayarlar (İsteğe göre değiştirilebilir)

| Parametre | Şu anki değer | Açıklama |
|-----------|----------------|----------|
| Tag ID | 1 | Hedef AprilTag (1-16) |
| Mesafe | 1.0m | Tag'in önünde durma mesafesi |
| Tolerance | 15cm | Pozisyon hassasiyeti |
| Yaw tolerance | 5° | Açı hassasiyeti |

---

## 🧪 Test 2: PathPlanner - Basit Çizgi

### Amaç
Robotun basit bir PathPlanner path'ini takip ettiğini test etmek.

### Path Bilgileri

```
Path: "Simple Line Test Path"
Başlangıç: (2.0, 4.0) - Kuzey'ye bakıyor
Bitiş:     (8.0, 4.0) - Kuzey'ye bakıyor
Uzunluk:   6 metre düz çizgi
Hız:       3.0 m/s maksimum
```

### Adımlar

1. **Robot pozisyonu**
   - Robotu yaklaşık (2.0, 4.0) pozisyonuna koyun
   - Robot kuzey yönüne bakmalı (heading ≈ 0°)

2. **Pozisyon reset**
   - Yakın bir AprilTag bulun (ID 1 örneğin)
   - Button 5 ile pozisyon reset yapın

3. **Auto seçimi**
   - SmartDashboard → "Auto Chooser"
   - "Simple Line Test" seçin

4. **Test başlatma**
   - Robot DISABLED → ENABLED
   - Autonomous modunda path izlemeli
   - Robot (2.0, 4.0)'dan (8.0, 4.0)'a gitmeli

5. **Beklenen davranış**
   - Robot düz çizgi gitmeli
   - 6 metre kat etmeli
   - Son duruşta durmalı

6. **Test sonucu**
   - [ ] Path başlandı
   - [ ] Robot doğru hızlandı/yavaşladı
   - [ ] Hedef pozisyona ulaştı
   - [ ] Odometry drift varsa, vision düzeltti

### Auto Seçenekleri

SmartDashboard "Auto Chooser" şunları gösterecek:

1. **Simple Line Test** (YENİ)
   - 6 metre düz çizgi

2. **Diagonal 180Deg Test**
   - Mevcut test path'i

3. **2M 180Deg Test** & **2M Test Path**
   - Diğer test path'leri

---

## 🧪 Test 3: Combined (Align + Path)

### Amaç
İki özelliği birlikte test etmek.

### Senaryo

```
1. Robotu başlangıçtan 2 metre uzağa koyun
2. Button 6 ile AprilTag'e yönelin
3. Pozisyon doğrulandı
4. Simple Line Test auto'sunu çalıştırın
5. Vision güncellemeleriyle path takibi
```

---

## 📊 SmartDashboard Göstergeleri

### Test Sırasında İzleme

| Gösterge | Test 1 | Test 2 | Test 3 |
|----------|--------|--------|--------|
| `Vision/HasTarget` | ✓ Gerekli | - | ✓ Gerekli |
| `Vision/TagID` | 1 | - | Herhangi |
| `Vision/PoseValid` | ✓ Gerekli | - | ✓ Gerekli |
| `Drive/VisionUpdate` | - | ✓ Otomatik | ✓ Otomatik |
| `Robot X` | Değişiyor | 2.0→8.0 | Değişiyor |
| `Robot Y` | ≈ 4.0 | ≈ 4.0 | Değişiyor |
| `Robot Heading` | Tag'e doğru | ≈ 0° | Tag'e doğru |

---

## 🔧 Sorun Giderme

### Button 6 Çalışmıyor

| Sorun | Çözüm |
|-------|--------|
| Robot gitmiyor | Tag görünmüyor mu? (`Vision/HasTarget`) |
| Garip yön | Robot yavaşça dönerse normal (yerçekimi merkezi) |
| Titreme | PID değerlerini ayarlamanız gerekebilir |
| Durmuyor | Tolerance değeri çok düşük olabilir |

### PathPlanner Çalışmıyor

| Sorun | Çözüm |
|-------|--------|
| Auto seçilmiyor | SmartDashboard "Auto Chooser" kontrol edin |
| Robot başlamıyor | Başlangıç pozisyonu yanlış olabilir |
| Yanlış yere gitti | Pozisyon reset yapın (Button 5) |
| Path takip etmiyor | Odometry drift'i var, vision düzeltmeli |

---

## 📝 Test Sonuçları Formu

### Test Tarihi: _____/_____/_____

### Test 1: AprilTag Alignment

| Kontroller | Sonuç | Notlar |
|-----------|--------|-------|
| Tag algılanıyor | ☐ Pass ☐ Fail | |
| Robot tag'e gidiyor | ☐ Pass ☐ Fail | |
| Doğru mesafede durdu | ☐ Pass ☐ Fail | |
| Doğru yönde durdu | ☐ Pass ☐ Fail | |

### Test 2: PathPlanner Basit Path

| Kontroller | Sonuç | Notlar |
|-----------|--------|-------|
| Auto başladı | ☐ Pass ☐ Fail | |
| Path takip edildi | ☐ Pass ☐ Fail | |
| Hedefe ulaştı | ☐ Pass ☐ Fail | |
| Vision güncellemeleri | ☐ Pass ☐ Fail | |

### Genel Değerlendirme

- [ ] Test 1 başarılı
- [ ] Test 2 başarılı
- [ ] Her iki özellik birlikte çalışıyor

### Bulunan Sorunlar

1. _____________________________________________________
2. _____________________________________________________
3. _____________________________________________________

---

## 🚀 Deploy Adımları

```bash
# 1. Robot'u açın ve bağlanın
ping 10.80.92.2

# 2. Kod deploy edin
./gradlew deploy

# 3. DriverStation'dan enable edin
# 4. Teste başlayın!
```

---

## 💡 İleri Düzeltmeler

### Test 1 Sonrası

Eğer AprilTag alignment çalışmıyorsa:

1. **PID değerlerini tune edin** (AlignToAprilTagCommand.java)
   ```java
   xController = new PIDController(2.0, 0, 0.1);    // Artır/Azalt
   yController = new PIDController(2.0, 0, 0.1);    // Artır/Azalt
   thetaController = new PIDController(1.5, 0, 0.05); // Artır/Azalt
   ```

2. **Hedef mesafeyi değiştirin**
   ```java
   // RobotContainer.java, Button 6
   new AlignToAprilTagCommand(
       driveSubsystem,
       visionSubsystem,
       1,      // Tag ID'yi değiştir
       0.5     // Mesafeyi değiştir
   )
   ```

### Test 2 Sonrası

Eğer PathPlanner path takibi sorunluysa:

1. **Başlangıç pozisyonu**
   - Robotu (2.0, 4.0)'a daha doğru koyun
   - Button 5 ile reset yapın

2. **Path ayarları**
   - Hızı düşürün (PathPlanner GUI'de)
   - Acceleration'u düşürün

---

## 📚 Referanslar

- **AlignToAprilTagCommand.java**: Vision alignment kodu
- **Simple Line Test Path.path**: PathPlanner path dosyası
- **Simple Line Test.auto**: Auto dosyası
- **ROBOT_USER_GUIDE.md**: Genel kullanıcı rehberi
- **APRILTAG_TEST_PLAN.md**: Detaylı test planı

---

**Kod hazır, test için bekleniyor!** 🎯
