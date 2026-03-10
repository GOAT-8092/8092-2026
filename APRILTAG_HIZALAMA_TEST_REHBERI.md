# AprilTag Alignment ve PathPlanner Test Rehberi

## Hazırlanan Özellikler

### 1. AlignToAprilTagCommand
Robot otomatik olarak bir AprilTag'e yönlendirir. Button 6 (R1) ile tetiklenir, basılı tutulduğu sürece çalışır.

### 2. RotateToAprilTag360Command
Robot bir AprilTag'e yönelik 360 derece dönüp kilitlenir. Button 7 (L2) ile tetiklenir, bir kez basılır ve kilitlenir.

### 3. TrackAprilTagCommand
Robot bir AprilTag'i sürekli takip eder ve belirtilen mesafeyi korur. Button 8 (R2) ile tetiklenir, basılı tutulduğu sürece çalışır.

### 4. PathPlanner Basit Path
"Simple Line Test Path" - 6 metre düz çizgi.

---

## Buton Atamaları

| Buton | PS4 Tusu | Komut | Açıklama |
|-------|----------|-------|----------|
| 1 | Square | Motor Test (On Sol) | On Sol motor (CAN ID 2) %30 |
| 2 | Cross | Motor Test (Arka Sol) | Arka Sol motor (CAN ID 3) %30 |
| 3 | Circle | Motor Test (Arka Sag) | Arka Sag motor (CAN ID 1) %30 |
| 4 | Triangle | Motor Test (On Sag) | On Sag motor (CAN ID 4) %30 |
| 5 | L1 | Vision Reset | Pozisyon reset (AprilTag) |
| 6 | R1 | AlignToAprilTagCommand | Tag 1'e hizala, 1.0m mesafe (basili tut) |
| 7 | L2 | RotateToAprilTag360Command | Tag 1'e yonelik 360° don (bir kez bas, kilitleniyor) |
| 8 | R2 | TrackAprilTagCommand | Tag 1'i sureklitakip et, 1.0m mesafe (basili tut) |

---

## CAN ID - Motor Eslesmesi

| CAN ID | Konum | Ters mi |
|--------|-------|---------|
| 1 | Arka Sag | Evet |
| 2 | On Sol | Hayir |
| 3 | Arka Sol | Hayir |
| 4 | On Sag | Evet |

---

## Test 1: AprilTag Hizalama (Button 6 / R1)

### Amac
Robotun bir AprilTag'e otomatik olarak yonlendigini test etmek.

### Gereksinimler
- Robot ve AprilTag (ID 1) hazir
- Limelight tag'i gorebilmeli
- Yeterli bosluk (robot hareket edecek)

### Adimlar

1. **Robot hazirliği**
   - Robot DISABLED modda
   - AprilTag (ID 1) robotun onune koyun (2-3 metre)

2. **Vision kontrolu**
   - Limelight tag'e dogru cevirin
   - SmartDashboard kontrolu:
     - `Vision/HasTarget` = true
     - `Vision/TagID` = 1
     - `Vision/PoseValid` = true

3. **Test baslatma**
   - Robot ENABLED moduna alin
   - Button 6 (R1)'a basili tutun
   - Robot tag'e dogru gitmeli

4. **Beklenen davranis**
   - Robot tag'e dogru ilerlemeli
   - Tag'in onunde durmalı (1 metre mesafede)
   - Tag'e dogru donmeli

5. **Button'i birakin**
   - Komut durur, robot normal joystick kontrolune doner

6. **Test sonucu**
   - [ ] Robot tag'e ulasti
   - [ ] Dogru mesafede durdu (~1.0m)
   - [ ] Dogru yonde durdu

### Parametreler

```java
AlignToAprilTagCommand(
    driveSubsystem,
    visionSubsystem,
    1,      // Tag ID 1
    1.0     // 1 metre onunde dur
);
```

### Ayarlar (Isteğe gore degistirilebilir)

| Parametre | Su anki deger | Aciklama |
|-----------|----------------|----------|
| Tag ID | 1 | Hedef AprilTag |
| Mesafe | 1.0m | Tag'in onunde durma mesafesi |
| Tolerance | 15cm | Pozisyon hassasiyeti |
| Yaw tolerance | 5° | Aci hassasiyeti |

---

## Test 2: 360 Derece Donme (Button 7 / L2)

### Amac
Robotun Tag 1'e yonelik donup kilitlendigini dogrulamak.

### Adimlar

1. **Hazirlik**
   - Robot ENABLED modda
   - AprilTag ID 1 gorunur

2. **Button 7 (L2)'ye bir kez basin**
   - Komut kilitlenir (basili tutmak gerekmez)
   - Robot tag'e yonelik doner

3. **Kilitlenme dogrulama**
   - Robot donus tamamlandiktan sonra o yonde kilitli kalmalı
   - Baska bir komut verilene kadar bu durum surer

4. **Test sonucu**
   - [ ] Robot tag yonune dondü
   - [ ] Komut kilitlendi (basili tutmak gerekmedi)

---

## Test 3: Surekli Takip (Button 8 / R2)

### Amac
Robotun Tag 1'i 1.0m mesafede surekli takip ettigini dogrulamak.

### Adimlar

1. **Hazirlik**
   - Robot ENABLED modda
   - AprilTag ID 1 gorunur

2. **Button 8 (R2)'ye basili tutun**
   - Robot tag'i takip etmeye baslar
   - Tag hareket ettirilirse robot da takip eder

3. **Tag'i hareket ettirin**
   - AprilTag'i yavascayavaşça saga/sola tasiyin
   - Robot tag'i izlemeli (1.0m mesafeyi koruyarak)

4. **Button'i birakin**
   - Takip durur, robot normal kontrole doner

5. **Test sonucu**
   - [ ] Robot tag'i takip etti
   - [ ] 1.0m mesafe korundü
   - [ ] Birakinca durdu

### Parametreler

```java
TrackAprilTagCommand(
    driveSubsystem,
    visionSubsystem,
    1,      // Tag ID 1
    1.0     // 1 metre mesafe koru
);
```

---

## Test 4: PathPlanner - Basit Çizgi

### Amac
Robotun basit bir PathPlanner path'ini takip ettigini test etmek.

### Path Bilgileri

```
Path: "Simple Line Test Path"
Baslangic: (2.0, 4.0) - Kuzey'ye bakiyor
Bitis:     (8.0, 4.0) - Kuzey'ye bakiyor
Uzunluk:   6 metre duz cizgi
Hiz:       3.0 m/s maksimum
```

### Adimlar

1. **Robot pozisyonu**
   - Robotu yaklasik (2.0, 4.0) pozisyonuna koyun
   - Robot kuzey yonune bakmali (heading ≈ 0°)

2. **Pozisyon reset**
   - Yakin bir AprilTag bulun
   - Button 5 (L1) ile pozisyon reset yapin

3. **Auto secimi**
   - SmartDashboard: "Auto Chooser"
   - "Simple Line Test" secin

4. **Test baslatma**
   - Robot DISABLED: ENABLED
   - Autonomous modunda path izlemeli
   - Robot (2.0, 4.0)'dan (8.0, 4.0)'a gitmeli

5. **Beklenen davranis**
   - Robot duz cizgi gitmeli
   - 6 metre kat etmeli
   - Son duruşta durmalı

6. **Test sonucu**
   - [ ] Path baslandi
   - [ ] Robot dogru hizlandi/yavasladi
   - [ ] Hedef pozisyona ulasti
   - [ ] Odometry drift varsa, vision duzelttti

### Auto Secenekleri

SmartDashboard "Auto Chooser" sunlari gosterecek:

1. Simple Line Test - 6 metre duz cizgi
2. Diagonal 180Deg Test - Mevcut test path'i
3. 2M 180Deg Test ve 2M Test Path - Diger test path'leri

---

## Test 5: Bilesik Test (Hizala + Path)

### Amac
Ozellikleri birlikte test etmek.

### Senaryo

```
1. Robotu baslangictan 2 metre uzaga koyun
2. Button 6 (R1) ile AprilTag'e yonelin
3. Pozisyon dogrulandi
4. Simple Line Test auto'sunu calistirin
5. Vision guncellemeleriyle path takibi
```

---

## SmartDashboard Gostergeleri

### Test Sirasinda Izleme

| Gosterge | Test 1 (Hizala) | Test 2 (360°) | Test 3 (Takip) | Test 4 (Path) |
|----------|-----------------|----------------|-----------------|---------------|
| `Vision/HasTarget` | Gerekli | Gerekli | Gerekli | - |
| `Vision/TagID` | 1 | 1 | 1 | - |
| `Vision/PoseValid` | Gerekli | - | Gerekli | - |
| `Drive/VisionUpdate` | - | - | - | Otomatik |
| `Robot X` | Degisiyor | Sabit | Degisiyor | 2.0->8.0 |
| `Robot Y` | ≈ 4.0 | ≈ 4.0 | Degisiyor | ≈ 4.0 |
| `Robot Heading` | Tag'e dogru | Tag'e dogru | Tag'e dogru | ≈ 0° |

---

## Sorun Giderme

### Button 6 (R1) Calismiyor

| Sorun | Cozum |
|-------|--------|
| Robot gitmiyor | Tag gorunuyor mu? (`Vision/HasTarget` kontrol edin) |
| Garip yon | Robot yavasca donerse normal (aci duzeltmesi) |
| Titreme | PID degerlerini ayarlamaniz gerekebilir |
| Durmuyor | Tolerance degeri cok dusuk olabilir |

### Button 7 (L2) Kilitlenmiyor

| Sorun | Cozum |
|-------|--------|
| Komut baslamıyor | Robot ENABLED modda mi? |
| Donmuyor | Tag gorunuyor mu? |
| Kilitlenmiyor | Bir kez basip biraktiginizdan emin olun |

### Button 8 (R2) Takip Etmiyor

| Sorun | Cozum |
|-------|--------|
| Robot baslamiyor | Button basili mi tutuluyor? |
| Mesafe yanlis | 1.0m kalibrasyon kontrolü edin |
| Takip kopuyor | Tag gorunur alanda mi? |

### PathPlanner Calismiyor

| Sorun | Cozum |
|-------|--------|
| Auto seçilmiyor | SmartDashboard "Auto Chooser" kontrol edin |
| Robot baslamiyor | Baslangic pozisyonu yanlis olabilir |
| Yanlis yere gitti | Pozisyon reset yapin (Button 5 / L1) |
| Path takip etmiyor | Odometry drift var, vision duzeltmeli |

---

## Test Sonuclari Formu

### Test Tarihi: _____/_____/_____

### Test 1: AprilTag Hizalama (Button 6 / R1)

| Kontroller | Sonuc | Notlar |
|-----------|--------|-------|
| Tag algilaniyor | Pass / Fail | |
| Robot tag'e gidiyor | Pass / Fail | |
| Dogru mesafede durdu | Pass / Fail | |
| Dogru yonde durdu | Pass / Fail | |

### Test 2: 360 Derece Donme (Button 7 / L2)

| Kontroller | Sonuc | Notlar |
|-----------|--------|-------|
| Robot tag yonune dondü | Pass / Fail | |
| Komut kilitlendi | Pass / Fail | |

### Test 3: Surekli Takip (Button 8 / R2)

| Kontroller | Sonuc | Notlar |
|-----------|--------|-------|
| Robot tag'i takip etti | Pass / Fail | |
| 1.0m mesafe korundü | Pass / Fail | |
| Birakinca durdu | Pass / Fail | |

### Test 4: PathPlanner Basit Path

| Kontroller | Sonuc | Notlar |
|-----------|--------|-------|
| Auto basladi | Pass / Fail | |
| Path takip edildi | Pass / Fail | |
| Hedefe ulasti | Pass / Fail | |
| Vision guncellemeleri | Pass / Fail | |

### Genel Degerlendirme

- [ ] Test 1 (Hizalama) basarili
- [ ] Test 2 (360° Donme) basarili
- [ ] Test 3 (Surekli Takip) basarili
- [ ] Test 4 (PathPlanner) basarili
- [ ] Tum ozellikler birlikte calisiyor

### Bulunan Sorunlar

1. _____________________________________________________
2. _____________________________________________________
3. _____________________________________________________

---

## Deploy Adimlari

```bash
# 1. Robot'u acin ve baglanin
ping 10.80.92.2

# 2. Kod derleyin
./gradlew build

# 3. Kod deploy edin
./gradlew deploy

# 4. DriverStation'dan enable edin
# 5. Teste baslayin!
```

---

## Ileri Duzeltmeler

### Hizalama Komutu PID Tune

Eger AprilTag alignment calismiyor veya hassasiyet yetersizse:

```java
// AlignToAprilTagCommand.java icinde
xController = new PIDController(2.0, 0, 0.1);    // Artir/Azalt
yController = new PIDController(2.0, 0, 0.1);    // Artir/Azalt
thetaController = new PIDController(1.5, 0, 0.05); // Artir/Azalt
```

### Hedef Tag veya Mesafeyi Degistirme

```java
// RobotContainer.java, Button 6 (R1)
new AlignToAprilTagCommand(
    driveSubsystem,
    visionSubsystem,
    1,      // Tag ID'yi degistir (ornek: 12 veya 15)
    0.5     // Mesafeyi degistir (metre)
)
```

### PathPlanner Path Ayarlari

Eger path takibi sorunluysa:
1. Baslangic pozisyonunu duzgun ayarlayin ve Button 5 (L1) ile reset yapin
2. PathPlanner GUI'de hizi dusurün
3. Acceleration degerini dusurün

---

## Referanslar

- AlignToAprilTagCommand.java: Vision alignment kodu
- RotateToAprilTag360Command.java: 360 derece donme kodu
- TrackAprilTagCommand.java: Surekli takip kodu
- Simple Line Test Path.path: PathPlanner path dosyasi
- Simple Line Test.auto: Auto dosyasi
- APRILTAG_TEST_PLANI.md: Detayli test plani

---

Ag bilgileri:
- roboRIO: 10.80.92.2
- Limelight web arayuzu: http://10.80.92.200:5801
- Limelight pipeline: 0
- FMAP: FRC2026_ANDYMARK.fmap (32 tag)
