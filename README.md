# Robot-Yeni

FRC robot yazilimi (Java + WPILib + GradleRIO) projesi.

## Icerik
- `src/main/java`: Robot kodu (alt sistemler, komutlar, sabitler)
- `src/test/java`: Birim ve dogrulama testleri
- `vendordeps`: Ucuncu parti bagimlilik tanimlari
- `scripts`: Yardimci komut dosyalari

## Gereksinimler
- Java 17 (WPILib ile uyumlu)
- Git
- WPILib gelistirme ortami (VS Code eklentileri dahil)

## Hizli Baslangic
1. Depoyu klonlayin:
   ```powershell
   git clone https://github.com/GOAT-8092/8092-2026.git
   cd 8092-2026
   ```
2. Derleme:
   ```powershell
   .\gradlew.bat build
   ```
3. Testleri calistirma:
   ```powershell
   .\gradlew.bat test
   ```

## Sik Kullanilan Komutlar
- Tam kontrol (derleme + test): `.\gradlew.bat build`
- Sadece test: `.\gradlew.bat test`
- Robot kodu derleme: `.\gradlew.bat compileJava`
- Bagimliliklari guncelleme/indirme: `.\gradlew.bat downloadDepsPreemptively`

## Dokumantasyon
- [ROBOT_SETUP.md](ROBOT_SETUP.md)
- [ROBOT_USER_GUIDE.md](ROBOT_USER_GUIDE.md)
- [HARDWARE_VALIDATION.md](HARDWARE_VALIDATION.md)
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
- [BUTTON_REFERENCE.md](BUTTON_REFERENCE.md)
- [APRILTAG_TEST_PLAN.md](APRILTAG_TEST_PLAN.md)
- [APRILTAG_ALIGN_TEST_GUIDE.md](APRILTAG_ALIGN_TEST_GUIDE.md)

## Notlar
- Projede `gradlew.bat` kullanimi tercih edilir (Windows).
- Yerel `main` daliniz `origin/main` gerisindeyse once guncelleyin:
  ```powershell
  git pull --ff-only
  ```
