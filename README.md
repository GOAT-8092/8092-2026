# GOAT #8092 Robot Kodu (2026)

Bu repo, FRC Team **G.O.A.T. #8092** (Tekirdag) icin gelistirilen robot yazilimini icerir.
Takim hakkinda: [8092.tr](https://www.8092.tr/)

## Takim Kimligi
- Takim adi: Greatest of All Times (G.O.A.T.) #8092
- Kurulus: 2019
- Konum: Tekirdag / Cerkezkoy
- Not: 8092.tr uzerindeki bilgilere gore Tekirdag'in ilk FRC takimi

## Proje Ozeti
- Dil ve altyapi: Java, WPILib Command-Based, GradleRIO
- Surus: Mecanum
- Mekanizma: alim, yukari tasiyici, atici, taret
- Vision: Limelight + AprilTag

## Klasor Yapisi
- `src/main/java`: Robot kodu (alt sistemler, komutlar, sabitler)
- `src/test/java`: Birim/dogrulama testleri
- `src/main/deploy`: Deploy edilen dashboard ve saha varliklari
- `vendordeps`: Ucuncu parti kutuphane tanimlari
- `scripts`: Yardimci scriptler

## Gereksinimler
- Java 17
- WPILib gelistirme ortami
- Git

## Hizli Baslangic
1. Depoyu klonla:
   ```powershell
   git clone https://github.com/GOAT-8092/8092-2026.git
   cd 8092-2026
   ```
2. Derle:
   ```powershell
   .\gradlew.bat build
   ```
3. Test et:
   ```powershell
   .\gradlew.bat test
   ```

## Sik Kullanilan Komutlar
- `.\gradlew.bat build`: Derleme + test
- `.\gradlew.bat test`: Sadece testler
- `.\gradlew.bat compileJava`: Sadece Java derleme
- `.\gradlew.bat deploy`: RoboRIO deploy
- `.\gradlew.bat downloadDepsPreemptively`: Vendordep indirme

## Dokumanlar
Bu dalda bulunan guncel Markdown dokumanlari:
- [ROBOT.md](ROBOT.md): Donanim haritasi, kontrol semasi, test akisi
- [TODO.md](TODO.md): Teknik takip ve capraz kontrol notlari
- [AGENTS.md](AGENTS.md): Gelistirici asistan baglami
- [CLAUDE.md](CLAUDE.md): Alternatif gelistirici asistan baglami
- [WPILib-License.md](WPILib-License.md): Lisans metni

## Calisma Notlari
- Windows ortaminda `gradlew.bat` kullan.
- Local `main` geri kaldiginda:
  ```powershell
  git pull --ff-only
  ```
- Surus disi motorlarin fiziksel aktivasyonu kod icindeki sabit ile kontrol edilir:
  - `Sabitler.MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN`
