# 🎮 Robot Hızlı Referans Kartı

## PS4 Driver Controller

```
                    ┌─────────────────┐
                    │                 │
         L Stick    │    ○ ○  ● ○ ○   │    R Stick
      İleri/Geri    │                 │    Döndürme
        + Strafe    │    ○ ○  ● ○ ○   │
                    │                 │
                    └─────────────────┘

         1  2  3  4  5
         ⬜ ⬜ ⬜ ⬜ ⬜
```

## Buton Fonksiyonları

| Buton | DISABLED Modda | ENABLED Modda |
|-------|----------------|---------------|
| 1 | Tüm motorlar test (%30) | - |
| 2 | Arka Sol motor test | - |
| 3 | Arka Sağ motor test | - |
| 4 | Ön Sağ motor test | - |
| 5 | Vision pozisyon reset | Vision pozisyon reset |

## Sürüş

**Sol Stick** = İleri/Geri + Strafe (Sağ/Sol)
**Sağ Stick** = Döndürme

**Alan Odaklı**: Stick yönü = Sahadaki yön (robot başlığı önemli değil)

## Vision Reset (Button 5)

```
1. Robotu bir AprilTag'in önüne koyun
2. Limelight tag'i görsün (Vision/HasTarget = true)
3. Button 5'e basın
4. DriverStation mesajı: "Pose reset from AprilTag X..."
```

## SmartDashboard - Önemli Göstergeler

| Gösterge | Ne Demek |
|----------|----------|
| `Vision/HasTarget` | AprilTag görünüyor |
| `Vision/PoseValid` | Pozisyon doğru |
| `Vision/RobotX` | Sahada X (m) |
| `Vision/RobotY` | Sahada Y (m) |
| `Robot Heading` | Robot açısı (°) |
| `Drive/VisionUpdate` | Vision çalışıyor |

## Acil Durum

- **E-Stop** = Kırmızı buton
- **Space** = Robot disable (klavyede)
- **Enter** = Robot enable (klavyede)

## Önemli Değerler

- **Maksimum Hız**: ~3.0 m/s
- **Deadband**: 0.08 (stick hassasiyeti)
- **Vision Update**: 20Hz (her 50ms)
- **AprilTag Sayısı**: 32 (16 her alliance)

## Sorun?

| Sorun | Çözüm |
|-------|-------|
| Robot gitmiyor | E-Stop kontrol edin |
| Vision yok | Tag'e doğru çevirin |
| Yanlış pozisyon | Button 5 reset |
| Garip sürüş | NavX kontrol edin |

---

**Maç Öncesi Kontrol**: Vision/FmapTagCount = 32 ✅
