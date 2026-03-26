# Hardware Validation Suite

This project now includes a structured hardware validation flow that records:

- `PASS/FAIL`
- UTC timestamp
- Robot build hash
- Check details

## Artifact format

Each line in `build/hardware-validation/results.txt`:

`STATUS | CHECK_NAME | TIMESTAMP | BUILD_HASH | DETAILS`

## Generate artifact

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run-hardware-validation.ps1 -BuildHash "<git-hash>"
```

## Recommended runbook

1. Disable robot in Driver Station.
2. Run individual motor checks at safe output.
3. Verify encoder direction/current behavior for each motor channel.
4. Verify roboRIO health (battery >= 10.0V, no brownout).
5. Verify Limelight pipeline/camMode/ledMode and FMAP tag count.
6. Save artifact with the robot build hash used on the roboRIO.

---

## Mekanizma Donanim Guncellemesi (2026-03-26)

Bu proje icin mekanizma donanimi asagidaki gibi gunceldir:

- Intake sistemi: 1 adet CIM motor
- Depodan aticiya yukari tasima (feeder/conveyor): 1 adet CIM motor
- Shooter sistemi: NEO + Spark Max
- Turret sistemi: NEO + Spark Max

Kod yansimalari:

- IntakeSubsystem: iki CIM hatti yonetir (alim + yukari tasiyici)
- ShootCommand: atis baslarken yukari tasiyici CIM motorunu da calistirir
- Sabitler:
  - MotorSabitleri.ALIM_CIM_PWM_KANALI
  - MotorSabitleri.DEPO_ATICI_YUKARI_TASIYICI_CIM_PWM_KANALI
  - ModulSabitleri.DEPO_ATICI_YUKARI_TASIYICI_HIZI

Notlar:

- Dokumanlarda onceki intake/shooter/turret aciklamalari bu bolumle birlikte degerlendirilmelidir.
- L1/R1 test atamalari ayri olarak surus testleri icin kullanilmaktadir.