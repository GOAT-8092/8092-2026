# Robot Kurulum Notlari

## 1) Surus Sistemi

- 4 adet NEO + Spark Max (CAN)
- Mecanum surus
- CAN ID haritasi:
  - ON_SOL_MOTOR_ID = 1
  - ON_SAG_MOTOR_ID = 2
  - ARKA_SAG_MOTOR_ID = 3
  - ARKA_SOL_MOTOR_ID = 4

## 2) Mekanizma Sistemi (Guncel)

### Intake
- 1 adet CIM
- PWM kanali: ALIM_CIM_PWM_KANALI = 8

### Depodan aticiya yukari tasima (feeder/conveyor)
- 1 adet CIM
- PWM kanali: DEPO_ATICI_YUKARI_TASIYICI_CIM_PWM_KANALI = 9

### Shooter
- NEO + Spark Max
- CAN ID:
  - SOL_ATICI_MOTOR_ID = 6
  - SAG_ATICI_MOTOR_ID = 7
  - UST_ATICI_MOTOR_ID = 8

### Turret
- NEO + Spark Max
- CAN ID:
  - TARET_MOTOR_ID = 9

## 3) Kontrol ve Test Notlari

- L1: CAN ID 3 motor %10 geri test
- R1: CAN ID 3 motor %10 ileri test
- R2: AprilTag takip
- PWM test motoru ayri bir test hatti olarak kullanilir (PWM kanal 1)

## 4) Konfigurasyon Bayragi

- SURUS_DISI_MOTORLARI_ETKIN
  - true: intake/shooter/turret/feeder hatlari aktif
  - false: bu alt sistemler fiziksel cihaza cikis vermez

## 5) Ag ve Ana Bilesenler

- roboRIO: 10.80.92.2
- Radio: 10.80.92.11
- Limelight: 10.80.92.200
- NavX: MXP SPI

## 6) Not

Bu dosya donanim icin tek referans kabul edilmelidir.
Kullanim/test akislari icin ROBOT_USER_GUIDE.md kullanin.
