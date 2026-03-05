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
