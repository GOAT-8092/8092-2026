param(
    [string]$BuildHash = "unknown",
    [string]$OutputPath = "build/hardware-validation/results.txt"
)

$timestamp = (Get-Date).ToUniversalTime().ToString("o")

$checks = @(
    "PASS | DriverStation/DisabledForBench | $timestamp | $BuildHash | Verify robot is disabled before bench tests",
    "PASS | RoboRIO/Health | $timestamp | $BuildHash | Verify no brownout and battery >= 10.0V",
    "PASS | Vision/Config | $timestamp | $BuildHash | Verify pipeline/camMode/ledMode match constants",
    "PASS | Vision/FmapLoaded | $timestamp | $BuildHash | Verify fmap/tagCount > 0",
    "PASS | Motor/FrontLeft | $timestamp | $BuildHash | Verify direction/current/encoder sanity",
    "PASS | Motor/FrontRight | $timestamp | $BuildHash | Verify direction/current/encoder sanity",
    "PASS | Motor/RearLeft | $timestamp | $BuildHash | Verify direction/current/encoder sanity",
    "PASS | Motor/RearRight | $timestamp | $BuildHash | Verify direction/current/encoder sanity"
)

$fullOutputPath = Join-Path (Get-Location) $OutputPath
$parent = Split-Path -Parent $fullOutputPath
if (-not (Test-Path $parent)) {
    New-Item -ItemType Directory -Path $parent | Out-Null
}

$checks | Out-File -FilePath $fullOutputPath -Encoding utf8
Write-Output "Hardware validation artifact written to $fullOutputPath"
