param(
    [string]$Distro = $(if ($env:AIR_MOUSE_WSL_DISTRO) { $env:AIR_MOUSE_WSL_DISTRO } else { "Ubuntu-24.04" }),
    [switch]$Headless,
    [ValidateSet("fast", "full")]
    [string]$Profile = $(if ($env:AIR_MOUSE_PROFILE) { $env:AIR_MOUSE_PROFILE } else { "fast" })
)

$ErrorActionPreference = "Stop"

if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
    throw "wsl.exe was not found. Install/enable WSL2 before running AirMouse."
}

$scriptPath = (Resolve-Path (Join-Path $PSScriptRoot "run_airmouse_sim.sh")).Path
$wslScriptPath = wsl.exe -d $Distro -- wslpath -a -u ($scriptPath -replace "\\", "/")
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($wslScriptPath)) {
    throw "Could not convert the launcher path for WSL distribution '$Distro'."
}

$guiValue = if ($Headless) { "0" } elseif ($env:AIR_MOUSE_GUI) { $env:AIR_MOUSE_GUI } else { "1" }
wsl.exe -d $Distro -- env "AIR_MOUSE_GUI=$guiValue" "AIR_MOUSE_PROFILE=$Profile" bash $wslScriptPath.Trim()
if ($LASTEXITCODE -ne 0) {
    throw "The WSL AirMouse launcher failed with exit code $LASTEXITCODE."
}

$qgroundControl = if ($env:QGROUNDCONTROL_EXE) {
    $env:QGROUNDCONTROL_EXE
} else {
    "C:\Program Files\QGroundControl\bin\QGroundControl.exe"
}
if (Test-Path -LiteralPath $qgroundControl) {
    if (-not (Get-Process -Name QGroundControl -ErrorAction SilentlyContinue)) {
        Start-Process -FilePath $qgroundControl
    }
    Write-Host "QGroundControl is installed and running."
} else {
    Write-Warning "QGroundControl was not found at $qgroundControl"
}

Write-Host "Verify with: wsl.exe -d $Distro -- bash -lc 'source /opt/ros/jazzy/setup.bash; source `$HOME/ros2_px4_ws/install/setup.bash; ros2 topic list'"
Write-Host "Performance profile: $Profile (use -Profile full for full-fidelity LiDAR and shadows)."
