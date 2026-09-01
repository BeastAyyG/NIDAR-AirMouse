$ErrorActionPreference = 'Stop'
$tokens = $null
$err = $null
$path = (Resolve-Path 'Run-AirMouseSim.ps1').Path
[System.Management.Automation.Language.Parser]::ParseFile($path, [ref]$tokens, [ref]$err)
if ($err.Count -gt 0) {
  $err | ForEach-Object { Write-Host $_.Message }
  exit 1
} else {
  Write-Host 'POWERSHELL-OK'
  Write-Host "Validated: $path"
}
