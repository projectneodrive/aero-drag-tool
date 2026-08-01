# Build the solver images on Windows. See build.sh for the rationale; this is
# the same thing for PowerShell, since the tool's primary host is Windows.
#
#   .\docker\build.ps1            # both
#   .\docker\build.ps1 openfoam   # just one
[CmdletBinding()]
param(
    [ValidateSet('all', 'openfoam', 'su2')]
    [string]$Target = 'all'
)

$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path

$openfoamImage = if ($env:AERO_OPENFOAM_IMAGE) { $env:AERO_OPENFOAM_IMAGE } else { 'aero-drag-tool/openfoam:13' }
$su2Image = if ($env:AERO_SU2_IMAGE) { $env:AERO_SU2_IMAGE } else { 'aero-drag-tool/su2:8.4.0' }

if ($Target -eq 'all' -or $Target -eq 'openfoam') {
    Write-Host "==> building $openfoamImage"
    docker build -f (Join-Path $here 'Dockerfile.openfoam') -t $openfoamImage $here
    if ($LASTEXITCODE -ne 0) { throw "OpenFOAM image build failed" }
}

if ($Target -eq 'all' -or $Target -eq 'su2') {
    Write-Host "==> building $su2Image (compiles SU2; expect this one to be slow)"
    docker build -f (Join-Path $here 'Dockerfile.su2') -t $su2Image $here
    if ($LASTEXITCODE -ne 0) { throw "SU2 image build failed" }
}

Write-Host ''
Write-Host 'Done. Check the tool picks them up with:'
Write-Host '    python src/runner.py info'
