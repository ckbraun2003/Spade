<#
.SYNOPSIS
  Builds the Spade Editor user guide: HTML (opens from the install folder) and a PDF.

.DESCRIPTION
  1. Makes a build venv under <Out>\venv and installs the pinned tools into it (requirements-docs.txt), only when
     the venv does not exist. That is a download from PyPI: with -NoFetch the script stops instead. It never touches
     any other Python.
  2. Stages the pages: docs/user-guide/*.md, the shared release text from packaging/common, and the screenshots.
     A picture the pages name and -Shots does not have becomes a labelled "Picture to come" placeholder
     (<Out>\figures-to-come.txt lists them).
  3. Builds the HTML with MkDocs Material and its offline plugin into <Out>\site (file:// safe, search works offline).
  4. Prints the PDF with Edge headless: <Out>\Spade-Editor-User-Guide-<version>.pdf.
  5. Writes <Out>\guide-build.txt (tool versions, picture and to-confirm counts).

  Package.ps1's call:  powershell -File docs\user-guide\build.ps1 -Out <dir> -Shots <the live smoke run's folder>
  What it takes from the output: <Out>\site\ (the HTML guide) and <Out>\Spade-Editor-User-Guide-<version>.pdf.

.PARAMETER Out     Build folder (default: <repo>\build-userguide, ignored by git).
.PARAMETER Shots   A live-smoke run folder (Desktop\spade-smoke\<run>) whose *.png are the guide's pictures.
.PARAMETER NoFetch Stop rather than download the tools when the venv is missing.
.PARAMETER Python  The Python that makes the venv (default: python on the path).
#>
param(
    [string]$Out,
    [string]$Shots,
    [switch]$NoFetch,
    [string]$Python = "python"
)
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$repo = (Resolve-Path (Join-Path $here "..\..")).Path
if (-not $Out) { $Out = Join-Path $repo "build-userguide" }
New-Item -ItemType Directory -Force -Path $Out | Out-Null
$Out = (Resolve-Path $Out).Path

$venvPython = Join-Path $Out "venv\Scripts\python.exe"
if (-not (Test-Path $venvPython)) {
    if ($NoFetch) { throw "build venv missing at $Out\venv and -NoFetch was given" }
    Write-Host "Making the build venv and installing the pinned tools into it (a download from PyPI)."
    & $Python -I -m venv (Join-Path $Out "venv")
    if ($LASTEXITCODE -ne 0) { throw "could not make the venv" }
    & $venvPython -I -m pip install --disable-pip-version-check -r (Join-Path $here "requirements-docs.txt")
    if ($LASTEXITCODE -ne 0) { throw "pip install failed" }
}

$stageArgs = @("stage", "--repo", $repo, "--out", $Out)
if ($Shots) { $stageArgs += @("--shots", (Resolve-Path $Shots).Path) }
& $venvPython -I (Join-Path $here "guide_tools.py") @stageArgs
if ($LASTEXITCODE -ne 0) { throw "staging failed" }

Push-Location $Out
try {
    $site = Join-Path $Out "site"
    if (Test-Path $site) { Remove-Item -Recurse -Force $site }
    & $venvPython -I -m mkdocs build --strict --quiet
    if ($LASTEXITCODE -ne 0) { throw "mkdocs build failed" }
} finally { Pop-Location }
& $venvPython -I (Join-Path $here "guide_tools.py") print --out $Out
if ($LASTEXITCODE -ne 0) { throw "print page failed" }

$version = "0.1.0"
$versionFile = Join-Path $repo "packaging\common\VERSION"
if (Test-Path $versionFile) { $version = ((Get-Content $versionFile -TotalCount 1).Trim() -split "\s+")[0] }
$pdf = Join-Path $Out "Spade-Editor-User-Guide-$version.pdf"
if (Test-Path $pdf) { Remove-Item -Force $pdf }

$edge = @("${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe", "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe") |
    Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $edge) { throw "Microsoft Edge not found: the PDF needs it" }
$profile = Join-Path $Out "edge-profile"  # its own profile, so a running Edge does not swallow the request
$printUrl = ([System.Uri](Join-Path $site "print.html")).AbsoluteUri
Start-Process -FilePath $edge -Wait -NoNewWindow -ArgumentList @(
    "--headless", "--disable-gpu", "--no-pdf-header-footer", "--no-first-run", "--disable-sync", "--disable-background-networking",
    "--user-data-dir=`"$profile`"",
    "--print-to-pdf=`"$pdf`"", $printUrl)
if (-not (Test-Path $pdf) -or (Get-Item $pdf).Length -lt 1000) { throw "Edge did not write the PDF" }

$freeze = (& $venvPython -I -m pip freeze --disable-pip-version-check) -join "`n"
$pyVersion = (& $venvPython -I --version)
$edgeVersion = (Get-Item $edge).VersionInfo.ProductVersion
$figures = @(Get-Content (Join-Path $Out "figures-to-come.txt") -ErrorAction SilentlyContinue | Where-Object { $_ }).Count
$confirm = @(Get-Content (Join-Path $Out "to-confirm.txt") -ErrorAction SilentlyContinue | Where-Object { $_ }).Count
@(
    "Spade Editor user guide build, $(Get-Date -Format 'yyyy-MM-dd HH:mm')",
    "python: $pyVersion (venv)", "edge: $edgeVersion", "pictures to come: $figures", "lines to confirm: $confirm", "",
    "pip freeze:", $freeze
) | Set-Content -Encoding utf8 (Join-Path $Out "guide-build.txt")

Write-Host "Guide built: $site and $pdf ($figures pictures to come, $confirm lines to confirm)"
