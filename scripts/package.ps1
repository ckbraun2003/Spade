<#
.SYNOPSIS
    ONE command: builds Spade Editor (release), stages its files, runs CPack's Inno Setup generator, and prints the setup.exe's
    path, version, git SHA, size and sha256.

.DESCRIPTION
    The installer is for testers: per-user, no administrator prompt, Start-menu and desktop shortcuts, an uninstaller in Add/Remove
    Programs. Its file list is the `editor` install component (packaging/Packaging.cmake): spade_sandbox.exe (and SpadeEditor.exe, the
    launcher, once it exists), assets/, the licence and notices, the release docs, the user guide and the MSVC runtime, app-local.

    Steps: refuse a dirty tree; check the inputs (packaging/common docs, the user guide's site and PDF, Inno Setup's ISCC.exe);
    configure and build the editor in the release tree; install the editor component into a staging folder and check its file list
    (what must be there, what must not); cpack -G INNOSETUP; print the result.

    THE REAL setup.exe IS BUILT FROM A GATED master SHA, so a tree with modified tracked files is refused. -AllowDirty is for
    the install test on a working tree: the file is named ...-dirty-setup.exe and is not a release.

    Run it in the foreground. Under the job queue: python C:/Users/Cameron/Desktop/Vantage/vantage.py job run --class build ... -- powershell -File scripts\package.ps1

.PARAMETER GuideDir
    The user guide's site folder (index.html inside), from docs/user-guide/build.ps1 -Out <dir>. Default: <repo>\build-userguide\site.

.PARAMETER GuidePdf
    The guide's PDF. Default: the newest Spade-Editor-User-Guide-*.pdf in the folder above GuideDir.

.PARAMETER OutDir
    Where the staging folder and the setup.exe go. Default: <repo>\build-package (git-ignored; builds are never committed).

.PARAMETER Iscc
    ISCC.exe, Inno Setup's compiler. Default: $env:SPADE_ISCC, then Inno Setup's registered install location, then PATH.

.PARAMETER BuildDir
    The release build tree. Default: <repo>\build-ninja\release. It must be configured already (scripts\build.ps1 configures it).

.PARAMETER AllowDirty
    Build from a tree with modified tracked files (the install test). The version string and the file name say -dirty.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File scripts\package.ps1
#>
[CmdletBinding()]
param(
    [string] $GuideDir,
    [string] $GuidePdf,
    [string] $OutDir,
    [string] $Iscc,
    [string] $BuildDir,
    [switch] $AllowDirty
)
$ErrorActionPreference = 'Stop'

$repo = Split-Path -Parent $PSScriptRoot
if (-not $GuideDir) { $GuideDir = Join-Path $repo 'build-userguide\site' }
if (-not $OutDir) { $OutDir = Join-Path $repo 'build-package' }
if (-not $BuildDir) { $BuildDir = Join-Path $repo 'build-ninja\release' }

function Fail([string] $Message) {
    Write-Host "package: REFUSED: $Message"
    exit 1
}

# ---- the tree -------------------------------------------------------------------------------------------------------------------------
$sha = (& git -C $repo rev-parse --short=7 HEAD).Trim()
if (-not $sha) { Fail 'git gave no commit SHA' }
$branch = (& git -C $repo rev-parse --abbrev-ref HEAD).Trim()
$changed = @(& git -C $repo status --porcelain --untracked-files=no)
$dirty = $changed.Count -gt 0
if ($dirty -and -not $AllowDirty) {
    Fail ("the tree has $($changed.Count) modified tracked file(s); a setup.exe is built from a gated master SHA. Commit, or pass -AllowDirty " +
          'for an install test (the file is named -dirty and is not a release).')
}
if ($branch -ne 'master') {
    Write-Host "package: note: building from branch '$branch', not master; a release setup.exe is built from the gated master SHA"
}

$versionFile = Join-Path $repo 'packaging\common\VERSION'
if (-not (Test-Path -LiteralPath $versionFile)) { Fail "no $versionFile" }
$version = ((Get-Content -LiteralPath $versionFile -Raw).Trim() -split '\s+')[0]
if ($version -notmatch '^\d+\.\d+\.\d+$') { Fail "$versionFile must begin with MAJOR.MINOR.PATCH (it has '$version')" }
$tag = if ($dirty) { "$sha-dirty" } else { $sha }
$fileName = "Spade-Editor-$version-$tag-setup"

# ---- the inputs -----------------------------------------------------------------------------------------------------------------------
foreach ($doc in 'INSTALL.md', 'RELEASE_NOTES.md', 'SYSTEM_REQUIREMENTS.md') {
    if (-not (Test-Path -LiteralPath (Join-Path $repo "packaging\common\$doc"))) { Fail "packaging\common\$doc is missing (Test/Docs writes it)" }
}
foreach ($licence in 'LICENSE', 'THIRD_PARTY_NOTICES.md') {
    if (-not (Test-Path -LiteralPath (Join-Path $repo $licence))) { Fail "$licence is missing at the repository root" }
}
if (-not (Test-Path -LiteralPath (Join-Path $GuideDir 'index.html'))) {
    Fail "no user guide at $GuideDir\index.html. Build it: powershell -File docs\user-guide\build.ps1 -Out <dir> (see -GuideDir)."
}
if (-not $GuidePdf) {
    $found = Get-ChildItem -LiteralPath (Split-Path -Parent $GuideDir) -Filter 'Spade-Editor-User-Guide-*.pdf' -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if ($found) { $GuidePdf = $found.FullName }
}
if (-not $GuidePdf -or -not (Test-Path -LiteralPath $GuidePdf)) { Fail 'no user guide PDF (Spade-Editor-User-Guide-*.pdf beside the site folder; see -GuidePdf)' }

if (-not $Iscc) { $Iscc = $env:SPADE_ISCC }
if (-not $Iscc) {
    foreach ($root in 'HKCU:', 'HKLM:', 'HKLM:\SOFTWARE\WOW6432Node') {
        $keyRoot = if ($root -eq 'HKLM:\SOFTWARE\WOW6432Node') { "$root\Microsoft\Windows\CurrentVersion\Uninstall" } else { "$root\Software\Microsoft\Windows\CurrentVersion\Uninstall" }
        $hit = Get-ChildItem $keyRoot -ErrorAction SilentlyContinue | ForEach-Object { Get-ItemProperty $_.PSPath } |
            Where-Object { $_.DisplayName -like 'Inno Setup*' -and $_.InstallLocation } | Select-Object -First 1
        if ($hit) { $Iscc = Join-Path $hit.InstallLocation 'ISCC.exe'; break }
    }
}
if (-not $Iscc) { $cmd = Get-Command ISCC.exe -ErrorAction SilentlyContinue; if ($cmd) { $Iscc = $cmd.Source } }
if (-not $Iscc -or -not (Test-Path -LiteralPath $Iscc)) {
    Fail 'Inno Setup (ISCC.exe) not found. winget install JRSoftware.InnoSetup --scope user, or pass -Iscc / set SPADE_ISCC.'
}
$isccVersion = (Get-Item -LiteralPath $Iscc).VersionInfo.ProductVersion

Write-Host "package: Spade Editor $version+$tag from branch $branch"
Write-Host "package: guide  $GuideDir (+ $GuidePdf)"
Write-Host "package: inno   $Iscc ($isccVersion)"

# ---- configure (the guide's paths), build ---------------------------------------------------------------------------------------------
if (-not (Test-Path -LiteralPath (Join-Path $BuildDir 'CMakeCache.txt'))) {
    Fail "$BuildDir is not configured; run scripts\build.ps1 -Preset release once"
}
$cacheBuild = $BuildDir
& (Join-Path $PSScriptRoot 'build.ps1') -BuildDir $cacheBuild -Target spade_editor_files -ParallelLevel 8
if ($LASTEXITCODE -ne 0) { Fail "the build failed (exit $LASTEXITCODE)" }

# The guide is read when the component is installed (packaging/Packaging.cmake), from these two variables; cpack's own install inherits them.
$env:SPADE_USER_GUIDE_DIR = (Resolve-Path -LiteralPath $GuideDir).Path
$env:SPADE_USER_GUIDE_PDF = (Resolve-Path -LiteralPath $GuidePdf).Path

# ---- stage and check the file list ----------------------------------------------------------------------------------------------------
$stage = Join-Path $OutDir 'stage'
if (Test-Path -LiteralPath $stage) { Remove-Item -LiteralPath $stage -Recurse -Force }
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
& cmake --install $BuildDir --config Release --component editor --prefix $stage | Out-Null
if ($LASTEXITCODE -ne 0) { Fail "cmake --install --component editor failed (exit $LASTEXITCODE)" }

$must = @('spade_sandbox.exe', 'LICENSE', 'THIRD_PARTY_NOTICES.md', 'VERSION', 'INSTALL.md', 'RELEASE_NOTES.md', 'SYSTEM_REQUIREMENTS.md',
          'assets\library', 'assets\scenes', 'assets\worlds', 'assets\fonts\Inter-Regular.ttf', 'docs\user-guide\index.html')
$missing = @($must | Where-Object { -not (Test-Path -LiteralPath (Join-Path $stage $_)) })
if (-not (Get-ChildItem -LiteralPath $stage -Filter 'vcruntime140*.dll' -ErrorAction SilentlyContinue)) { $missing += 'vcruntime140.dll (the MSVC runtime, app-local)' }
if (-not (Get-ChildItem -LiteralPath (Join-Path $stage 'docs\user-guide') -Filter '*.pdf' -ErrorAction SilentlyContinue)) { $missing += 'docs\user-guide\*.pdf' }
if ($missing) { Fail ("the editor component lacks: " + ($missing -join ', ')) }
$files = @(Get-ChildItem -LiteralPath $stage -Recurse -File)
$stray = @($files | Where-Object { $_.Extension -in '.lib', '.hpp', '.h', '.pdb', '.ilk', '.obj' -or $_.Name -in 'flow_movie.exe', 'spade_tests.exe', 'spade_bench.exe' })
if ($stray) { Fail ("the editor component carries files it must not: " + (($stray | ForEach-Object { $_.Name }) -join ', ')) }
Write-Host "package: staged $($files.Count) files, $([math]::Round(($files | Measure-Object Length -Sum).Sum / 1MB, 1)) MB, in $stage"

# ---- cpack ----------------------------------------------------------------------------------------------------------------------------
$config = Join-Path $BuildDir 'CPackConfig.cmake'
& cpack --config $config -G INNOSETUP -B $OutDir `
    "-DCPACK_PACKAGE_FILE_NAME=$fileName" "-DCPACK_INNOSETUP_EXECUTABLE=$Iscc" | Out-Host
if ($LASTEXITCODE -ne 0) { Fail "cpack failed (exit $LASTEXITCODE)" }

$setup = Join-Path $OutDir "$fileName.exe"
if (-not (Test-Path -LiteralPath $setup)) { Fail "cpack finished but $setup does not exist" }
$item = Get-Item -LiteralPath $setup
$hash = (Get-FileHash -LiteralPath $setup -Algorithm SHA256).Hash.ToLower()
Write-Host ''
Write-Host "package: setup   $($item.FullName)"
Write-Host "package: version $version+$tag"
Write-Host "package: sha     $sha$(if ($dirty) { ' (dirty tree: not a release)' })"
Write-Host "package: size    $($item.Length) bytes ($([math]::Round($item.Length / 1MB, 1)) MB)"
Write-Host "package: sha256  $hash"
exit 0
