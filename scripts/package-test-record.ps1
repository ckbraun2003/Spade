<#
.SYNOPSIS
    The record of one installer test, as text: scripts\package-test.ps1 fills a hashtable and these functions turn it into
    `<setup name>.test.md` and one ledger row. Dot-sourced by package-test.ps1 and by tests\package_test_record_test.ps1.

.DESCRIPTION
    The hashtable `$R` holds:
      StartedUtc [datetime]   SetupPath, SetupName, SetupBytes, Sha256   SetupCommit (the tag in the setup's name; "-dirty" is kept)
      ScriptCommit            the checkout package-test.ps1 ran from ("unknown" outside a git checkout)
      ExpectVersion           what `spade_sandbox --version` had to print       VersionPrinted (what it did print)
      Ticket                  the Vantage job ticket ("(none: run by hand)" otherwise)       ElapsedSeconds [double]
      Machine                 hashtable: Windows, Cpu, Memory, Gpus (string[])
      Checks                  array of objects with Name, Ok [bool], Detail
      SmokeRun [bool]         false when -SkipSmoke       Smoke: hashtable or $null (Exit, Journal, End, Result, ChecksCounted, Notes, KeptDir, Kept (file names))
      Kept [bool]             -Keep: the install was left in place, step 6 not run
      LogPath                 the full log's path         Exit [int] the script's exit code
#>

function ConvertTo-RecordCell([string] $Text) {
    # One table cell: no pipes, no line breaks.
    if ($null -eq $Text) { return '' }
    ($Text -replace '\|', '\|' -replace '[\r\n]+', ' ').Trim()
}

function ConvertTo-PublicRecord([string] $Text, [hashtable] $Folders = $null) {
    # The copy of a record that goes into the public repo: the user's profile, local-app-data and roaming-app-data folders become
    # %USERPROFILE%, %LOCALAPPDATA% and %APPDATA%, so a home folder (C:\Users\<name>) is never printed. The record beside the setup keeps the real paths.
    # $Folders maps those three names to folders; by default the running user's. Matches either slash, a doubled backslash (JSON text in a journal note),
    # any case, and only a whole folder name (C:\Users\Ada does not match C:\Users\Adam). The deepest folder is replaced first.
    if ($null -eq $Folders) { $Folders = @{ USERPROFILE = $env:USERPROFILE; LOCALAPPDATA = $env:LOCALAPPDATA; APPDATA = $env:APPDATA } }
    $pairs = @($Folders.GetEnumerator() | Where-Object { $_.Value } | Sort-Object { $_.Value.Length } -Descending)
    foreach ($pair in $pairs) {
        $parts = @($pair.Value.TrimEnd('\', '/') -split '[\\/]+' | Where-Object { $_ })
        $pattern = ($parts | ForEach-Object { [regex]::Escape($_) }) -join '(?:\\\\|\\|/)'
        $Text = [regex]::Replace($Text, "(?i)$pattern(?![\w~-])", ('%' + $pair.Key + '%').Replace('$', '$$'))
    }
    $Text
}

function Get-InstallTestVerdict($R) {
    $failed = @($R.Checks | Where-Object { -not $_.Ok })
    if ($failed.Count -gt 0) { return 'FAIL' }
    if ($R.Kept) { return 'KEPT' }
    if (-not $R.SmokeRun) { return 'PASS-NO-SMOKE' }
    'PASS'
}

function Format-Bytes([double] $N) {
    if ($N -ge 1MB) { return ('{0:N1} MB' -f ($N / 1MB)) }
    if ($N -ge 1KB) { return ('{0:N1} KB' -f ($N / 1KB)) }
    "$N bytes"
}

function Format-Elapsed([double] $S) {
    $t = [TimeSpan]::FromSeconds($S)
    if ($t.TotalMinutes -ge 1) { return ('{0} min {1} s ({2:N0} s)' -f [int][math]::Floor($t.TotalMinutes), $t.Seconds, $S) }
    '{0:N0} s' -f $S
}

function New-InstallTestRecord($R) {
    $verdict = Get-InstallTestVerdict $R
    $n = @($R.Checks).Count
    $bad = @($R.Checks | Where-Object { -not $_.Ok })
    $L = New-Object System.Collections.Generic.List[string]
    $L.Add("# Installer test: $($R.SetupName)")
    $L.Add('')
    $summary = "**Result: $verdict** ($($n - $bad.Count) of $n checks passed"
    if ($bad.Count -gt 0) { $summary += "; failed: $((@($bad | ForEach-Object { $_.Name })) -join '; ')" }
    $summary += ')'
    if ($verdict -eq 'PASS-NO-SMOKE') { $summary += ' The live smoke was not run (-SkipSmoke), so this installer is not smoked.' }
    if ($verdict -eq 'KEPT') { $summary += ' The install was kept (-Keep); the uninstall checks were not run.' }
    $L.Add($summary)
    $L.Add('')
    $L.Add('| Field | Value |')
    $L.Add('|---|---|')
    $L.Add("| Tested (UTC) | $($R.StartedUtc.ToUniversalTime().ToString('yyyy-MM-dd HH:mm:ss'))Z |")
    $L.Add("| Setup | ``$(ConvertTo-RecordCell $R.SetupName).exe`` |")
    $L.Add("| Setup path | ``$(ConvertTo-RecordCell $R.SetupPath)`` |")
    $L.Add("| Setup size | $(Format-Bytes $R.SetupBytes) ($($R.SetupBytes) bytes) |")
    $L.Add("| Setup sha256 | ``$($R.Sha256)`` |")
    $L.Add("| Setup built from commit | ``$($R.SetupCommit)`` (the tag in the setup's name; ``-dirty`` means uncommitted changes) |")
    $L.Add("| Test script from commit | ``$($R.ScriptCommit)`` |")
    $L.Add("| Version the installed editor printed | $(ConvertTo-RecordCell $R.VersionPrinted) (expected ``$($R.ExpectVersion)``) |")
    $L.Add("| Job ticket | $(ConvertTo-RecordCell $R.Ticket) |")
    $L.Add("| Total time | $(Format-Elapsed $R.ElapsedSeconds) |")
    $L.Add('')
    $L.Add('## Machine')
    $L.Add('')
    $L.Add("- Windows: $(ConvertTo-RecordCell $R.Machine.Windows)")
    $L.Add("- CPU: $(ConvertTo-RecordCell $R.Machine.Cpu)")
    $L.Add("- Memory: $(ConvertTo-RecordCell $R.Machine.Memory)")
    foreach ($g in @($R.Machine.Gpus)) { $L.Add("- GPU: $(ConvertTo-RecordCell $g)") }
    $L.Add('')
    $L.Add('## Checks')
    $L.Add('')
    $L.Add('| # | Result | Check | Detail |')
    $L.Add('|---|---|---|---|')
    $i = 0
    foreach ($c in @($R.Checks)) {
        $i++
        $res = if ($c.Ok) { 'PASS' } else { 'FAIL' }
        $L.Add("| $i | $res | $(ConvertTo-RecordCell $c.Name) | $(ConvertTo-RecordCell $c.Detail) |")
    }
    $L.Add('')
    $L.Add('## Live smoke')
    $L.Add('')
    if (-not $R.SmokeRun) {
        $L.Add('Not run: -SkipSmoke was given, or the script stopped before step 5.')
    } elseif ($null -eq $R.Smoke) {
        $L.Add('The smoke did not run to a journal.')
    } else {
        $s = $R.Smoke
        $L.Add("- Exit code: $($s.Exit)")
        $L.Add("- Journal ends: ``$(ConvertTo-RecordCell $s.End)``")
        $L.Add("- Journal result line: ``$(ConvertTo-RecordCell $s.Result)``")
        $L.Add("- Journal checks counted: $($s.ChecksCounted)")
        foreach ($note in @($s.Notes)) { if ($note) { $L.Add("- Journal note: $(ConvertTo-RecordCell $note)") } }
        if ($s.KeptDir) {
            $L.Add("- Kept in ``$(ConvertTo-RecordCell $s.KeptDir)``: $((@($s.Kept)) -join ', ')")
            $still = @($s.Kept | Where-Object { $_ -like '*.png' } | Select-Object -First 1)
            if ($still.Count -gt 0) { $L.Add("- A still: ``$(ConvertTo-RecordCell (Join-Path $s.KeptDir $still[0]))``") }
        } else {
            $L.Add('- Nothing kept (no journal).')
        }
    }
    $L.Add('')
    $L.Add('## Log')
    $L.Add('')
    $L.Add("The full log of this run: ``$(ConvertTo-RecordCell $R.LogPath)``. The script exited $($R.Exit).")
    $L.Add('')
    ($L -join "`n")
}

function New-InstallLedgerRow($R, [string] $RecordRelPath) {
    $verdict = Get-InstallTestVerdict $R
    $n = @($R.Checks).Count
    $ok = @($R.Checks | Where-Object { $_.Ok }).Count
    $smoke = if (-not $R.SmokeRun) { 'not run' } elseif ($null -eq $R.Smoke) { 'no journal' } else { ConvertTo-RecordCell $R.Smoke.End }
    $gpu = (@($R.Machine.Gpus) | Select-Object -First 1)
    $cells = @(
        ($R.StartedUtc.ToUniversalTime().ToString('yyyy-MM-dd HH:mm') + 'Z'),
        ('`' + $R.SetupName + '`'),
        ('`' + $R.Sha256.Substring(0, 12) + '`'),
        (Format-Bytes $R.SetupBytes),
        "**$verdict** $ok/$n",
        (ConvertTo-RecordCell $smoke),
        (ConvertTo-RecordCell "$($R.Machine.Windows); $gpu"),
        (ConvertTo-RecordCell $R.Ticket),
        ('[record](' + $RecordRelPath + ')')
    )
    '| ' + ($cells -join ' | ') + ' |'
}

$InstallLedgerHeader = @'
# Installer ledger

One row per installer tested by `scripts/package-test.ps1` (run with `-LedgerDir`). The row says what was tested and how it came out; the record it links to
holds every check, the machine, the smoke journal's summary and the job ticket. The record's `.test.md` is copied into `installer-records/` when the row is added.
The stills, the tour video and the full log stay beside the setup.exe in the build folder (not in git).

| Tested (UTC) | Setup | sha256 (12) | Size | Result | Live smoke | Windows; GPU, driver | Job ticket | Record |
|---|---|---|---|---|---|---|---|---|
'@
