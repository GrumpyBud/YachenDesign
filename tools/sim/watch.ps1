<#
.SYNOPSIS
  Runs the robot simulator and restarts it a few seconds after you save a change.

.DESCRIPTION
  Builds with Gradle (prepareSim), then launches the simulator JVM directly so a restart skips
  Gradle's own startup. While it runs, saving any file under src/main, build.gradle or vendordeps
  rebuilds and restarts the simulator. If the build fails, the old simulator keeps running.
  AdvantageScope and Elastic reconnect on their own after a restart.

  Keys while running: r = rebuild and restart now, q = quit.

.PARAMETER Gui
  Load the WPILib simulation GUI (Driver Station controls, joysticks, NetworkTables view).

.PARAMETER DriverStation
  Let the real Driver Station connect to the simulator.

.PARAMETER NoWatch
  Build and run once without restarting on changes.

.EXAMPLE
  .\tools\sim\watch.ps1 -Gui
#>
param(
  [switch]$Gui,
  [switch]$DriverStation,
  [switch]$NoWatch
)

$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$simDir = Join-Path $root 'build\sim'
$jniDir = Join-Path $root 'build\jni\release'
$gradle = Join-Path $root 'gradlew.bat'
$watchPaths = @(
  (Join-Path $root 'src\main'),
  (Join-Path $root 'vendordeps'),
  (Join-Path $root 'build.gradle')
)

function Write-Status([string]$text, [string]$color = 'Cyan') {
  Write-Host ("[sim {0}] {1}" -f (Get-Date -Format 'HH:mm:ss'), $text) -ForegroundColor $color
}

function Test-PortInUse([int]$port) {
  try {
    $listener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Any, $port)
    $listener.Start()
    $listener.Stop()
    return $false
  } catch {
    return $true
  }
}

function Set-JavaHome {
  # The WPILib VS Code terminal sets JAVA_HOME; a plain terminal may not.
  if ($env:JAVA_HOME -and (Test-Path (Join-Path $env:JAVA_HOME 'bin\java.exe'))) { return }
  if (Get-Command java -ErrorAction SilentlyContinue) { return }
  $jdk = Get-ChildItem -Path (Join-Path $env:PUBLIC 'wpilib') -Directory -ErrorAction SilentlyContinue |
    Sort-Object Name -Descending |
    ForEach-Object { Join-Path $_.FullName 'jdk' } |
    Where-Object { Test-Path (Join-Path $_ 'bin\java.exe') } |
    Select-Object -First 1
  if (-not $jdk) {
    throw 'No JDK found. Run this from the WPILib VS Code terminal, or set JAVA_HOME to the WPILib JDK.'
  }
  $env:JAVA_HOME = $jdk
  Write-Status "Using JDK $jdk" 'DarkGray'
}

function Invoke-SimBuild {
  Write-Status 'Building...'
  $start = Get-Date
  # Out-Host keeps Gradle's output on screen instead of in this function's return value.
  & $gradle -p $root prepareSim -q --console=plain | Out-Host
  $ok = $LASTEXITCODE -eq 0
  if ($ok) {
    Write-Status ('Built in {0:N1} s' -f ((Get-Date) - $start).TotalSeconds)
  }
  return $ok
}

function Find-Extension([string]$name) {
  $file = Get-ChildItem -Path $jniDir -File |
    Where-Object { $_.BaseName -eq $name -or $_.BaseName -eq "lib$name" } |
    Where-Object { $_.Extension -in '.dll', '.so', '.dylib' } |
    Select-Object -First 1
  if (-not $file) { throw "Simulation extension $name not found in $jniDir" }
  return $file.FullName
}

function Start-Sim {
  $javaHome = (Get-Content (Join-Path $simDir 'java-home.txt') -Raw).Trim()
  $java = Join-Path $javaHome 'bin\java.exe'
  $main = (Get-Content (Join-Path $simDir 'main-class.txt') -Raw).Trim()

  # Inherited by the simulator process.
  $env:CLASSPATH = (Get-Content (Join-Path $simDir 'classpath.txt') -Raw).Trim()
  $env:PATH = "$jniDir;$script:basePath"
  $extensions = @()
  if ($Gui) { $extensions += Find-Extension 'halsim_gui' }
  if ($DriverStation) { $extensions += Find-Extension 'halsim_ds_socket' }
  $env:HALSIM_EXTENSIONS = $extensions -join ';'

  $javaArgs = @(
    "-Djava.library.path=`"$jniDir`"",
    '--add-opens', 'java.base/jdk.internal.vm=ALL-UNNAMED',
    '--add-opens', 'java.base/java.lang=ALL-UNNAMED',
    '--enable-native-access=ALL-UNNAMED',
    '-Dfile.encoding=UTF-8',
    $main
  )
  $process = Start-Process -FilePath $java -ArgumentList $javaArgs -WorkingDirectory $root `
    -NoNewWindow -PassThru
  # Reading the handle now lets PowerShell report the exit code later.
  $null = $process.Handle
  Write-Status "Simulator started (pid $($process.Id))" 'Green'
  return $process
}

function Stop-Sim($process) {
  if ($process -and -not $process.HasExited) {
    Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
    $process.WaitForExit(5000) | Out-Null
  }
}

function Get-SourceStamp {
  $latest = [DateTime]::MinValue
  $count = 0
  foreach ($path in $watchPaths) {
    if (-not (Test-Path $path)) { continue }
    $items = if (Test-Path $path -PathType Leaf) { Get-Item $path } else {
      Get-ChildItem -Path $path -Recurse -File -ErrorAction SilentlyContinue
    }
    foreach ($item in $items) {
      $count++
      if ($item.LastWriteTimeUtc -gt $latest) { $latest = $item.LastWriteTimeUtc }
    }
  }
  return "$($latest.Ticks):$count"
}

function Read-Key {
  try {
    if ([Console]::KeyAvailable) { return [Console]::ReadKey($true).KeyChar }
  } catch {
    # No interactive console (output redirected); only file changes restart the simulator.
  }
  return $null
}

if (Test-PortInUse 5810) {
  Write-Status 'Port 5810 is in use: another simulator or robot program is already running. Stop it first.' 'Red'
  exit 1
}

Set-JavaHome
$script:basePath = $env:PATH
$sim = $null
try {
  if (-not (Invoke-SimBuild)) {
    Write-Status 'Build failed. Fix the error and save to try again.' 'Red'
    if ($NoWatch) { exit 1 }
  } else {
    $sim = Start-Sim
  }
  if ($NoWatch) {
    $sim.WaitForExit()
    exit $sim.ExitCode
  }

  Write-Status 'Watching src/main, build.gradle and vendordeps. Keys: r = restart, q = quit.' 'DarkGray'
  $stamp = Get-SourceStamp
  $reportedExit = $false
  while ($true) {
    Start-Sleep -Milliseconds 400
    $key = Read-Key
    if ($key -eq 'q') { break }

    $newStamp = Get-SourceStamp
    $changed = $newStamp -ne $stamp
    if ($changed) {
      # Let editors finish saving every file before building.
      do {
        $stamp = $newStamp
        Start-Sleep -Milliseconds 300
        $newStamp = Get-SourceStamp
      } while ($newStamp -ne $stamp)
    }

    if ($changed -or $key -eq 'r') {
      Write-Status ($(if ($changed) { 'Change detected' } else { 'Restart requested' }))
      if (Invoke-SimBuild) {
        Stop-Sim $sim
        $sim = Start-Sim
        $reportedExit = $false
      } else {
        Write-Status 'Build failed; the simulator keeps running the previous build.' 'Red'
      }
      $stamp = Get-SourceStamp
    }

    if ($sim -and $sim.HasExited -and -not $reportedExit) {
      Write-Status "Simulator exited with code $($sim.ExitCode). Save a change or press r to start it again." 'Yellow'
      $reportedExit = $true
    }
  }
} finally {
  Stop-Sim $sim
  Write-Status 'Stopped.' 'DarkGray'
}
