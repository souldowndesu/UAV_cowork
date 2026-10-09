param([string]$Executable, [string]$Settings)
$ErrorActionPreference = 'Stop'
try {
    $scenePath = (Resolve-Path -LiteralPath $Executable).Path
    $settingsPath = (Resolve-Path -LiteralPath $Settings).Path
    if (Get-NetTCPConnection -LocalPort 41451 -State Listen -ErrorAction SilentlyContinue) {
        Write-Host 'Simulator RPC is already available. Reuse it, or close it before changing settings.'
        exit 0
    }
    # Packaged Unreal bootstrappers can wait on a Windows security dialog.
    # Run the known packaged engine directly, without changing security settings.
    $sceneName = [IO.Path]::GetFileNameWithoutExtension($scenePath)
    $enginePath = Join-Path ([IO.Path]::GetDirectoryName($scenePath)) "$sceneName\Binaries\Win64\$sceneName.exe"
    $projectArgument = ''
    if (Test-Path -LiteralPath $enginePath -PathType Leaf) {
        $scenePath = (Resolve-Path -LiteralPath $enginePath).Path
        $projectArgument = "$sceneName "
    }
    $loading = Get-Process -Name $sceneName -ErrorAction SilentlyContinue | Where-Object { $_.WorkingSet64 -gt 5MB }
    if ($loading) {
        Write-Host 'A simulator engine is already starting. Wait for RPC or close that engine before retrying.'
        exit 1
    }
    $sceneStart = New-Object System.Diagnostics.ProcessStartInfo
    $sceneStart.FileName = $scenePath
    $sceneStart.WorkingDirectory = [IO.Path]::GetDirectoryName($scenePath)
    $sceneStart.Arguments = $projectArgument + '-settings="' + $settingsPath + '" -windowed -ResX=1280 -ResY=720'
    $sceneStart.UseShellExecute = $false
    $sceneStart.CreateNoWindow = $true
    $sceneStart.WindowStyle = [Diagnostics.ProcessWindowStyle]::Hidden
    $sceneProcess = [Diagnostics.Process]::Start($sceneStart)
    Write-Host "Simulator started (PID $($sceneProcess.Id)). Wait for the scene, then open replay.py or chase_test.py."
} catch {
    Write-Host "Cannot start simulator: $($_.Exception.Message)"
    exit 1
}
