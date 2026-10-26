# PICOtoPI 모듈을 로컬 드라이브(기본 E:\PICOtoPI)로 복사합니다.
# 기존 파일은 지우지 않습니다 (robocopy /E, /MIR 아님). 같은 이름 파일만 새 버전으로 덮어씁니다.
#   powershell -ExecutionPolicy Bypass -File tools\deploy_picotopi.ps1
#   powershell -ExecutionPolicy Bypass -File tools\deploy_picotopi.ps1 -Dest "E:\작업\PICOtoPI"
param([string]$Dest = "E:\PICOtoPI")
$ErrorActionPreference = "Stop"
$Repo = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path (Split-Path -Qualifier $Dest))) { throw "드라이브가 없습니다: $Dest" }

$layout = @(
  @{ src = "picotopi";        dst = "modules\picotopi" },
  @{ src = "v8";              dst = "legacy\v8" },
  @{ src = "docs\research";   dst = "docs\research" },
  @{ src = "docs\report";     dst = "docs\report" }
)
foreach ($m in $layout) {
  robocopy (Join-Path $Repo $m.src) (Join-Path $Dest $m.dst) /E /XD __pycache__ runs /XF *.pyc /NFL /NDL /NJH /NJS | Out-Null
  if ($LASTEXITCODE -ge 8) { throw "복사 실패: $($m.src)" }
}
New-Item -ItemType Directory -Force (Join-Path $Dest "tests"), (Join-Path $Dest "runs") | Out-Null
Copy-Item (Join-Path $Repo "tests\test_picotopi.py") (Join-Path $Dest "tests") -Force
Copy-Item (Join-Path $Repo "tests\test_v8_smoke.py") (Join-Path $Dest "tests") -Force
Copy-Item (Join-Path $Repo "requirements-v8.txt") (Join-Path $Dest "requirements.txt") -Force

# modules\ 와 legacy\ 를 import 경로에 두는 실행 스크립트
@"
@echo off
set PYTHONPATH=%~dp0modules;%~dp0legacy;%PYTHONPATH%
cd /d %~dp0
python %*
"@ | Set-Content -Encoding ASCII (Join-Path $Dest "run.bat")

Write-Host "완료: $Dest"
Write-Host "  $Dest\run.bat -m pytest tests -q"
Write-Host "  $Dest\run.bat -m picotopi.train --seed 0 --out runs\seed0"
