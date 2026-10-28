# 학습(슬레이브) 노트북 1회 준비: 가상환경 + PyTorch(CUDA) + MuJoCo + 테스트.
# 저장소 루트 또는 E:\PICOtoPI 에서 실행:
#   powershell -ExecutionPolicy Bypass -File tools\setup_trainer.ps1
#   powershell -ExecutionPolicy Bypass -File tools\setup_trainer.ps1 -Cpu      # GPU 없는 노트북
param([switch]$Cpu, [string]$Venv = ".venv_picotopi")
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root

if (-not (Test-Path "$Venv\Scripts\python.exe")) { py -3.10 -m venv $Venv }
$Py = Join-Path $Root "$Venv\Scripts\python.exe"
& $Py -m pip install --upgrade pip
if ($Cpu) { & $Py -m pip install torch --index-url https://download.pytorch.org/whl/cpu }
else      { & $Py -m pip install torch --index-url https://download.pytorch.org/whl/cu121 }
& $Py -m pip install "mujoco>=3.1" "numpy<2" pytest

& $Py -c "import torch, mujoco; print('torch', torch.__version__, 'cuda', torch.cuda.is_available(), 'mujoco', mujoco.__version__)"
& $Py -m pytest tests -q
Write-Host ""
Write-Host "준비 완료. 학습 예:  $Venv\Scripts\python.exe -m picotopi.train --seed 0 --out runs\seed0"
