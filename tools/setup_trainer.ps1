# 학습(슬레이브) 노트북 준비: 가상환경 + PyTorch + MuJoCo + 테스트.
# 저장소 루트 또는 E:\PICOtoPI 에서 실행:
#   powershell -ExecutionPolicy Bypass -File tools\setup_trainer.ps1                 # 지금: CPU 전용 (저사양 시험)
#   powershell -ExecutionPolicy Bypass -File tools\setup_trainer.ps1 -Mode pascal    # GTX 1080 Ti 외장 GPU 연결 후
#   powershell -ExecutionPolicy Bypass -File tools\setup_trainer.ps1 -Mode modern    # RTX 20xx 이상
# pascal: PyTorch CUDA 12.8 이후 빌드는 Pascal(sm_61)을 뺐으므로 CUDA 12.6 빌드를 씁니다.
#         CUDA 12.6 빌드는 PyTorch 2.14가 마지막이라 torch<2.15로 고정합니다.
# 모드를 바꿔 다시 실행하면 같은 가상환경에서 torch만 교체합니다.
param([ValidateSet("cpu", "pascal", "modern")][string]$Mode = "cpu", [string]$Venv = ".venv_picotopi")
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root
# E:\PICOtoPI 배치(modules\, legacy\)면 import 경로 추가
if (Test-Path "modules\picotopi") { $env:PYTHONPATH = "$Root\modules;$Root\legacy;$env:PYTHONPATH" }

if (-not (Test-Path "$Venv\Scripts\python.exe")) { py -3.10 -m venv $Venv }
$Py = Join-Path $Root "$Venv\Scripts\python.exe"
& $Py -m pip install --upgrade pip
switch ($Mode) {
  "cpu"    { & $Py -m pip install --force-reinstall "torch<2.15" --index-url https://download.pytorch.org/whl/cpu }
  "pascal" { & $Py -m pip install --force-reinstall "torch<2.15" --index-url https://download.pytorch.org/whl/cu126 }
  "modern" { & $Py -m pip install --force-reinstall torch --index-url https://download.pytorch.org/whl/cu128 }
}
& $Py -m pip install "mujoco>=3.1" "numpy<2" pytest

# GPU 모드: cuda가 보이는지 + 실제 커널이 도는지 (Pascal 미지원 빌드면 "no kernel image" 오류)
& $Py -c @"
import torch, mujoco
print('torch', torch.__version__, '| mujoco', mujoco.__version__, '| cuda', torch.cuda.is_available())
if '$Mode' != 'cpu':
    assert torch.cuda.is_available(), 'CUDA 장치가 안 보입니다: 드라이버/외장 GPU 연결 확인'
    print('GPU', torch.cuda.get_device_name(0), 'sm_%d%d' % torch.cuda.get_device_capability(0))
    x = torch.randn(256, 256, device='cuda'); print('커널 시험 OK', float((x @ x).sum()))
"@
if ($LASTEXITCODE -ne 0) { throw "PyTorch 확인 실패" }
& $Py -m pytest tests -q
Write-Host ""
if ($Mode -eq "cpu") {
  Write-Host "저사양 시험 학습:  $Venv\Scripts\python.exe -m picotopi.train --seed 0 --total 20000 --warmup 4000 --workers 2 --log-every 4000 --device cpu --out runs\test0"
} else {
  Write-Host "학습:  $Venv\Scripts\python.exe -m picotopi.train --seed 0 --device cuda --out runs\seed0"
}
