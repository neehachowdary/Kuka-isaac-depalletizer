# Setup guide

Base dependencies: **Isaac Sim 6.0.1** (`6.0.1-rc.7+release.42383.32955d8d.gl`) and **curobo `v0.7.8`** — not `main`/v2, which has a totally different API (`MotionPlanner` instead of `MotionGen`, etc.) that this repo's scripts don't use.

All commands below run through `<isaacsim>\python.bat`, not a system Python. Every version pin below was forced by a reproduced failure, noted inline so a redo doesn't rediscover it.

## 1. Isaac Sim

Install 6.0.1 normally (Omniverse/Isaac Sim installer or standalone package).

## 2. curobo v0.7.8 (built from source — no prebuilt wheel)

```
git clone https://github.com/NVlabs/curobo.git
cd curobo
git checkout v0.7.8
```

**Toolchain, in order:**

1. `<isaacsim>\python.bat -m pip install torch==2.8.0+cu128 --index-url https://download.pytorch.org/whl/cu128`
2. CUDA Toolkit **12.8**, matching `cu128` above:
   `winget install --id Nvidia.CUDA --version "12.8" --source winget --accept-source-agreements --accept-package-agreements`
   (Only a *major*-version mismatch hard-fails the build, e.g. a 13.x toolkit against `cu128` torch; minor-only mismatches just warn.)
3. VS 2022 Build Tools, C++ workload, **plus MSVC toolset 14.29 specifically**:
   `winget install --id Microsoft.VisualStudio.2022.BuildTools --source winget --accept-source-agreements --accept-package-agreements --override "--quiet --wait --add Microsoft.VisualStudio.Workload.VCTools --add Microsoft.VisualStudio.Component.VC.14.29.16.11.x86.x64 --includeRecommended --norestart"`
   (14.44 and 14.40 both hit real `nvcc`/MSVC bugs compiling curobo's `.cu` kernels against CUDA 12.8 — a `cudafe++` crash and a `'std'` ambiguous-symbol error, respectively. 14.29 builds clean.)
4. `<isaacsim>\python.bat -m pip install ninja`, and make sure `<isaacsim>\kit\python\Scripts` is on `PATH`.
   (Without ninja, torch's Windows build fallback compiles `.cu` files with plain `cl.exe` instead of routing them through `nvcc`.)

**Build, pinned to 14.29** (a newer toolset is also installed, and `vcvarsall.bat` defaults to the latest unless told otherwise):

```powershell
$vcvarsall = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat"
cmd /c "`"$vcvarsall`" x64 -vcvars_ver=14.29 && set" > "$env:TEMP\vcvars_out.txt" 2>&1
Get-Content "$env:TEMP\vcvars_out.txt" | ForEach-Object {
    if ($_ -match '^([A-Za-z_][A-Za-z0-9_]*)=(.*)$') {
        Set-Item -Path "Env:$($Matches[1])" -Value $Matches[2] -ErrorAction SilentlyContinue
    }
}
$env:DISTUTILS_USE_SDK = "1"  # trust this environment instead of re-detecting (and re-picking latest) toolset
$env:CUDA_PATH = "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8"
$env:CUDA_HOME = $env:CUDA_PATH
$env:PATH = "$env:CUDA_PATH\bin;<isaacsim>\kit\python\Scripts;$env:PATH"

<isaacsim>\python.bat -m pip install -e .\curobo\ --no-build-isolation
```

**Then re-pin `warp-lang` below 1.13:** the install above pulls the *latest* `warp-lang` as a dependency, which is wrong — curobo v0.7.8 calls the deprecated `warp.torch.device_from_torch(...)`, removed in `warp-lang 1.13`. Fix:

```
<isaacsim>\python.bat -m pip install "warp-lang==1.12.0"
```

## 3. `usd-core`

`plan_palletize.py` only reads a static USD file — it never touches a live Kit scene — so it doesn't need to boot Isaac Sim (`SimulationApp`) at all. `pip install usd-core` gives it `pxr` directly. This also sidesteps a real bug: booting the full app loads Isaac Sim's own bundled `warp`/`torch` copies (via `omni.warp.core`/`omni.isaac.ml_archive`), which silently shadow the versions above and reintroduce version-mismatch crashes.

`fix_box_positions.py`, `fix_conveyor.py`, and `run_depalletize.py` all manipulate the live Kit stage/robot articulation, so they still need the full `SimulationApp` boot — this shortcut is `plan_palletize.py`-only.

## 4. KUKA KR50 R2500 robot config

Not bundled with curobo (its configs cover Franka/UR/Kinova/Jaco/iiwa+Allegro/Techman/"simple" — no KUKA). Needs two files, not part of any public repo:

- `kr50_r2500.urdf` — anywhere; referenced by absolute path from the yml.
- `kr50_r2500.yml` — copied into `<curobo>\src\curobo\content\configs\robot\kr50_r2500.yml` (required location — `get_robot_configs_path()` looks there specifically), with `robot_cfg.kinematics.urdf_path` set to the URDF's absolute path on this machine.

## 5. Warehouse USD scene

`newware_house.usd` must be the populated scene (`/World/box_0`–`box_3`, `/World/conveyor`, `/kr50_r2500`) — a blank stage fails at `fix_box_positions.py` with "accessed schema on invalid prim." The trajectory output directory (e.g. `C:\Users\<you>\warehouse\`) must exist too; it's not created automatically.

## 6. Per-machine paths

All four scripts hardcode absolute paths (USD scene, output directory) — update these for the target machine.

## 7. Run order

```
fix_box_positions.py   # corrects box positions in the USD stage, saves
fix_conveyor.py         # corrects conveyor position in the USD stage, saves
plan_palletize.py       # cuRobo motion planning -> writes depalletize_trajectory.npy / _segments.npy
run_depalletize.py      # boots Isaac Sim, executes the planned pick-and-place
```

## Appendix — confirmed-working versions

| Component | Version |
|---|---|
| Isaac Sim | `6.0.1-rc.7+release.42383.32955d8d.gl` |
| curobo | `v0.7.8` (unmodified source) |
| CUDA Toolkit | `12.8` (`nvcc V12.8.93`) |
| MSVC toolset | `14.29.30133` (pinned explicitly at build time) |
| torch | `2.8.0+cu128` |
| warp-lang | `1.12.0` |
| usd-core | `26.8` |
