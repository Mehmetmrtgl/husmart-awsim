# Beytepe (Hacettepe) campus — AWSIM extension

This folder adds Hacettepe University's Beytepe campus and the sensors of the Hüsmart vehicle to AWSIM.
Autoware side (sensor kit, map, launch scripts, tests): **[husmart-sim](https://github.com/Mehmetmrtgl/husmart-sim)**.

## Contents

| Folder | What it contains |
|---|---|
| `Editor/BeytepeSceneBuilder.cs` | Builds the scene and the player. Menu: **AWSIM → Beytepe → 1. Sahneyi Kur (build scene) / 2. Oynatıcıyı Derle (build player)**. The scene is copied from the AWSIM demo scene and Shinjuku is replaced with Beytepe. |
| `Editor/BeytepeModelPostprocessor.cs` | Model import settings (textures, colliders). |
| `Runtime/BeytepeSensorRig.cs` | Hüsmart sensors (lidars, ZED X, MTi-7). Poses and models are read from JSON at startup; no rebuild needed. |
| `Runtime/BeytepeNpcEgoAwareness.cs` | NPC vehicles are aware of the ego (cut-ins, yielding at intersections). |
| `Runtime/BeytepeCrosswalkPedestrians.cs` | Pedestrians at crosswalks. |
| `Runtime/BeytepeKasis.cs`, `Prefabs/Kasisler.prefab` | Speed bumps (13, at their real locations). |
| `Runtime/BeytepeBina.cs`, `Prefabs/Binalar.prefab`, `Data/binalar.json` | Buildings (OSM footprints + lidar/OSM heights; facades, eaves, roofs). |
| `Runtime/BeytepeGokyuzu.cs` | Blue gradient sky. |
| `Runtime/BeytepeMti7.cs`, `BeytepeStereoCamera.cs` | Xsens MTi-7 (IMU/GNSS/INS) and ZED X stereo camera simulation. |
| `Model/Beytepe.fbx` | Campus roads and terrain (RoadRunner export). The source `.rrscene` is in husmart-sim `roadrunner/`. |
| `Textures/` | Asphalt, concrete, grass and bark textures ([ambientCG](https://ambientcg.com), CC0). |
| `Model/Gercek/` | Experimental world generated from a real vehicle recording; used only by the `BEYTEPE_WORLD=gercek` build. Its ground tiles are not in the repo. |

`BeytepeSimulation.unity` is **not** in the repo: it is regenerated from `Assets/Awsim/Scenes/AutowareSimulationDemo.unity` on every build.

## Building

Requires Unity **6000.0.61f1** and a Personal license. Do not start Unity from a terminal where ROS 2 is sourced.

    git clone -b beytepe https://github.com/Mehmetmrtgl/husmart-awsim.git ~/awsim/AWSIM-src
    git clone https://github.com/Mehmetmrtgl/husmart-sim.git ~/husmart-sim
    ~/husmart-sim/build_beytepe_awsim.sh        # builds the scene and the player headless (~15-30 min)

Output: `~/awsim/Beytepe-Build/AWSIM-Beytepe.x86_64`. Report: `~/awsim/beytepe_build_report.txt`.
A prebuilt player is available on this repo's **Releases** page.

## Changes to AWSIM's own files

All are marked with a `Beytepe` comment (`git diff main beytepe -- Assets/Awsim Assets/RGLUnityPlugin`).

| File | Why |
|---|---|
| `Awsim/.../Lanelet/IO/OsmToLaneletMap.cs` | Boundary lines stored in reverse in the OSM are re-oriented from the geometry. |
| `Awsim/.../Ros2/DotNetSimulationTimeSource.cs` | Sensor timestamps are locked to Unity physics time. |
| `Awsim/.../Gnss/GnssSensor.cs` | Fixes the MGRS error on a map with a Local projector. |
| `Awsim/.../AccelVehicle/AccelVehicle.cs`, `Ros2/AccelVehicleReportRos2Publisher.cs` | Fixes nonsensical heading_rate while stopped (e.g. 482 rad/s); angular velocity sign. |
| `Awsim/.../NpcVehicle/*` | Hooks for NPCs yielding to the ego (`BeytepeNpcEgoAwareness`). |
| `RGLUnityPlugin/.../LidarModels/*` | Adds the Hesai JT128 lidar model. |
| `RGLUnityPlugin/.../SceneManager.cs`, `RGLMeshObject.cs` | Removes a ~90 ms stall per lidar capture in a scene with ~55k objects (unchanged transforms are not re-sent). |
| `ProjectSettings/*` | Beytepe scene in the build list, layers/tags and quality settings. |

## Data sources and licenses

- AWSIM: code (`*.cs` etc.) is Apache-2.0; AWSIM's assets (`*.fbx`, `*.png`, `*.x86_64` ...) are CC BY-NC — see the root `LICENSE`. This fork is distributed under the same terms; the player on Releases may not be used commercially.
- Building footprints: © OpenStreetMap contributors, [ODbL](https://opendatacommons.org/licenses/odbl/).
- Textures: ambientCG, CC0.
