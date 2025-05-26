# useful commands

mkdir -p ~/weijie/docker/isaac-sim/{cache/{kit,ov,pip,glcache,computecache},logs,data,documents}

# This script is used to run Isaac Sim 4.5 in a Docker container with specific configurations and volume mounts.
docker run --name isaac-sim-4_5 \
    --entrypoint bash -it --gpus all --rm --network host \
    -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" \
    -v ~/weijie/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/weijie/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/weijie/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/weijie/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/weijie/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/weijie/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/weijie/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/weijie/docker/isaac-sim/documents:/root/Documents:rw \
    -v ~/weijie/my_examples:/isaac-sim/my_examples:rw \
    -v ~/isaacsim_assets_4-5:/isaac-sim/isaacsim_assets:rw \
    nvcr.io/nvidia/isaac-sim:4.5.0
    
---

## Configuring Isaac Sim to Use Local Assets
https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_faq.html 
Edit the /home/<username>/isaacsim/apps/isaacsim.exp.base.kit file and add the settings below:

apps/isaacsim.exp.full.streaming.kit

[settings]
persistent.isaac.asset_root.default = "/isaac-sim/isaacsim_assets/Assets/Isaac/4.5"
exts."isaacsim.asset.browser".folders = [
    "/isaac-sim/isaacsim_assets/Assets/Isaac/4.5/Isaac/Robots",
    "/isaac-sim/isaacsim_assets/Assets/Isaac/4.5/Isaac/People",
    "/isaac-sim/isaacsim_assets/Assets/Isaac/4.5/Isaac/IsaacLab",
    "/isaac-sim/isaacsim_assets/Assets/Isaac/4.5/Isaac/Props",
    "/isaac-sim/isaacsim_assets/Assets/Isaac/4.5/Isaac/Environments",
    "/isaac-sim/isaacsim_assets/Assets/Isaac/4.5/Isaac/Materials",
    "/isaac-sim/isaacsim_assets/Assets/Isaac/4.5/Isaac/Samples",
    "/isaac-sim/isaacsim_assets/Assets/Isaac/4.5/Isaac/Sensors",
]

or

./runheadless.sh --/persistent/isaac/asset_root/default="/isaac-sim/isaacsim_assets/Assets/Isaac/4.5"

./isaac-sim.sh --/persistent/isaac/asset_root/default="/isaac-sim/isaacsim_assets/Assets/Isaac/4.5"



-v <path-in-local>:<path-in-container>:rw \


---

4.0.0 version

docker run --name isaac-sim-4_0 \
    --entrypoint bash -it --gpus all --rm --network host \
    -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" \
    -v ~/weijie/docker/isaac-sim-400/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/weijie/docker/isaac-sim-400/cache/ov:/root/.cache/ov:rw \
    -v ~/weijie/docker/isaac-sim-400/cache/pip:/root/.cache/pip:rw \
    -v ~/weijie/docker/isaac-sim-400/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/weijie/docker/isaac-sim-400/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/weijie/docker/isaac-sim-400/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/weijie/docker/isaac-sim-400/data:/root/.local/share/ov/data:rw \
    -v ~/weijie/docker/isaac-sim-400/documents:/root/Documents:rw \
    -v ~/weijie/IsaacSim-Dataset-Generation:/isaac-sim/IsaacSim-Dataset-Generation:rw \
    nvcr.io/nvidia/isaac-sim:4.0.0

    -e "OMNI_SERVER=omniverse://10.3.150.177/NVIDIA/Assets/Isaac/4.0" \

docker exec -it isaac-sim-4_5 bash

## linux
wget \
  --recursive \
  --no-clobber \
  --page-requisites \
  --convert-links \
  --adjust-extension \
  --no-parent \
  --domains=docs.isaacsim.omniverse.nvidia.com \
  --level=inf \
  --restrict-file-names=windows \
  https://docs.isaacsim.omniverse.nvidia.com/latest/index.html


## windows
wget `
  --recursive `
  --no-clobber `
  --page-requisites `
  --convert-links `
  --adjust-extension `
  --no-parent `
  --domains=docs.isaacsim.omniverse.nvidia.com `
  --level=inf `
  --restrict-file-names=windows `
  "https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/index.html"
 


  #   "https://docs.isaacsim.omniverse.nvidia.com/latest/index.html"


  ./python.sh -m pip install debugpy 

./python.sh -m debugpy --wait-for-client --listen 0.0.0.0:5678 standalone_examples/api/isaacsim.core.api/time_stepping.py

