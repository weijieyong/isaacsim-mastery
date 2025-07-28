# Isaac Sim Mastery

A collection of Isaac Sim examples and robotics simulations using Docker containers.

> [!IMPORTANT]
> This repository is tailored for a specific workflow: developing on a Windows laptop, connecting via SSH to a shared GPU-enabled PC, running Isaac Sim inside a Docker container, and streaming the GUI using the Isaac Sim WebRTC Streaming Client.

## Setup

### Asset Configuration (Optional)

Assets can be downloaded locally and mounted for faster access:
```bash
# Mount local assets (optional)
~/weijie/isaacsim-4-5/isaacsim_assets_4-5:/isaac-sim/isaacsim_assets:rw
```

> [!NOTE]
> This step can be skipped as should be automatically fetched from the NVIDIA asset cloud by default

### Prerequisites

Make the run script executable:
```bash
chmod +x run.sh
```

## Usage

### Quick Start

Run the simulation with:
```bash
./run.sh
```

### Development Setup

For development with VS Code integration:
```bash
docker compose up
```

Then attach VS Code to the running container for easier development.

### Running Scripts

Execute Python scripts inside the container:
```bash
./python.sh <script.py>
```

## Headless Mode Configuration

For headless mode visualization in Docker containers, use the following configuration:

```python
app_config = {
    "headless": True,
    "hide_ui": False,
    "width": 1280,
    "height": 720,
    "renderer": "RaytracedLighting"  # Can also be PathTracing
}

simulation_app = SimulationApp(launch_config=app_config)

# Enable WebRTC streaming (optional) - useful for headless mode visualization
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.livestream.webrtc")
```