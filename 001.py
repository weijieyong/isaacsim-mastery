from isaacsim import SimulationApp

# app configuration
# for more details:
# /isaac-sim/exts/isaacsim.simulation_app/isaacsim/simulation_app/simulation_app.py
app_config = {
        "headless": True,
        "hide_ui": False,
        "width": 1280,
        "height": 720,
        "renderer": "RaytracedLighting"  # Can also be PathTracing
    }

simulation_app = SimulationApp(launch_config=app_config)

# streaming via WebRTC (optional), useful when using headless mode
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.livestream.webrtc")


# keep it running
while simulation_app._app.is_running() and not simulation_app.is_exiting():
    # Run in realtime mode, we don't specify the step size
    simulation_app.update()

simulation_app.close()  # Cleanup application