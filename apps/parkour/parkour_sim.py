"""Compatibility entry point for the shared G1 MuJoCo simulation launcher."""
from pathlib import Path
import runpy


if __name__ == "__main__":
    project_root = Path(__file__).resolve().parents[2]
    runpy.run_path(str(project_root / "apps/simulation/sim.py"), run_name="__main__")
