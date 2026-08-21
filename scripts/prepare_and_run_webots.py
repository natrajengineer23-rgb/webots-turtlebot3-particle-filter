"""Prepare the TurtleBot3 world and launch Webots correctly on macOS."""
import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_WEBOTS_APP = Path("/Applications/Webots.app")
CONTROLLERS = {
    "mapping": "mapping_controller",
    "localization": "localization_controller",
}


def find_sample_world(webots_app: Path) -> Path:
    resources = webots_app / "Contents/Resources"
    candidates = (
        resources / "projects/robots/robotis/turtlebot/worlds/turtlebot3_burger.wbt",
        resources / "projects/robots/robots/turtlebot/worlds/turtlebot3_burger.wbt",
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = list(resources.glob("projects/**/turtlebot3_burger.wbt"))
    if not matches:
        raise FileNotFoundError(
            "The official turtlebot3_burger.wbt sample was not found inside Webots.app."
        )
    return matches[0]


def set_controller(world_text: str, controller: str) -> str:
    """Set the controller on the first TurtleBot3Burger instance."""
    existing = re.compile(
        r'(TurtleBot3Burger\s*\{.{0,4000}?\bcontroller\s+)"[^"]*"', re.DOTALL
    )
    configured, count = existing.subn(rf'\1"{controller}"', world_text, count=1)
    if count:
        return configured
    opener = re.compile(r"(TurtleBot3Burger\s*\{)")
    configured, count = opener.subn(rf'\1\n  controller "{controller}"', world_text, count=1)
    if not count:
        raise ValueError("No TurtleBot3Burger instance was found in the sample world.")
    return configured


def write_runtime_files(project: Path, python_command: Path) -> None:
    contents = f"[python]\nCOMMAND = {python_command}\n"
    for controller in CONTROLLERS.values():
        path = project / "controllers" / controller / "runtime.ini"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)


def webots_launch_command(webots_app: Path, world: Path) -> list[str]:
    """Build a Launch Services command so Webots initialises its macOS bundle paths."""
    return [
        "/usr/bin/open",
        "-n",
        "-a",
        str(webots_app),
        str(world),
        "--args",
        "--mode=realtime",
    ]


def prepare(mode: str, webots_app: Path, project: Path = PROJECT) -> tuple[Path, Path]:
    if mode == "localization" and not (project / "maps/map.yaml").exists():
        raise FileNotFoundError("Run mapping first: maps/map.yaml does not exist yet.")
    binary = webots_app / "Contents/MacOS/webots"
    if not binary.exists():
        raise FileNotFoundError(f"Webots executable not found at {binary}")
    destination = project / "worlds/turtlebot3_mapping.wbt"
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.exists():
        sample = find_sample_world(webots_app)
        shutil.copy2(sample, destination)
    configured = set_controller(destination.read_text(), CONTROLLERS[mode])
    destination.write_text(configured)
    python_command = project / ".venv/bin/python3"
    if not python_command.exists():
        raise FileNotFoundError("The project virtual environment is missing. Run run_mapping.command again.")
    # Preserve the .venv path.  On macOS this executable is normally a symlink;
    # resolving it produces /Library/Frameworks/.../python3 and bypasses the
    # virtual environment where the project packages are installed.
    write_runtime_files(project, python_command)
    return binary, destination


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=CONTROLLERS)
    parser.add_argument("--webots-app", type=Path, default=DEFAULT_WEBOTS_APP)
    parser.add_argument("--project-dir", type=Path, default=PROJECT)
    parser.add_argument("--prepare-only", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = arguments()
    try:
        binary, world = prepare(args.mode, args.webots_app, args.project_dir.resolve())
    except (FileNotFoundError, ValueError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    if args.prepare_only:
        print(f"Prepared {world}")
        return 0
    # Starting Contents/MacOS/webots directly can make webots:// resources resolve
    # against /Applications/Webots.app/projects instead of the app bundle.  Launch
    # Services initialises the bundle exactly as Finder does.
    subprocess.Popen(webots_launch_command(args.webots_app, world))
    print(f"Webots launched in {args.mode} mode with {world.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
