import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import List, Optional

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = PROJECT_ROOT / 'src'
PARTITIONER_DIR = SRC_DIR / 'partitioner'
PARTITIONER_BUILD_DIR = PARTITIONER_DIR / 'build'
DEFAULT_BIN_DIR = PROJECT_ROOT / 'bin'
DEVELOP_BIN_PARENT = PROJECT_ROOT / 'expr-src'

PYTHON_SCRIPTS = [
    'BV-Parti_launcher.py',
    'control_message.py',
    'coordinator.py',
    'dispatcher.py',
    'leader.py',
    'partition_tree.py',
]

PARTITIONER_BINARY_SOURCE = PARTITIONER_BUILD_DIR / 'src' / 'main' / 'bitwuzla'
BITWUZLA_SOURCE = PROJECT_ROOT / 'linux-pre_built' / 'binaries' / 'bitwuzla-0.8.0-bin'


def log(message: str) -> None:
    print(f"[build.py] {message}")


def ensure_dir(path: Path) -> None:
    if not path.exists():
        log(f"Creating directory: {path}")
        path.mkdir(parents=True, exist_ok=True)


def copy_file(src: Path, dest: Path) -> None:
    if not src.exists():
        raise FileNotFoundError(f"Required file not found: {src}")
    log(f"Copying {src} -> {dest}")
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)


def run_command(command: List[str], cwd: Optional[Path] = None) -> None:
    location = str(cwd) if cwd is not None else os.getcwd()
    log(f"Running command: {' '.join(command)} (cwd={location})")
    result = subprocess.run(
        command,
        cwd=cwd,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.stdout:
        log(result.stdout.decode())
    if result.stderr:
        log(result.stderr.decode())


def configure_and_build_partitioner(build_type: str) -> None:
    log("=== Configuring and Building Partitioner (Bitwuzla) ===")
    configure_script = PARTITIONER_DIR / 'configure.py'
    if not configure_script.exists():
        raise FileNotFoundError(f"Missing configure script: {configure_script}")

    configure_cmd = [
        sys.executable,
        str(configure_script),
        build_type,
        '--no-unit-testing',
        '--no-testing',
        # '--wipe',
    ]
    run_command(configure_cmd, cwd=PARTITIONER_DIR)
    run_command(['meson', 'compile'], cwd=PARTITIONER_BUILD_DIR)


def stage_python_scripts(bin_dir: Path) -> None:
    ensure_dir(bin_dir)
    for script in PYTHON_SCRIPTS:
        src_file = SRC_DIR / script
        dest_file = bin_dir / script
        if not src_file.exists():
            raise FileNotFoundError(f"Missing source file: {src_file}")
        copy_file(src_file, dest_file)


def stage_binaries(bin_dir: Path) -> None:
    binaries_dir = bin_dir / 'binaries'
    ensure_dir(binaries_dir)
    copy_file(PARTITIONER_BINARY_SOURCE, binaries_dir / 'partitioner-bin')
    copy_file(BITWUZLA_SOURCE, binaries_dir / 'bitwuzla-0.8.0-bin')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="BV-Parti build helper.")
    parser.add_argument(
        '--build-type',
        default='debug',
        choices=['release', 'debug', 'debugoptimized'],
        help="Partitioner build type (default: %(default)s).",
    )
    parser.add_argument(
        '--develop',
        metavar='expr_tag',
        help="Use develop output directories under expr-src/<expr_tag>.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    bin_dir = DEFAULT_BIN_DIR
    if args.develop:
        bin_dir = DEVELOP_BIN_PARENT / args.develop
        log(f"Develop mode enabled; using output directory: {bin_dir}")
    else:
        log(f"Using default output directory: {bin_dir}")

    log("=== BV-Parti Build Script Started ===")
    stage_python_scripts(bin_dir)
    configure_and_build_partitioner(args.build_type)
    stage_binaries(bin_dir)
    log("Build completed successfully.")


if __name__ == '__main__':
    try:
        main()
    except subprocess.CalledProcessError as err:
        log(f"Command failed with exit code {err.returncode}: {' '.join(err.cmd)}")
        if err.stdout:
            log(err.stdout.decode())
        if err.stderr:
            log(err.stderr.decode())
        raise SystemExit(1)
    except Exception as exc:
        log(f"Build failed: {exc}")
        raise SystemExit(1)
