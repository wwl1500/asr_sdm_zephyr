#!/usr/bin/env python3
"""Install Zephyr workspace, west modules, and SDK for asr_sdm_zephyr-style layouts.

Run from the repo root. Steps: optional apt deps, venv (west, PyYAML), clone/update
repos from the manifest, ``west init``/``update``/``zephyr-export``, Zephyr Python
requirements, latest sdk-ng SDK (GitHub releases API) or ``--sdk-version``, checksum
verify, then ``zephyr_ws/env.sh``. Linux + apt only.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


DEFAULT_REPO_FILE = "zephyr.repos"
DEFAULT_WORKSPACE_DIR = "zephyr_ws"
DEFAULT_VENV_DIR = ".venv"
DEFAULT_BOARD = "xiao_rp2350/rp2350a/m33"
DEFAULT_APP_PATH = "projects/asr_sdm_screw_unit"
DEFAULT_APP_BUILD_PATH = "build/asr_sdm_screw_unit"
SDK_RELEASE_BASE = "https://github.com/zephyrproject-rtos/sdk-ng/releases/download"
SDK_NG_API_LATEST = "https://api.github.com/repos/zephyrproject-rtos/sdk-ng/releases/latest"
SDK_ARCHIVE_SUFFIX = "_linux-x86_64_gnu.tar.xz"
HTTP_TIMEOUT_SEC = 120

APT_PACKAGES = [
    "git",
    "cmake",
    "ninja-build",
    "gperf",
    "ccache",
    "dfu-util",
    "device-tree-compiler",
    "wget",
    "curl",
    "xz-utils",
    "file",
    "make",
    "gcc",
    "gcc-multilib",
    "g++-multilib",
    "libsdl2-dev",
    "libmagic1",
    "python3-dev",
    "python3-pip",
    "python3-setuptools",
    "python3-tk",
    "python3-wheel",
    "python3-venv",
]


def run(cmd: list[str], cwd: Path | None = None) -> None:
    display = shlex.join(cmd)
    where = f" (cwd={cwd})" if cwd is not None else ""
    print(f"+ {display}{where}")
    subprocess.run(cmd, cwd=cwd, check=True)


def warn_unexpected_layout(repo_root: Path) -> None:
    if not (repo_root / "projects").is_dir() or not (repo_root / "modules").is_dir():
        print(
            "WARNING: Current directory does not look like asr_sdm_zephyr "
            "(missing 'projects/' or 'modules/'). Proceeding anyway."
        )


def install_system_dependencies() -> None:
    if sys.platform != "linux":
        raise RuntimeError("This installer currently supports Linux only (apt-based).")
    if shutil.which("apt-get") is None:
        raise RuntimeError("apt-get not found. Please install dependencies manually.")

    run(["sudo", "apt-get", "update"])
    run(
        [
            "sudo",
            "apt-get",
            "install",
            "-y",
            "--no-install-recommends",
            *APT_PACKAGES,
        ]
    )


def ensure_venv(venv_dir: Path) -> tuple[Path, Path]:
    if not (venv_dir / "bin" / "python3").exists():
        run([sys.executable, "-m", "venv", str(venv_dir)])

    venv_python = venv_dir / "bin" / "python3"
    venv_pip = venv_dir / "bin" / "pip"
    run([str(venv_python), "-m", "pip", "install", "--upgrade", "pip"])
    run([str(venv_pip), "install", "west", "pyyaml"])
    return venv_python, venv_pip


def _load_repos_yaml(repo_file: Path, venv_python: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore

        with repo_file.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except ModuleNotFoundError:
        snippet = (
            "import json, sys, yaml; "
            "print(json.dumps(yaml.safe_load(open(sys.argv[1], 'r', encoding='utf-8'))))"
        )
        proc = subprocess.run(
            [str(venv_python), "-c", snippet, str(repo_file)],
            check=True,
            text=True,
            capture_output=True,
        )
        return json.loads(proc.stdout)


def load_repos(repo_file: Path, venv_python: Path) -> dict[str, dict[str, Any]]:
    if not repo_file.exists():
        raise FileNotFoundError(f"Missing repos file: {repo_file}")

    data = _load_repos_yaml(repo_file, venv_python)
    if not isinstance(data, dict):
        raise RuntimeError(f"Invalid repos file content: {repo_file}")

    repos = data.get("repositories")
    if not isinstance(repos, dict) or not repos:
        raise RuntimeError(f"No repositories found in: {repo_file}")
    return repos


def sync_repo(workspace_dir: Path, name: str, info: dict[str, Any]) -> Path:
    repo_type = str(info.get("type", "git"))
    if repo_type != "git":
        raise RuntimeError(f"Unsupported repository type for '{name}': {repo_type}")

    url = str(info.get("url", "")).strip()
    if not url:
        raise RuntimeError(f"Repository '{name}' has no url")

    version = str(info.get("version", "main"))
    repo_rel = Path(str(info.get("path", name)))
    repo_dir = workspace_dir / repo_rel
    repo_dir.parent.mkdir(parents=True, exist_ok=True)

    if repo_dir.exists() and (repo_dir / ".git").is_dir():
        g = ["git", "-C", str(repo_dir)]
        run(g + ["fetch", "--tags", "--prune", "origin"])
        run(g + ["fetch", "origin", version])
        run(g + ["checkout", "--force", "FETCH_HEAD"])
    elif repo_dir.exists():
        raise RuntimeError(f"Path exists and is not a git repo: {repo_dir}")
    else:
        run(["git", "clone", "--branch", version, "--depth", "1", url, str(repo_dir)])

    return repo_dir


def west_cmd(venv_python: Path) -> list[str]:
    return [str(venv_python), "-m", "west"]


def setup_west(workspace_dir: Path, zephyr_repo_dir: Path, venv_python: Path) -> None:
    west = west_cmd(venv_python)

    if not (workspace_dir / ".west").exists():
        if not zephyr_repo_dir.is_dir():
            raise RuntimeError(f"Zephyr repo not found: {zephyr_repo_dir}")
        repo_rel = os.path.relpath(zephyr_repo_dir, workspace_dir)
        run(west + ["init", "-l", repo_rel], cwd=workspace_dir)

    run(west + ["update"], cwd=workspace_dir)
    run(west + ["zephyr-export"], cwd=workspace_dir)


def install_zephyr_python_requirements(venv_pip: Path, zephyr_repo_dir: Path) -> None:
    req = zephyr_repo_dir / "scripts" / "requirements.txt"
    if not req.exists():
        raise RuntimeError(f"Zephyr requirements file not found: {req}")
    run([str(venv_pip), "install", "-r", str(req)])


def download_file(url: str, output: Path) -> None:
    if shutil.which("wget"):
        run(["wget", "-O", str(output), url])
        return
    if shutil.which("curl"):
        run(["curl", "-fL", url, "-o", str(output)])
        return
    raise RuntimeError("Neither wget nor curl is available for downloading files.")


def parse_expected_hash(sum_file: Path, filename: str) -> str:
    for raw_line in sum_file.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        digest = parts[0]
        listed_name = parts[1].lstrip("*")
        if listed_name == filename:
            return digest.lower()
    raise RuntimeError(f"Checksum for '{filename}' not found in {sum_file}")


def sha256_file(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            hasher.update(chunk)
    return hasher.hexdigest()


def fetch_latest_sdk_release() -> tuple[str, str, str, str]:
    """Resolve the latest sdk-ng release: (version, archive_name, archive_url, sums_url)."""
    headers = {
        "Accept": "application/vnd.github+json",
        "User-Agent": "asr_sdm_zephyr-install_zephyr.py",
    }
    token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    if token:
        headers["Authorization"] = f"Bearer {token}"

    req = urllib.request.Request(SDK_NG_API_LATEST, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_SEC) as resp:
            payload = json.load(resp)
    except urllib.error.HTTPError as exc:
        raise RuntimeError(
            f"Failed to fetch latest sdk-ng release from GitHub (HTTP {exc.code}). "
            "If rate-limited, set GITHUB_TOKEN or try again later."
        ) from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(
            f"Could not reach GitHub to resolve latest Zephyr SDK: {exc.reason}"
        ) from exc

    assets = {a["name"]: a["browser_download_url"] for a in payload.get("assets", [])}
    archive_name = next(
        (
            n
            for n in assets
            if n.startswith("zephyr-sdk-") and n.endswith(SDK_ARCHIVE_SUFFIX)
        ),
        None,
    )
    if not archive_name:
        raise RuntimeError(
            "Latest sdk-ng release has no zephyr-sdk-*_linux-x86_64_gnu.tar.xz asset."
        )

    tag = str(payload.get("tag_name", ""))
    if not tag:
        raise RuntimeError("GitHub release response missing tag_name.")

    archive_url = assets[archive_name]
    sums_url = assets.get("sha256.sum")
    if not sums_url:
        sums_url = f"{SDK_RELEASE_BASE}/{tag}/sha256.sum"

    version = archive_name[len("zephyr-sdk-") : -len(SDK_ARCHIVE_SUFFIX)]
    return version, archive_name, archive_url, sums_url


def install_sdk(workspace_dir: Path, sdk_version: str | None) -> Path:
    if sdk_version is None:
        sdk_version, archive_name, archive_url, sums_url = fetch_latest_sdk_release()
        print(f"Zephyr SDK (latest): {sdk_version}")
    else:
        archive_name = f"zephyr-sdk-{sdk_version}{SDK_ARCHIVE_SUFFIX}"
        release_prefix = f"{SDK_RELEASE_BASE}/v{sdk_version}"
        archive_url = f"{release_prefix}/{archive_name}"
        sums_url = f"{release_prefix}/sha256.sum"

    sdk_dir = workspace_dir / f"zephyr-sdk-{sdk_version}"
    archive_path = workspace_dir / archive_name
    sums_path = workspace_dir / f"sha256-v{sdk_version}.sum"

    if not sdk_dir.exists():
        download_file(archive_url, archive_path)
        download_file(sums_url, sums_path)

        expected = parse_expected_hash(sums_path, archive_name)
        actual = sha256_file(archive_path)
        if actual != expected:
            raise RuntimeError(
                f"SDK archive checksum mismatch for {archive_name}:\n"
                f"expected: {expected}\nactual:   {actual}"
            )

        run(["tar", "-xf", str(archive_path)], cwd=workspace_dir)

        if not sdk_dir.exists():
            candidates = sorted(
                p
                for p in workspace_dir.glob(f"zephyr-sdk-{sdk_version}*")
                if p.is_dir()
            )
            if not candidates:
                raise RuntimeError("SDK archive extracted, but SDK directory was not found.")
            if candidates[0] != sdk_dir:
                candidates[0].rename(sdk_dir)

    setup_script = sdk_dir / "setup.sh"
    if not setup_script.exists():
        raise RuntimeError(f"SDK setup script not found: {setup_script}")

    run(["bash", str(setup_script), "-h", "-c", "-t", "arm-zephyr-eabi"], cwd=sdk_dir)
    return sdk_dir


def write_env_sh(workspace_dir: Path, sdk_dir: Path) -> Path:
    env_sh = workspace_dir / "env.sh"
    content = f"""#!/usr/bin/env bash
# Generated by install_zephyr.py. Source this file in your shell.
SCRIPT_DIR="$(cd -- "$(dirname -- "${{BASH_SOURCE[0]}}")" && pwd)"
REPO_ROOT="$(cd -- "${{SCRIPT_DIR}}/.." && pwd)"
export ZEPHYR_BASE="${{SCRIPT_DIR}}/zephyr"
export ZEPHYR_TOOLCHAIN_VARIANT="zephyr"
export ZEPHYR_SDK_INSTALL_DIR="${{SCRIPT_DIR}}/{sdk_dir.name}"
export PATH="${{REPO_ROOT}}/.venv/bin:${{PATH}}"
"""
    env_sh.write_text(content, encoding="utf-8")
    env_sh.chmod(0o755)
    return env_sh


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Install Zephyr workspace and SDK.")
    p.add_argument(
        "--repo-file",
        default=DEFAULT_REPO_FILE,
        help=f"YAML repos manifest (default: {DEFAULT_REPO_FILE})",
    )
    p.add_argument(
        "--workspace-dir",
        default=DEFAULT_WORKSPACE_DIR,
        help=f"Zephyr workspace directory (default: {DEFAULT_WORKSPACE_DIR})",
    )
    p.add_argument(
        "--venv-dir",
        default=DEFAULT_VENV_DIR,
        help=f"Python venv directory (default: {DEFAULT_VENV_DIR})",
    )
    p.add_argument(
        "--sdk-version",
        default=None,
        metavar="VER",
        help=(
            "Pin Zephyr SDK to this version (e.g. 0.16.5). "
            "Default: latest published sdk-ng release (from GitHub)."
        ),
    )
    p.add_argument(
        "--skip-apt",
        action="store_true",
        help="Skip apt dependency installation.",
    )
    p.add_argument(
        "--zephyr-ref",
        default=None,
        metavar="REF",
        help=(
            "Git ref for the zephyr repository (branch or tag), e.g. main or v4.3.0. "
            "Overrides the version in the repos manifest. Default: use manifest."
        ),
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parent
    os.chdir(repo_root)

    repo_file = repo_root / args.repo_file
    workspace_dir = repo_root / args.workspace_dir
    venv_dir = repo_root / args.venv_dir

    warn_unexpected_layout(repo_root)
    workspace_dir.mkdir(parents=True, exist_ok=True)

    if not args.skip_apt:
        install_system_dependencies()

    venv_python, venv_pip = ensure_venv(venv_dir)
    repos = load_repos(repo_file, venv_python)

    zephyr_repo_dir: Path | None = None
    for name, info in repos.items():
        if not isinstance(info, dict):
            raise RuntimeError(f"Invalid repo entry for '{name}'")
        manifest: dict[str, Any] = (
            {**info, "version": args.zephyr_ref}
            if name == "zephyr" and args.zephyr_ref is not None
            else info
        )
        repo_dir = sync_repo(workspace_dir, name, manifest)
        if name == "zephyr":
            zephyr_repo_dir = repo_dir

    if zephyr_repo_dir is None:
        raise RuntimeError("Manifest must contain a 'zephyr' repository entry.")

    setup_west(workspace_dir, zephyr_repo_dir, venv_python)
    install_zephyr_python_requirements(venv_pip, zephyr_repo_dir)
    sdk_dir = install_sdk(workspace_dir, args.sdk_version)
    env_sh = write_env_sh(workspace_dir, sdk_dir)

    print("\nInstallation completed.")
    print("Next commands:")
    print(f"  source {venv_dir}/bin/activate")
    print(f"  source {env_sh}")
    # print(f"  cd {workspace_dir}")
    print(
        "  west build -p always "
        f"-b {DEFAULT_BOARD} -s {DEFAULT_APP_PATH} -d {DEFAULT_BOARD}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as exc:
        print(f"\nERROR: command failed with exit code {exc.returncode}", file=sys.stderr)
        raise SystemExit(exc.returncode)
    except Exception as exc:  # pylint: disable=broad-except
        print(f"\nERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
