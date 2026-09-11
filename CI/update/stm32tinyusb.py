import argparse
import fnmatch
import json
import os
import shutil
import stat
import subprocess
import sys
import time
from pathlib import Path

from jinja2 import Environment, FileSystemLoader

script_path = Path(__file__).parent.resolve()
sys.path.append(str(script_path.parent))

from utils import (
    createFolder,
    copyFolder,
    copyFile,
    execute_cmd,
    defaultConfig,
)

# GitHub
gh_tinyusb = "hathach/tinyusb"

# Current core repo: script is expected to run from inside the core
core_path = script_path.parent.parent.resolve()

# Persistent local clone path, beside the core repo
repo_local_path = core_path
tinyusb_repo_path = core_path.parent / "tinyusb"

# Destination in the core
tinyusb_dest_path = core_path / "system" / "Middlewares" / "tinyusb"

# Middleware wrapper output path
tinyusb_lib_out_path = core_path / "libraries" / "tinyusb" / "src" / "tinyusb"

# Template
templates_dir = script_path / "templates"
mw_template_file = "mw-tinyusb.c"

j2_env = Environment(
    loader=FileSystemLoader(str(templates_dir)),
    trim_blocks=True,
    lstrip_blocks=True,
)
mw_template = j2_env.get_template(mw_template_file)

# Allowlist relative to tinyusb_dest_path
tinyusb_allowlist = {
    "LICENSE",
    "src/tusb.c",
    "src/tusb.h",
    "src/tusb_option.h",
    "src/common/**",
    "src/device/**",
    "src/osal/osal.h",
    "src/osal/osal_none.h",
    "src/class/cdc/cdc.h",
    "src/class/cdc/cdc_device.c",
    "src/class/cdc/cdc_device.h",
    "src/class/cdc/cdc_rndis.h",
    "src/class/dfu/dfu.h",
    "src/class/dfu/dfu_device.c",
    "src/class/dfu/dfu_device.h",
    "src/class/dfu/dfu_rt_device.c",
    "src/class/dfu/dfu_rt_device.h",
    "src/class/hid/hid.h",
    "src/class/hid/hid_device.c",
    "src/class/hid/hid_device.h",
    "src/class/midi/midi.h",
    "src/class/midi/midi_device.c",
    "src/class/midi/midi_device.h",
    "src/class/midi/midi2_device.c",
    "src/class/midi/midi2_device.h",
    "src/class/msc/msc.h",
    "src/class/msc/msc_device.c",
    "src/class/msc/msc_device.h",
    "src/class/vendor/vendor_device.c",
    "src/class/vendor/vendor_device.h",
    "src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c",
    "src/portable/st/stm32_fsdev/fsdev_common.c",
    "src/portable/st/stm32_fsdev/fsdev_common.h",
    "src/portable/st/stm32_fsdev/fsdev_stm32.h",
    "src/portable/synopsys/dwc2/dcd_dwc2.c",
    "src/portable/synopsys/dwc2/dwc2_common.c",
    "src/portable/synopsys/dwc2/dwc2_common.h",
    "src/portable/synopsys/dwc2/dwc2_stm32.h",
    "src/portable/synopsys/dwc2/dwc2_type.h",
}


def _on_rm_error(func, path, exc_info):
    try:
        os.chmod(path, stat.S_IWRITE)
        func(path)
    except OSError:
        pass


def emptyFolder(path: Path, retries: int = 10, delay: float = 0.5):
    if not path.exists():
        return

    if not path.is_dir():
        print(f"Error: {path} is not a directory")
        sys.exit(1)

    last_error = None

    for _ in range(retries):
        try:
            for item in path.iterdir():
                if item.is_dir():
                    shutil.rmtree(item, onerror=_on_rm_error)
                else:
                    try:
                        item.unlink()
                    except OSError:
                        os.chmod(item, stat.S_IWRITE)
                        item.unlink()
            return
        except OSError as e:
            last_error = e
            time.sleep(delay)

    err = f": {last_error}" if last_error else ""
    print(f"Error: Failed to empty directory {path}{err}")
    sys.exit(1)


def checkConfig():
    global repo_local_path
    global tinyusb_repo_path

    config_file_path = script_path / "update_config.json"

    if config_file_path.is_file():
        try:
            with open(config_file_path, "r") as config_file:
                path_config = json.load(config_file)

            if "REPO_LOCAL_PATH" not in path_config:
                path_config["REPO_LOCAL_PATH"] = str(core_path)
                defaultConfig(config_file_path, path_config)
            else:
                repo_local_path = Path(path_config["REPO_LOCAL_PATH"]).resolve()

        except IOError:
            print(f"Failed to open {config_file_path}!")
            sys.exit(1)
    else:
        defaultConfig(config_file_path, {"REPO_LOCAL_PATH": str(core_path)})

    # Store TinyUSB beside the configured local repo path
    tinyusb_repo_path = repo_local_path.parent / "tinyusb"
    createFolder(tinyusb_repo_path.parent)


def isRepoClean(repo_path: Path) -> bool:
    status = execute_cmd(
        ["git", "-C", str(repo_path), "status", "--porcelain"],
        None,
    ).strip()
    return status == ""


def isTinyUSBPathAllowed(rel_path: Path) -> bool:
    rel = rel_path.as_posix()
    return any(fnmatch.fnmatch(rel, pattern) for pattern in tinyusb_allowlist)


def pruneTinyUSB():
    print("Pruning TinyUSB middleware using STM32 allowlist...")

    removed_files = 0
    removed_dirs = 0

    for fp in sorted(tinyusb_dest_path.rglob("*")):
        if fp.is_file():
            rel_path = fp.relative_to(tinyusb_dest_path)
            if not isTinyUSBPathAllowed(rel_path):
                try:
                    fp.unlink()
                except OSError:
                    os.chmod(fp, stat.S_IWRITE)
                    fp.unlink()
                removed_files += 1

    dir_list = [dp for dp in tinyusb_dest_path.rglob("*") if dp.is_dir()]
    dir_list.sort(key=lambda p: len(p.parts), reverse=True)

    for dp in dir_list:
        if not any(dp.iterdir()):
            dp.rmdir()
            removed_dirs += 1

    print(f"Removed {removed_files} file(s).")
    print(f"Removed {removed_dirs} empty directorie(s).")


def checkGhCli():
    try:
        execute_cmd(["gh", "--version"], None)
    except Exception:
        print("GitHub CLI 'gh' is not available!")
        sys.exit(1)


def checkCoreRepo():
    if not core_path.exists():
        print(f"Could not find core repo: {core_path}!")
        sys.exit(1)

    if not (core_path / ".git").exists():
        print(f"{core_path} is not a git repository!")
        sys.exit(1)

    print("Checking current core repository...")
    if not isRepoClean(core_path):
        print("Core repository has modified or new files!")
        sys.exit(1)

    branch = execute_cmd(
        ["git", "-C", str(core_path), "rev-parse", "--abbrev-ref", "HEAD"],
        None,
    ).strip()
    print(f"Current branch: {branch}")


def createBranch():
    bname = upargs.branch

    bname_list = [
        bn[2:]
        for bn in execute_cmd(
            ["git", "-C", str(core_path), "branch", "--list"],
            None,
        ).splitlines()
    ]

    if bname in bname_list:
        print(f"Branch {bname} already exists, checking it out...")
        execute_cmd(["git", "-C", str(core_path), "checkout", bname], None)
    else:
        print(f"Creating branch {bname}...")
        execute_cmd(["git", "-C", str(core_path), "checkout", "-b", bname], None)

    status = execute_cmd(
        ["git", "-C", str(core_path), "rev-parse", "--abbrev-ref", "HEAD"],
        None,
    ).strip()
    if status != bname:
        print(f"Failed to switch to branch {bname}!")
        sys.exit(1)


def latestTag(repo_path: Path) -> str:
    tags = execute_cmd(
        ["git", "-C", str(repo_path), "tag", "--sort=-v:refname"],
        None,
    ).splitlines()

    if not tags:
        print(f"Could not find any tag in {repo_path}!")
        sys.exit(1)

    version_tag = tags[0].strip()

    execute_cmd(
        ["git", "-C", str(repo_path), "checkout", "-f", version_tag],
        subprocess.DEVNULL,
    )

    return version_tag


def prepareTinyUSBRepo() -> str:
    print(f"Preparing local TinyUSB repository: {tinyusb_repo_path}")

    if tinyusb_repo_path.exists():
        execute_cmd(["git", "-C", str(tinyusb_repo_path), "fetch", "--tags", "--force", "--prune"], None)
        execute_cmd(["git", "-C", str(tinyusb_repo_path), "reset", "--hard"], None)
        execute_cmd(["git", "-C", str(tinyusb_repo_path), "clean", "-fdx"], None)
    else:
        execute_cmd(["gh", "repo", "clone", gh_tinyusb, str(tinyusb_repo_path)], None)
        execute_cmd(["git", "-C", str(tinyusb_repo_path), "fetch", "--tags", "--force", "--prune"], None)

    return latestTag(tinyusb_repo_path)


def findLicenseFile(repo_path: Path) -> Path:
    candidates = [
        repo_path / "LICENSE",
        repo_path / "LICENSE.txt",
        repo_path / "LICENSE.md",
    ]

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    print(f"No LICENSE file found in {repo_path}!")
    sys.exit(1)


def commitIfNeeded(commit_msg: str) -> bool:
    # Stage all changes first
    add_res = subprocess.run(
        ["git", "-C", str(core_path), "add", "--all"],
        capture_output=True,
        text=True,
        check=False,
    )
    if add_res.returncode != 0:
        output = (add_res.stdout + "\n" + add_res.stderr).strip()
        print("git add failed with the following output:")
        print(output)
        sys.exit(add_res.returncode)

    # Check if anything is staged
    diff_res = subprocess.run(
        ["git", "-C", str(core_path), "diff", "--cached", "--name-only"],
        capture_output=True,
        text=True,
        check=False,
    )
    if diff_res.returncode != 0:
        output = (diff_res.stdout + "\n" + diff_res.stderr).strip()
        print("git diff --cached failed with the following output:")
        print(output)
        sys.exit(diff_res.returncode)

    staged_files = diff_res.stdout.strip()
    if not staged_files:
        print("No staged changes detected. Skipping commit.")
        return False

    print("Staged files:")
    print(staged_files)

    commit_res = subprocess.run(
        [
            "git",
            "-C",
            str(core_path),
            "commit",
            "--all",
            "--signoff",
            f"--message={commit_msg}",
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    if commit_res.returncode != 0:
        output = (commit_res.stdout + "\n" + commit_res.stderr).strip()
        print("git commit failed with the following output:")
        print(output)
        sys.exit(commit_res.returncode)

    # Optional: keep same behavior as utils.commitFiles()
    rebase_res = subprocess.run(
        ["git", "-C", str(core_path), "rebase", "--whitespace=fix", "HEAD~1"],
        capture_output=True,
        text=True,
        check=False,
    )
    if rebase_res.returncode != 0:
        output = (rebase_res.stdout + "\n" + rebase_res.stderr).strip()
        print("git rebase --whitespace=fix failed with the following output:")
        print(output)
        sys.exit(rebase_res.returncode)

    print("Changes committed successfully.")
    return True


def applyTinyUSBPatch():
    patch_path = script_path / "patch" / "tinyusb"
    patch_list = sorted(patch_path.glob("*.patch")) if patch_path.is_dir() else []

    if not patch_list:
        print("No patches to apply.")
        return

    patch_failed = []
    print(
        f"Apply {len(patch_list)} patch{'' if len(patch_list) == 1 else 'es'} for tinyusb"
    )

    for patch in patch_list:
        check_cmd = [
            "git",
            "-C",
            str(core_path),
            "apply",
            "--check",
            str(patch),
        ]
        check_res = subprocess.run(
            check_cmd,
            capture_output=True,
            text=True,
        )

        if check_res.returncode != 0:
            output = (check_res.stdout + check_res.stderr).strip()
            patch_failed.append([patch, output if output else "git apply --check failed"])
            continue

        am_cmd = [
            "git",
            "-C",
            str(core_path),
            "am",
            "--keep-non-patch",
            "--quiet",
            "--signoff",
            str(patch),
        ]
        am_res = subprocess.run(
            am_cmd,
            capture_output=True,
            text=True,
        )

        if am_res.returncode != 0:
            subprocess.run(
                ["git", "-C", str(core_path), "am", "--abort"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            output = (am_res.stdout + am_res.stderr).strip()
            patch_failed.append([patch, output if output else "git am failed"])

    if patch_failed:
        for patch, output in patch_failed:
            print(f"Failed to apply {patch}:\n{output}")


def resolveTinyUSBSrc(base_path: Path) -> Path:
    candidates = [
        base_path / "src",
        base_path / "Src",
    ]

    for candidate in candidates:
        if candidate.is_dir():
            return candidate

    print(f"Could not find TinyUSB source directory in {base_path}!")
    print("Expected one of:")
    for candidate in candidates:
        print(f"  - {candidate}")
    sys.exit(1)


def listCFiles(src_root: Path) -> list[Path]:
    return sorted([fp for fp in src_root.rglob("*.c") if fp.is_file()])


def generateWrapper(src_root: Path, dst_root: Path, src_file: Path):
    rel_path = src_file.relative_to(src_root)
    dst_dir = dst_root / rel_path.parent
    dst_file = dst_dir / f"mw-{src_file.name}"

    createFolder(dst_dir)

    with open(dst_file, "w", newline="\n", encoding="utf-8") as out_file:
        out_file.write(
            mw_template.render(
                include_path=rel_path.as_posix(),
            )
        )


def generateTinyUSBMiddlewareWrappers(src_root: Path, dst_root: Path):
    file_list = listCFiles(src_root)

    if not file_list:
        print(f"No .c files found in {src_root}!")
        sys.exit(1)

    print(f"TinyUSB source root: {src_root}")
    print(f"Output wrapper root: {dst_root}")

    createFolder(dst_root)
    emptyFolder(dst_root)

    for src_file in file_list:
        generateWrapper(src_root, dst_root, src_file)

    print(f"Generated {len(file_list)} TinyUSB wrapper file(s).")


def updateTinyUSB():
    tinyusb_tag = prepareTinyUSBRepo()
    print(f"Using tinyusb tag: {tinyusb_tag}")

    src_path = tinyusb_repo_path / "src"
    if not src_path.is_dir():
        print(f"Could not find src folder in {tinyusb_repo_path}!")
        sys.exit(1)

    license_path = findLicenseFile(tinyusb_repo_path)

    print(f"Cleaning destination: {tinyusb_dest_path}")
    createFolder(tinyusb_dest_path)
    time.sleep(0.5)
    emptyFolder(tinyusb_dest_path)

    print("Copying tinyusb src/ ...")
    copyFolder(src_path, tinyusb_dest_path / "src")

    print("Copying tinyusb LICENSE ...")
    copyFile(license_path, tinyusb_dest_path / "LICENSE")

    pruneTinyUSB()

    import_commit_msg = f"""system(tinyusb): update middleware to {tinyusb_tag}

Imported from https://github.com/hathach/tinyusb
Pruned with STM32 Arduino allowlist"""

    commitIfNeeded(import_commit_msg)
    applyTinyUSBPatch()

    src_root = resolveTinyUSBSrc(tinyusb_dest_path)
    generateTinyUSBMiddlewareWrappers(src_root, tinyusb_lib_out_path)

    wrapper_commit_msg = f"""system(tinyusb): regenerate middleware wrappers for {tinyusb_tag}

Generated from system/Middlewares/tinyusb sources"""

    commitIfNeeded(wrapper_commit_msg)


upparser = argparse.ArgumentParser(
    description="Update system/Middlewares/tinyusb from hathach/tinyusb, prune it with an STM32 allowlist, then generate middleware wrappers"
)

upparser.add_argument(
    "-b",
    "--branch",
    default="tinyusb_update",
    help="branch name to create/use for the update",
)

upargs = upparser.parse_args()


def main():
    checkConfig()
    checkGhCli()
    checkCoreRepo()
    createBranch()
    updateTinyUSB()


if __name__ == "__main__":
    main()
