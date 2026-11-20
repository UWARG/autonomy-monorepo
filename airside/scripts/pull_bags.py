#!/usr/bin/env python3
"""
List the rosbags on the airside Pi and download the ones you pick.

Plug a laptop into the Pi's Ethernet port and run:

    python3 airside/scripts/pull_bags.py

Works on Linux, macOS and Windows using the ssh and sftp commands that ship
with each. Interrupted downloads resume where they stopped on the next run.
Standard library only, Python 3.7+.
"""

import argparse
import json
import re
import shlex
import signal
import subprocess
import sys
import threading
from datetime import datetime
from pathlib import Path

DEFAULT_HOST = "10.42.0.1"  # the Pi's address on a NetworkManager shared eth0
DEFAULT_REMOTE_DIR = "~/autonomy-monorepo/airside/data/bags"

# Fail in seconds rather than minutes when the cable is unplugged.
_CONNECT_OPTS = ["-o", "ConnectTimeout=5"]

# Everything the Pi sends back is checked against these before it reaches an
# sftp command or a local path, so a broken or hostile listing cannot escape
# --out or inject sftp commands.
_BAG_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*")
_FILE_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*\.mcap|metadata\.yaml")
_UNSAFE_PATH_CHARS = re.compile(r'["\\\x00-\x1f\x7f]')
# Device names Windows reserves in any folder (COM1.mcap would open a serial port).
_WINDOWS_RESERVED = re.compile(r"(CON|PRN|AUX|NUL|COM[0-9]|LPT[0-9])(\..*)?", re.IGNORECASE)
_STATUSES = ("complete", "incomplete", "recording", "empty", "unreadable")


def clean(text):
    """Make text from the Pi safe to print: no terminal control sequences."""
    return re.sub(r"[\x00-\x1f\x7f-\x9f]", "?", str(text))


def safe_name(name, pattern):
    return (isinstance(name, str) and pattern.fullmatch(name) is not None
            and not _WINDOWS_RESERVED.fullmatch(name))


def valid_bag(bag):
    """Check the shape of one bag entry from the Pi's listing."""
    def is_int(v):
        return isinstance(v, int) and not isinstance(v, bool)
    try:
        return (
            bag["status"] in _STATUSES
            and (bag["start_ns"] is None
                 or (is_int(bag["start_ns"]) and 0 <= bag["start_ns"] < 2**63))
            and (bag["duration_s"] is None
                 or (isinstance(bag["duration_s"], (int, float)) and 0 <= bag["duration_s"] < 1e9))
            and isinstance(bag["files"], list)
            and all(isinstance(f, dict) and is_int(f.get("size")) and f["size"] >= 0
                    and is_int(f.get("mtime_ns")) for f in bag["files"])
            and is_int(bag["size"])
        )
    except (KeyError, TypeError):
        return False

# Runs on the Pi with its own python3 and prints a JSON listing. Keep it
# compatible with old Python 3 and free of third-party imports. One bad bag
# must never stop the others from being listed.
REMOTE_LISTER = r'''
import json, os, sys, time
from datetime import datetime, timezone

RECORDING_WINDOW_S = 30

def parse_metadata(text):
    start_ns = duration_ns = None
    parent = None
    for line in text.splitlines():
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())
        if indent == 2:
            parent = stripped.rstrip(":")
        elif indent == 4 and ":" in stripped:
            key, value = (p.strip() for p in stripped.split(":", 1))
            try:
                number = int(value)
            except ValueError:
                continue
            if parent == "duration" and key == "nanoseconds":
                duration_ns = number
            elif parent == "starting_time" and key == "nanoseconds_since_epoch":
                start_ns = number
    return start_ns, duration_ns

def start_from_name(name):
    try:
        start = datetime.strptime(name, "airside_%Y-%m-%dT%H-%M-%SZ")
    except ValueError:
        return None
    return int(start.replace(tzinfo=timezone.utc).timestamp()) * 1000000000

def recorder_running():
    """True if a `ros2 bag record` process is alive (Docker processes show here too)."""
    try:
        pids = [p for p in os.listdir("/proc") if p.isdigit()]
    except OSError:
        return False
    for pid in pids:
        try:
            with open("/proc/%s/cmdline" % pid, "rb") as f:
                args = f.read().split(b"\0")
        except OSError:
            continue
        if b"bag" in args and b"record" in args:
            return True
    return False

def describe(path, name, now):
    files = []
    for f in sorted(os.listdir(path)):
        full = os.path.join(path, f)
        if f.endswith(".mcap") and os.path.isfile(full) and not os.path.islink(full):
            files.append(f)
    mcaps = list(files)
    for f in files + ["metadata.yaml"]:
        full = os.path.join(path, f)
        if os.path.exists(full) and not os.access(full, os.R_OK):
            raise OSError("no read permission on " + f)
    start_ns = duration_ns = None
    meta = os.path.join(path, "metadata.yaml")
    newest = max([os.path.getmtime(os.path.join(path, f)) for f in mcaps]
                 or [os.path.getmtime(path)])
    if os.path.isfile(meta) and not os.path.islink(meta):
        with open(meta, "rb") as f:
            start_ns, duration_ns = parse_metadata(f.read().decode("utf-8", "replace"))
        files.append("metadata.yaml")
        # A power cut can leave metadata.yaml empty or cut short; such a bag
        # needs the same reindex as one with no metadata at all.
        status = "complete" if start_ns is not None and duration_ns is not None else "incomplete"
    elif not mcaps:
        status = "empty"
    else:
        age = now - newest
        status = "recording" if 0 <= age < RECORDING_WINDOW_S else "incomplete"
    if start_ns is None:
        start_ns = start_from_name(name)
    sizes = []
    for f in files:
        st = os.stat(os.path.join(path, f))
        sizes.append({"name": f, "size": st.st_size, "mtime_ns": int(st.st_mtime * 1e9)})
    return {
        "name": name,
        "status": status,
        "start_ns": start_ns,
        "duration_s": None if duration_ns is None else duration_ns / 1e9,
        "size": sum(s["size"] for s in sizes),
        "files": sizes,
        "mtime": newest,
    }

bags_dir = os.path.abspath(os.path.expanduser(sys.argv[1]))
result = {"bags_dir": bags_dir, "exists": True, "is_dir": False, "error": None, "bags": []}
try:
    os.stat(bags_dir)  # raises PermissionError if a parent folder can't be entered
    result["is_dir"] = os.path.isdir(bags_dir)
except FileNotFoundError:
    result["exists"] = False
except OSError as err:
    result["error"] = str(err)
if not result["is_dir"] and not result["error"] and not os.path.exists(bags_dir):
    result["exists"] = False
if result["is_dir"]:
    try:
        names = sorted(os.listdir(bags_dir), reverse=True)
    except OSError as err:
        names = []
        result["error"] = str(err)
    now = time.time()
    for name in names:
        path = os.path.join(bags_dir, name)
        if name.startswith(".") or os.path.islink(path) or not os.path.isdir(path):
            continue
        try:
            result["bags"].append(describe(path, name, now))
        except Exception as err:
            result["bags"].append({"name": name, "status": "unreadable", "error": str(err),
                                   "start_ns": None, "duration_s": None, "size": 0, "files": []})
    # The MCAP writer buffers chunks, so a live bag's files can go a while
    # without changing. If the recorder is running, the newest unfinished bag
    # is the one it is writing.
    # Pick it by file time, not name: the name comes from the clock, which
    # can jump backwards after a reboot on a Pi with no clock battery.
    live = [b for b in result["bags"] if b["status"] in ("recording", "empty")]
    unfinished = [b for b in result["bags"] if b["status"] == "incomplete"]
    if not any(b["status"] == "recording" for b in live) and unfinished and recorder_running():
        newest = max(unfinished + live, key=lambda b: b.get("mtime", 0))
        if newest["status"] == "incomplete":
            newest["status"] = "recording"
print(json.dumps(result))
'''


class Cancelled(Exception):
    pass


def _raise_cancelled(signum, frame):
    raise Cancelled()


def ssh_base(args):
    target = f"{args.user}@{args.host}" if args.user else args.host
    return target, (["-p", str(args.port)] if args.port else [])


def list_remote(args):
    """Run REMOTE_LISTER on the Pi over ssh and return its parsed output."""
    target, port_opts = ssh_base(args)
    remote_cmd = f"python3 - {shlex.quote(args.remote_dir)}"
    try:
        result = subprocess.run(
            ["ssh", *_CONNECT_OPTS, *port_opts, target, remote_cmd],
            input=REMOTE_LISTER,
            stdout=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
    except FileNotFoundError:
        sys.exit("ssh not found. Install OpenSSH (built into Windows 10+, macOS, Linux).")
    if result.returncode == 255:
        sys.exit(
            f"Could not connect to {target}.\n"
            "Check the cable is plugged in, the host and user are right, "
            "and that you can run: ssh " + " ".join([*port_opts, target])
        )
    if result.returncode != 0:
        sys.exit(
            f"Connected to {target}, but listing the bags failed (exit {result.returncode}).\n"
            "See the error above. The Pi needs python3 installed."
        )
    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError:
        sys.exit(
            f"Unexpected output from {target} instead of a bag list:\n"
            f"{clean(result.stdout[:500])}\n"
            "Something in the Pi's shell startup may be printing text."
        )


def human_size(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
        n /= 1024


def human_duration(seconds):
    if seconds is None:
        return "?"
    minutes, secs = divmod(int(round(seconds)), 60)
    return f"{minutes}m {secs:02d}s"


def local_start(start_ns):
    if start_ns is None:
        return "?"
    try:
        return datetime.fromtimestamp(start_ns / 1e9).strftime("%Y-%m-%d %H:%M")
    except (OverflowError, OSError, ValueError):
        return "?"


def print_table(bags):
    print(f"{'#':>3}  {'bag':<30}  {'start (local)':<16}  {'duration':>8}  {'size':>9}  status")
    for i, bag in enumerate(bags, 1):
        print(
            f"{i:>3}  {bag['name']:<30}  {local_start(bag['start_ns']):<16}  "
            f"{human_duration(bag['duration_s']):>8}  {human_size(bag['size']):>9}  "
            f"{bag['status']}"
        )


def usable_bags(listing):
    """Split the listing into bags that can be offered and notes about the rest."""
    usable, notes = [], []
    empty = 0
    seen = set()
    bags = listing.get("bags")
    for bag in bags if isinstance(bags, list) else []:
        name = bag.get("name", "") if isinstance(bag, dict) else ""
        if not safe_name(name, _BAG_NAME) or name.lower() in seen:
            # Windows folders ignore case, so "A" and "a" would share one.
            notes.append(f"skipped {ascii(name)}: unexpected name")
        elif not valid_bag(bag):
            notes.append(f"skipped {name}: malformed entry in the Pi's listing")
        elif bag["status"] == "unreadable":
            notes.append(f"skipped {name}: cannot be read on the Pi ({clean(bag.get('error'))})")
        elif bag["status"] == "empty":
            empty += 1
        elif not all(safe_name(f.get("name"), _FILE_NAME) for f in bag["files"]):
            notes.append(f"skipped {name}: contains unexpected file names")
        else:
            usable.append(bag)
        if isinstance(name, str):
            seen.add(name.lower())
    if empty:
        notes.append(f"{empty} empty bag folder(s) hidden")
    return usable, notes


def parse_selection(text, count):
    """
    Turn "1,3-5", "1 2" or "all" into sorted 0-based indices.

    Raises ValueError with a readable message on bad input.
    """
    text = text.strip().lower()
    if text == "all":
        return list(range(count))
    # "1 - 3" is a range; any other whitespace or comma separates items.
    parts = re.split(r"[,\s]+", re.sub(r"\s*-\s*", "-", text))
    picked = set()
    for part in parts:
        if not part:
            continue
        match = re.fullmatch(r"([0-9]+)(?:-([0-9]+))?", part)
        if not match:
            raise ValueError(f"'{part}' is not a number or range like 3-5")
        lo = int(match.group(1))
        hi = int(match.group(2)) if match.group(2) else lo
        if lo > hi:
            raise ValueError(f"{part} is backwards, did you mean {hi}-{lo}?")
        if not (1 <= lo and hi <= count):
            raise ValueError(f"{part} is out of range 1-{count}")
        picked.update(range(lo - 1, hi))
    if not picked:
        raise ValueError("nothing selected")
    return sorted(picked)


def ask_selection(count):
    while True:
        try:
            text = input("\nBags to download (e.g. 1,3-5 or all, q to quit): ")
        except EOFError:
            print()
            raise Cancelled() from None
        if not text.strip():
            continue
        if text.strip().lower() in ("q", "quit"):
            sys.exit(0)
        try:
            return parse_selection(text, count)
        except ValueError as err:
            print(f"  {err}")


# Per-bag record of each file's size and modified time on the Pi when it was
# downloaded, so a file that changed on the Pi is fetched again rather than
# trusted or appended to.
_STATE_FILE = ".pull_bags_state.json"


def load_state(bag_dir):
    try:
        state = json.loads((bag_dir / _STATE_FILE).read_text())
        return state if isinstance(state, dict) else {}
    except (OSError, ValueError):
        return {}


def save_state(bag_dir, state):
    bag_dir.mkdir(parents=True, exist_ok=True)
    tmp = bag_dir / (_STATE_FILE + ".tmp")
    tmp.write_text(json.dumps(state))
    tmp.replace(bag_dir / _STATE_FILE)


def plan_downloads(bags, out_dir):
    """
    Work out which files still need fetching.

    Returns a list of (bag, file_info, local_path, have_bytes, restart) and the
    number of files already complete locally. restart means the local copy
    can't be trusted as a prefix of the Pi's file and must be replaced.
    """
    todo, done = [], 0
    for bag in bags:
        bag_dir = out_dir / bag["name"]
        state = load_state(bag_dir)
        for f in bag["files"]:
            local = bag_dir / f["name"]
            if f["size"] == 0 and not local.exists():
                bag_dir.mkdir(parents=True, exist_ok=True)
                local.touch()  # sftp has nothing to send for an empty file
            have = local.stat().st_size if local.is_file() else 0
            seen = state.get(f["name"])
            same = isinstance(seen, list) and seen == [f["size"], f["mtime_ns"]]
            growing = bag["status"] == "recording" and f["name"] != "metadata.yaml"
            if have == f["size"] and (same or seen is None):
                done += 1  # no record means an older download; trust the size
            elif growing and have >= f["size"]:
                done += 1
            else:
                # Resume only a copy of this exact version of the file, or of
                # a file that is still being recorded (it only grows).
                restart = have > f["size"] or (have > 0 and not same and not growing)
                todo.append((bag, f, local, 0 if restart else have, restart))
    return todo, done


def sftp_quote(path):
    if _UNSAFE_PATH_CHARS.search(path):
        raise ValueError(f"path contains characters sftp cannot handle: {ascii(path)}")
    return f'"{path}"'


def build_commands(todo, remote_root):
    """One self-contained get per file: a failed command can't affect the next."""
    commands = []
    for bag, f, local, _, _ in todo:
        remote = f"{remote_root}/{bag['name']}/{f['name']}"
        # get -a appends to what is there, or downloads in full if nothing is.
        commands.append(f"get -a {sftp_quote(remote)} {sftp_quote(local.resolve().as_posix())}")
    commands.append("bye")
    return "\n".join(commands) + "\n"


def downloaded_bytes(todo):
    total = 0
    for _, _, local, _, _ in todo:
        try:
            total += local.stat().st_size
        except OSError:
            pass
    return total


def show_progress(todo, expected, stop):
    """Print overall progress by watching the local files grow."""
    start = downloaded_bytes(todo)
    shown = False
    while True:
        finished = stop.wait(0.5)
        done = downloaded_bytes(todo)
        # Stay quiet until data arrives so ssh's password prompt isn't overwritten.
        if done != start or shown:
            shown = True
            done = min(done, expected)
            pct = 100 * done / expected if expected else 100
            print(f"\r  {pct:5.1f}%  {human_size(done)} / {human_size(expected)}   ",
                  end="", flush=True)
        if finished:
            break
    if shown:
        print()


def run_sftp(args, todo, remote_root):
    """Fetch every planned file in one sftp session so the password is asked once."""
    target, port_opts = ssh_base(args)
    port_opts = ["-P", port_opts[1]] if port_opts else []
    try:
        commands = build_commands(todo, remote_root)
    except ValueError as err:
        sys.exit(f"Cannot download: {err}")

    states = {}
    for _, f, local, _, restart in todo:
        local.parent.mkdir(parents=True, exist_ok=True)
        if restart and local.exists():
            local.unlink()
        state = states.setdefault(local.parent, load_state(local.parent))
        state[f["name"]] = [f["size"], f["mtime_ns"]]
    for bag_dir, state in states.items():
        save_state(bag_dir, state)

    expected = sum(f["size"] for _, f, _, _, _ in todo)
    before = downloaded_bytes(todo)
    stop = threading.Event()
    watcher = threading.Thread(target=show_progress, args=(todo, expected, stop), daemon=True)
    try:
        # Commands go in on stdin rather than with -b: batch mode turns off
        # password prompts. stdout is hidden because sftp echoes every command;
        # errors still reach the terminal on stderr.
        proc = subprocess.Popen(
            ["sftp", "-q", *_CONNECT_OPTS, *port_opts, target],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            text=True,
            encoding="utf-8",
            errors="surrogateescape",
        )
    except FileNotFoundError:
        sys.exit("sftp not found. Install OpenSSH (built into Windows 10+, macOS, Linux).")
    watcher.start()
    try:
        proc.communicate(commands)
    except (KeyboardInterrupt, Cancelled):
        stop.set()
        watcher.join()
        print("Download interrupted. Run the same command again to resume.", file=sys.stderr)
        raise
    finally:
        # Whatever happened, sftp must not outlive this script. Ignore further
        # Ctrl+C while stopping it so the cleanup can't be skipped.
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            stop_process(proc)
            stop.set()
            watcher.join()
        finally:
            signal.signal(signal.SIGINT, previous)
    return proc.returncode, before


def stop_process(proc):
    while True:
        try:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
            return
        except (KeyboardInterrupt, Cancelled):
            continue  # a second Ctrl+C must not skip the cleanup


def verify(bags, out_dir):
    """Compare local file sizes with what the Pi reported. Returns True if all match."""
    ok = True
    for bag in bags:
        bad = []
        for f in bag["files"]:
            local = out_dir / bag["name"] / f["name"]
            have = local.stat().st_size if local.is_file() else -1
            good = have >= f["size"] if bag["status"] == "recording" else have == f["size"]
            if not good:
                bad.append(f["name"])
        if bad:
            ok = False
            print(f"  INCOMPLETE {bag['name']}: {', '.join(bad)} (run again to resume)")
        else:
            note = " (copied up to now, still recording)" if bag["status"] == "recording" else ""
            print(f"  ok         {bag['name']}{note}")
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"Pi address (default {DEFAULT_HOST})")
    parser.add_argument("--user", help="Pi login (default: your ssh config or local username)")
    parser.add_argument("--port", type=int, help="ssh port (default 22)")
    parser.add_argument(
        "--remote-dir",
        default=DEFAULT_REMOTE_DIR,
        help=f"bags directory on the Pi (default {DEFAULT_REMOTE_DIR})",
    )
    parser.add_argument("--out", default="bags", help="where to save (default ./bags)")
    parser.add_argument("--select", help='pick without prompting, e.g. "1,3-5" or "all"')
    args = parser.parse_args()
    out_dir = Path(args.out).expanduser()

    listing = list_remote(args)
    if not isinstance(listing, dict):
        sys.exit("Unexpected bag list from the Pi.")
    remote_root = listing.get("bags_dir")
    if (not isinstance(remote_root, str) or not remote_root.startswith("/")
            or _UNSAFE_PATH_CHARS.search(remote_root) or clean(remote_root) != remote_root):
        sys.exit(f"Refusing bags folder path from the Pi: {ascii(remote_root)}")
    if listing.get("error"):
        sys.exit(f"Cannot read {remote_root} on {args.host}: {clean(listing['error'])}")
    if not listing.get("is_dir"):
        reason = "is not a folder" if listing.get("exists") else "does not exist"
        sys.exit(
            f"{remote_root} {reason} on {args.host}. "
            "Pass --remote-dir with the bags folder's path on the Pi."
        )
    bags, notes = usable_bags(listing)
    if bags:
        print(f"Bags in {remote_root} on {args.host}:\n")
        print_table(bags)
    if notes:
        print()
        for note in notes:
            print(f"({note})")
    if not bags:
        sys.exit(f"No bags to download in {remote_root} on {args.host}.")

    if args.select is not None:
        try:
            chosen = [bags[i] for i in parse_selection(args.select, len(bags))]
        except ValueError as err:
            sys.exit(f"--select: {err}")
    else:
        chosen = [bags[i] for i in ask_selection(len(bags))]
    print("\nSelected: " + ", ".join(b["name"] for b in chosen))

    try:
        out_dir.mkdir(parents=True, exist_ok=True)
        todo, done = plan_downloads(chosen, out_dir)
    except OSError as err:
        sys.exit(f"--out {args.out}: {err}")

    total = sum(f["size"] for b in chosen for f in b["files"])
    resuming = sum(1 for _, f, _, have, _ in todo if 0 < have < f["size"])
    print(
        f"{len(todo)} file(s) to fetch ({resuming} resuming), "
        f"{done} already downloaded, {human_size(total)} total"
    )
    if todo:
        try:
            code, before = run_sftp(args, todo, remote_root)
        except OSError as err:
            sys.exit(f"--out {args.out}: {err}")
        if code != 0 and downloaded_bytes(todo) == before:
            sys.exit(f"sftp failed (exit {code}) before downloading anything. See the error above.")

    print(f"\nSaved to {out_dir.resolve()}")
    ok = verify(chosen, out_dir)
    if any(b["status"] in ("incomplete", "recording") for b in chosen):
        print(
            "\nIncomplete or still-recording bags have no usable metadata.yaml. "
            "They may need `ros2 bag reindex <bag> -s mcap` before `ros2 bag play`."
        )
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except AttributeError:
            pass
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _raise_cancelled)
    try:
        main()
    except (KeyboardInterrupt, Cancelled):
        print("\nStopped.", file=sys.stderr)
        sys.exit(130)
