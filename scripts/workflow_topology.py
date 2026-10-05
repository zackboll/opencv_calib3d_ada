"""Small fail-closed checks for the repository's workflow routing."""
from pathlib import Path
import re


def _section(text: str, heading: str) -> list[str]:
    lines = text.splitlines()
    start = None
    indent = None
    for i, line in enumerate(lines):
        if line.strip() == f"{heading}:":
            start = i + 1
            indent = len(line) - len(line.lstrip())
            break
    if start is None:
        raise ValueError(f"missing {heading} section")
    out = []
    for line in lines[start:]:
        if line.strip() and len(line) - len(line.lstrip()) <= indent:
            break
        out.append(line)
    return out


def _event_names(text: str) -> set[str]:
    names = set()
    for line in _section(text, "on"):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        indent = len(line) - len(line.lstrip())
        if indent == 2:
            match = re.fullmatch(r"\s{2}([A-Za-z_]+):\s*(?:#.*)?", line)
            if not match:
                raise ValueError(f"unsupported trigger syntax: {line}")
            names.add(match.group(1))
    return names


def _push_is_main_only(text: str) -> bool:
    lines = [line.strip() for line in _section(text, "push")
             if line.strip() and not line.lstrip().startswith("#")]
    return lines in (["branches: [main]"], ["branches:", "- main"])


def _jobs(text: str) -> dict[str, str]:
    lines = _section(text, "jobs")
    result = {}
    current = None
    chunks = []
    for line in lines:
        match = re.match(r"^  ([A-Za-z0-9_-]+):\s*$", line)
        if match:
            if current is not None:
                result[current] = "\n".join(chunks)
            current = match.group(1)
            chunks = []
        elif current is not None:
            chunks.append(line)
    if current is not None:
        result[current] = "\n".join(chunks)
    return result


def check_topology(cross_text: str, windows_text: str, compatibility_text: str) -> None:
    if _event_names(cross_text) != {"pull_request", "push", "workflow_dispatch"}:
        raise ValueError("cross-platform workflow triggers changed")
    if not _push_is_main_only(cross_text):
        raise ValueError("cross-platform push must target main only")
    if _event_names(windows_text) != {"push"} or not _push_is_main_only(windows_text):
        raise ValueError("Windows must be push-to-main only")
    if "pull_request:" in windows_text or "workflow_dispatch:" in windows_text:
        raise ValueError("Windows may not run on PR or manual dispatch")
    if _event_names(compatibility_text) != {"workflow_dispatch"}:
        raise ValueError("compatibility matrix must be manual-only")

    jobs = _jobs(cross_text)
    expected = {
        "repository-checks": "ubuntu-24.04",
        "linux": "ubuntu-24.04",
        "linux-sanitizers": "ubuntu-24.04",
        "macos": "macos-14",
    }
    if set(jobs) != set(expected):
        raise ValueError(f"unexpected PR jobs: {set(jobs)}")
    for name, runner in expected.items():
        if re.search(rf"(?m)^    runs-on:\s*{re.escape(runner)}\s*$", jobs[name]) is None:
            raise ValueError(f"unexpected runner for {name}")
        if "strategy:" in jobs[name]:
            raise ValueError("PR runner matrices require review")


def check_workflows(directory: Path) -> None:
    check_topology(
        (directory / "cross-platform.yml").read_text(),
        (directory / "windows-post-merge.yml").read_text(),
        (directory / "opencv-compatibility.yml").read_text(),
    )
