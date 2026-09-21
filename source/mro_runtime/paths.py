"""Conservative file helpers for our own writes, not an OS sandbox.

Existing ancestors below root must be real directories. Existing writable files
must be regular files with one link. Concurrent malicious replacement of parent
directories is outside this helper's guarantee; do not share writable sessions.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import secrets
import stat


class PathViolation(ValueError):
    pass


def _root(root) -> Path:
    root = Path(root).absolute()
    if root.is_symlink() or not root.is_dir():
        raise PathViolation(f"Workspace must be an existing real directory: {root}")
    return root.resolve(strict=True)


def confined_path(root, value, require_exists=False) -> Path:
    base = _root(root)
    raw = Path(value)
    if ".." in raw.parts:
        raise PathViolation("Parent traversal is forbidden")
    path = raw if raw.is_absolute() else base / raw
    try:
        relative = path.relative_to(base)
    except ValueError as exc:
        raise PathViolation(f"Path is outside workspace: {path}") from exc
    cursor = base
    for index, component in enumerate(relative.parts):
        cursor = cursor / component
        try:
            info = cursor.lstat()
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(info.st_mode):
            raise PathViolation(f"Symlink is forbidden for controlled writes: {cursor}")
        if index < len(relative.parts) - 1 and not stat.S_ISDIR(info.st_mode):
            raise PathViolation(f"Parent is not a directory: {cursor}")
    resolved = path.resolve(strict=require_exists)
    if not resolved.is_relative_to(base):
        raise PathViolation(f"Resolved path is outside workspace: {resolved}")
    return path


def ensure_private_file(root, value) -> Path:
    path = confined_path(root, value, require_exists=True)
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise PathViolation(f"Expected regular file with a single link: {path}")
    return path


def ensure_directory(root, value) -> Path:
    base = _root(root)
    path = confined_path(base, value)
    cursor = base
    for part in path.relative_to(base).parts:
        cursor = cursor / part
        try:
            cursor.mkdir(mode=0o700)
        except FileExistsError:
            pass
        confined_path(base, cursor, require_exists=True)
        if not cursor.is_dir():
            raise PathViolation(f"Expected directory: {cursor}")
    return path


def write_new_bytes(root, value, data: bytes) -> Path:
    if not isinstance(data, bytes):
        raise TypeError("data must be bytes")
    path = confined_path(root, value)
    ensure_directory(root, path.parent)
    confined_path(root, path)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags, 0o600)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    return path


def atomic_replace_json(root, value, payload) -> Path:
    """Replace only a verified private metadata file; never use for RGB data."""
    path = confined_path(root, value)
    ensure_directory(root, path.parent)
    if path.exists():
        ensure_private_file(root, path)
    data = (json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")
    temporary = path.with_name(f".{path.name}.{secrets.token_hex(8)}.tmp")
    write_new_bytes(root, temporary, data)
    try:
        confined_path(root, path)
        if path.exists():
            ensure_private_file(root, path)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            ensure_private_file(root, temporary)
            temporary.unlink()
    return path


def read_json(root, value):
    return json.loads(ensure_private_file(root, value).read_text(encoding="utf-8"))
