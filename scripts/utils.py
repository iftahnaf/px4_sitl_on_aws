# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025, Iftach Naftaly <iftahnaf@gmail.com>

# this utils are used in rosbags package analysis


from pathlib import Path
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
from rosbags.typesys.store import Typestore


def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)

def get_px4_msgs(px4_msgs_path: str) -> list:
    """List all px4 messages in a given directory"""
    px4_msgs_path = Path(px4_msgs_path)
    if not px4_msgs_path.exists():
        raise FileNotFoundError(f"Path {px4_msgs_path} does not exist")

    msg_files = []
    for path in px4_msgs_path.rglob('*.msg'):
        if path.is_file():
            msg_files.append(path)

    return msg_files

def register_custom_msgs(msgs_files: list) -> Typestore:
    """Register custom messages in the type store."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    add_types = {}

    for pathstr in msgs_files:
        msgpath = Path(pathstr)
        msgdef = msgpath.read_text(encoding='utf-8')
        add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

    typestore.register(add_types)
    return typestore
