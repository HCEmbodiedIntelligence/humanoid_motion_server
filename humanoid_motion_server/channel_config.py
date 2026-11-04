"""Channel configuration parsing and data structures."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional
import yaml


@dataclass
class ChannelConfig:
    name: str
    kind: str  # servo_p, servo_j, move_j, move_p, move_l
    endpoint: str
    priority: int
    group: str
    base_frame: str
    tip_frame: str
    fk_pose_topic: Optional[str] = None


def load_channels(config_path: str | Path) -> List[ChannelConfig]:
    path = Path(config_path).resolve()
    if not path.is_file():
        raise FileNotFoundError(f"channels config file not found: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    raw_channels = data.get("channels", [])
    channels = []
    for raw in raw_channels:
        channels.append(
            ChannelConfig(
                name=str(raw.get("name", "")),
                kind=str(raw.get("kind", "")).lower(),
                endpoint=str(raw.get("endpoint", "")),
                priority=int(raw.get("priority", 0)),
                group=str(raw.get("group", "")),
                base_frame=str(raw.get("base_frame", "")),
                tip_frame=str(raw.get("tip_frame", "")),
                fk_pose_topic=raw.get("fk_pose_topic"),
            )
        )
    return channels
