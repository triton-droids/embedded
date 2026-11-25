"""Render the supervisor snapshot shared by the inline and external publishers.

``safety_node`` can publish ``/safety/status`` itself (``status_publisher:=inline``)
or hand the snapshot to a separate process (``status_publisher:=external``).
Both paths must produce the same DiagnosticArray, so the rendering lives here
instead of being duplicated at the two call sites.
"""
from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

SCHEMA_VERSION = 1


def render(snapshot: dict, stamp=None) -> DiagnosticArray:
    """Build the diagnostics array described by ``snapshot``.

    The snapshot is the JSON payload published on ``/safety/supervisor_state``;
    it is deliberately plain data so the rendering process needs no access to
    the supervisor object.
    """
    array = DiagnosticArray()
    if stamp is not None:
        array.header.stamp = stamp

    latched = bool(snapshot.get("latched", False))
    reason = str(snapshot.get("reason", ""))
    checks = snapshot.get("checks") or []

    summary = DiagnosticStatus()
    summary.name = "humanoid_safety/supervisor"
    summary.hardware_id = "humanoid"
    if latched:
        summary.level = DiagnosticStatus.ERROR
        summary.message = reason
    elif any(not check.get("healthy", False) for check in checks):
        summary.level = DiagnosticStatus.WARN
        summary.message = "safety checks initializing or unhealthy"
    else:
        summary.level = DiagnosticStatus.OK
        summary.message = "all safety checks clear"
    summary.values = [
        KeyValue(key="imu_received", value=str(snapshot.get("imu_received", 0))),
        KeyValue(key="imu_rejected_source", value=str(snapshot.get("imu_rejected_source", 0))),
        KeyValue(key="imu_rejected_orientation",
                 value=str(snapshot.get("imu_rejected_orientation", 0))),
        KeyValue(key="latched", value=str(latched)),
        KeyValue(key="reason", value=reason),
        KeyValue(key="estop_publish_failures",
                 value=str(snapshot.get("estop_publish_failures", 0))),
    ]
    array.status.append(summary)

    for check in checks:
        status = DiagnosticStatus()
        status.name = f"humanoid_safety/{check.get('name', '')}"
        status.hardware_id = "humanoid"
        if check.get("active", False):
            status.level = DiagnosticStatus.ERROR
        elif not check.get("healthy", False):
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.message = str(check.get("reason", ""))
        status.values = [
            KeyValue(key="active", value=str(bool(check.get("active", False)))),
            KeyValue(key="healthy", value=str(bool(check.get("healthy", False)))),
            KeyValue(key="clearable", value=str(bool(check.get("clearable", False)))),
        ]
        array.status.append(status)

    return array
