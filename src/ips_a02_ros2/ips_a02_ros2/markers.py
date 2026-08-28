"""Dựng MarkerArray cho RViz2: anchor, tag, nhãn, vệt di chuyển, vòng nhiễu, vector vận tốc, vùng ảo."""

import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

PALETTE = [
    (0.22, 0.74, 0.97), (0.13, 0.77, 0.37), (0.96, 0.62, 0.07), (0.65, 0.55, 0.98),
    (0.96, 0.45, 0.71), (0.18, 0.83, 0.75), (0.98, 0.44, 0.52), (0.52, 0.80, 0.09),
    (0.38, 0.65, 0.98), (0.98, 0.75, 0.14),
]


def color(i, a=1.0):
    r, g, b = PALETTE[i % len(PALETTE)]
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def _base(frame, ns, mid, mtype, stamp):
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns = ns
    m.id = mid
    m.type = mtype
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.lifetime = Duration(sec=0, nanosec=0)
    return m


def anchor_markers(anchors, frame, stamp, height=0.35):
    """Anchor = khối vuông cam + nhãn tên + cột thả xuống sàn (dễ định vị trong không gian 3D)."""
    arr = MarkerArray()
    for i, a in enumerate(anchors):
        on = a.get("online", True)
        c = ColorRGBA(r=0.96, g=0.62, b=0.07, a=1.0) if on else ColorRGBA(r=0.4, g=0.45, b=0.5, a=1.0)

        m = _base(frame, "anchors", i, Marker.CUBE, stamp)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = a["x"], a["y"], a["z"]
        m.scale.x = m.scale.y = m.scale.z = 0.22
        m.color = c
        arr.markers.append(m)

        t = _base(frame, "anchor_labels", i, Marker.TEXT_VIEW_FACING, stamp)
        t.pose.position.x, t.pose.position.y = a["x"], a["y"]
        t.pose.position.z = a["z"] + height
        t.scale.z = 0.32
        t.color = ColorRGBA(r=1.0, g=0.85, b=0.4, a=1.0)
        t.text = f"{a['name']}  z={a['z']:.1f}"
        arr.markers.append(t)

        p = _base(frame, "anchor_poles", i, Marker.LINE_STRIP, stamp)
        p.scale.x = 0.02
        p.color = ColorRGBA(r=0.96, g=0.62, b=0.07, a=0.35)
        p.points = [Point(x=a["x"], y=a["y"], z=0.0), Point(x=a["x"], y=a["y"], z=a["z"])]
        arr.markers.append(p)
    return arr


def fence_markers(fences, frame, stamp, z=0.02):
    arr = MarkerArray()
    mid = 0
    for f in fences:
        for ring in f.get("bounds", []):
            m = _base(frame, "fences", mid, Marker.LINE_STRIP, stamp)
            m.scale.x = 0.05
            m.color = ColorRGBA(r=0.65, g=0.55, b=0.98, a=0.9)
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=z) for p in ring]
            if m.points and (m.points[0].x, m.points[0].y) != (m.points[-1].x, m.points[-1].y):
                m.points.append(m.points[0])
            arr.markers.append(m)
            if m.points:
                t = _base(frame, "fence_labels", mid, Marker.TEXT_VIEW_FACING, stamp)
                t.pose.position.x = m.points[0].x
                t.pose.position.y = m.points[0].y
                t.pose.position.z = z + 0.3
                t.scale.z = 0.28
                t.color = ColorRGBA(r=0.78, g=0.71, b=1.0, a=1.0)
                t.text = f.get("name", "fence")
                arr.markers.append(t)
            mid += 1
    return arr


def tag_markers(tags, frame, stamp, opts):
    """tags: list các TagState đã sắp xếp ổn định (index quyết định màu + marker id)."""
    arr = MarkerArray()
    for i, t in enumerate(tags):
        fresh = t.online
        a = 1.0 if fresh else 0.35
        c = color(i, a)

        m = _base(frame, "tags", i, Marker.SPHERE, stamp)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = t.x, t.y, t.z
        m.scale.x = m.scale.y = m.scale.z = opts["tag_size"]
        m.color = c
        arr.markers.append(m)

        lb = _base(frame, "tag_labels", i, Marker.TEXT_VIEW_FACING, stamp)
        lb.pose.position.x, lb.pose.position.y = t.x, t.y
        lb.pose.position.z = t.z + opts["tag_size"] + 0.18
        lb.scale.z = opts.get("label_size", 0.26)
        lb.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=a)
        lb.text = t.name if fresh else f"{t.name} (mất tín hiệu)"
        if opts.get("show_coords"):
            lb.text += f"\n{t.x:.2f}, {t.y:.2f}, {t.z:.2f}"
        arr.markers.append(lb)

        if opts["show_noise"]:
            n = _base(frame, "tag_noise", i, Marker.CYLINDER, stamp)
            n.pose.position.x, n.pose.position.y = t.x, t.y
            n.pose.position.z = t.z
            r = max(0.05, t.noise) * 2.0
            n.scale.x = n.scale.y = r
            n.scale.z = 0.01
            n.color = color(i, 0.20 * a)
            arr.markers.append(n)

        if opts["show_trail"] and len(t.trail) > 1:
            tr = _base(frame, "tag_trails", i, Marker.LINE_STRIP, stamp)
            tr.scale.x = 0.04
            tr.color = color(i, 0.75 * a)
            tr.points = [Point(x=p[0], y=p[1], z=p[2]) for p in t.trail]
            arr.markers.append(tr)

        if opts["show_velocity"]:
            sp = math.hypot(t.vx, t.vy)
            v = _base(frame, "tag_vel", i, Marker.ARROW, stamp)
            v.scale.x, v.scale.y, v.scale.z = 0.05, 0.11, 0.14
            v.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.85 * a)
            if sp > 0.05:
                v.points = [Point(x=t.x, y=t.y, z=t.z),
                            Point(x=t.x + t.vx * opts["vel_scale"],
                                  y=t.y + t.vy * opts["vel_scale"], z=t.z)]
            else:
                v.action = Marker.DELETE
            arr.markers.append(v)
    return arr


def delete_markers(namespaces, ids, frame, stamp):
    """Xoá marker của tag đã biến mất khỏi danh sách."""
    arr = MarkerArray()
    for ns in namespaces:
        for i in ids:
            m = _base(frame, ns, i, Marker.SPHERE, stamp)
            m.action = Marker.DELETE
            arr.markers.append(m)
    return arr
