# 0006. x11vnc with Xorg and a forced HDMI output for headless access

- Status: Accepted
- Date: 2026-10-27

## Context

The Pi is usually operated without a monitor, but RViz needs a desktop. Remote access must show the real desktop session, start automatically after boot, and have a usable resolution with no monitor attached.

## Decision

- GDM autologin for `robotpi` with Wayland disabled, because x11vnc only works with Xorg.
- x11vnc as a systemd service (`x11vnc-auto.sh`) that waits for the running Xorg, reads its display and auth file, and serves it on port 5900.
- `video=HDMI-A-1:1920x1080@60D` in `cmdline.txt` forces a 1920×1080 output so Xorg has a resolution headless.

Files are mirrored in `raspi_config/`.

## Alternatives considered

- **TigerVNC (`vncserver`):** starts a second, separate GNOME session for the same user, which GNOME does not support well (typically only the wallpaper shows). A stray instance was found on the Pi.
- **GNOME Remote Desktop:** works with Wayland, not tried.
- **HDMI dummy plug:** would make the forced output unnecessary, not tried.

## Consequences

- A real monitor must be plugged into **HDMI0** (next to USB-C power). On the other port GNOME sees two screens and may put panel, dock and icons on the invisible one.
- GNOME under Xorg on the Pi is less tested than Wayland.
