# Disk safety (avoid another wipe)

When the disk hits **100% full**, the kernel and editors can **truncate files to 0 bytes** (you lost `train_menu.sh`, `package.xml`, etc.).

## Do this on the machine

```bash
# Check space
df -h /

# If syslog is huge (common on this project):
sudo truncate -s 0 /var/log/syslog
sudo journalctl --vacuum-size=500M

# ROS logs
rm -rf ~/.ros/log/*
```

## Optional: limit log growth (sudo)

```bash
sudo tee /etc/logrotate.d/ros2-anon <<'EOF'
~/.ros/log/*.log {
    daily
    rotate 3
    compress
    missingok
    notifempty
}
EOF
```

## What git does **not** replace

- Running training checkpoints still live on disk; commit `trained_models/` or copy `thesis_locked_*` to USB/cloud.
- Replay buffers are **not** saved — only `.pth` actors.

After freeing space, run:

```bash
bash ~/ros2_ws/scripts/check_repo_integrity.sh
```
