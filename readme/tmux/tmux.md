# Mastering tmux — Practical Guide & Cheat Sheet

## What is tmux?

tmux is a terminal multiplexer: it lets you run multiple terminal sessions inside a single terminal window.

Core advantages:

* Persistent sessions (survive SSH disconnects)
* Split terminal panes
* Multiple windows/tabs
* Session management
* Remote development workflows
* Keyboard-driven productivity
* Shared terminal sessions
* Automation/scripting

It is one of the most useful tools for Linux development, robotics, servers, and SSH-heavy workflows.

---

# 1. Mental Model of tmux

tmux hierarchy:

```text
tmux server
 └── sessions
      └── windows
           └── panes
```

## Session

A workspace/project.

Example:

* One session for MoveIt
* One session for robotics stack
* One session for deployment

## Window

Like a terminal tab.

## Pane

Split terminal inside a window.

---

# 2. Installation

## Ubuntu

```bash
sudo apt update
sudo apt install tmux
```

Check version:

```bash
tmux -V
```

---

# 3. Starting tmux

Start a session:

```bash
tmux
```

Start named session:

```bash
tmux new -s robotics
```

Attach to session:

```bash
tmux attach -t robotics
```

List sessions:

```bash
tmux ls
```

Kill session:

```bash
tmux kill-session -t robotics
```

---

# 4. The Prefix Key (MOST IMPORTANT)

tmux commands start with a **prefix**.

Default prefix:

```text
Ctrl+b
```

Meaning:

1. Press `Ctrl+b`
2. Release
3. Press another key

Example:

```text
Ctrl+b %
```

splits vertically.

---

# 5. Detaching & Reattaching (ESSENTIAL)

Detach from tmux:

```text
Ctrl+b d
```

The session keeps running in background.

Reconnect later:

```bash
tmux attach
```

or:

```bash
tmux attach -t robotics
```

This is the killer feature for SSH.

---

# 6. Windows

## Create Window

```text
Ctrl+b c
```

## Next Window

```text
Ctrl+b n
```

## Previous Window

```text
Ctrl+b p
```

## Go to Window Number

```text
Ctrl+b 0
Ctrl+b 1
Ctrl+b 2
...
```

## Rename Window

```text
Ctrl+b ,
```

## Close Window

Exit shell:

```bash
exit
```

or:

```text
Ctrl+b &
```

---

# 7. Panes (VERY IMPORTANT)

## Vertical Split

```text
Ctrl+b %
```

## Horizontal Split

```text
Ctrl+b "
```

## Move Between Panes

```text
Ctrl+b arrow-key
```

Example:

```text
Ctrl+b →
```

## Resize Panes

```text
Ctrl+b Ctrl+arrow-key
```

Example:

```text
Ctrl+b Ctrl+Right
```

## Close Pane

```bash
exit
```

or:

```text
Ctrl+b x
```

## Toggle Pane Zoom

Makes one pane fullscreen temporarily:

```text
Ctrl+b z
```

SUPER useful.

---

# 8. Copy Mode (CRITICAL)

tmux has its own scrollback buffer.

Enter copy mode:

```text
Ctrl+b [
```

Then:

* Arrow keys → move
* PageUp/PageDown
* `/` → search
* `n` → next result

Exit:

```text
q
```

---

# 9. Mouse Support (Highly Recommended)

Enable mouse support.

Create config:

```bash
nano ~/.tmux.conf
```

Add:

```tmux
set -g mouse on
```

Reload config:

```bash
tmux source-file ~/.tmux.conf
```

Now you can:

* Click panes
* Resize with mouse
* Scroll naturally

---

# 10. Essential tmux Workflow

Example robotics workflow:

## Window 1 — Build

```bash
colcon build --symlink-install
```

## Window 2 — ROS Core

```bash
ros2 launch ...
```

## Window 3 — Monitoring

```bash
htop
```

## Pane Split

One pane:

```bash
rviz2
```

Other pane:

```bash
ros2 topic echo
```

Detach safely during long builds or remote work.

---

# 11. Session Management (Power User)

## Create Detached Session

```bash
tmux new -d -s build
```

## Send Command Into Session

```bash
tmux send-keys -t build "colcon build" Enter
```

## Kill Server

Kills ALL sessions:

```bash
tmux kill-server
```

---

# 12. Renaming Things

Rename session:

```bash
tmux rename-session -t old new
```

Rename window:

```text
Ctrl+b ,
```

---

# 13. Synchronize Panes

Run same command in multiple panes.

Enable:

```text
Ctrl+b :
```

Then type:

```tmux
setw synchronize-panes on
```

Disable:

```tmux
setw synchronize-panes off
```

Useful for:

* Multiple robots
* Multiple servers
* Distributed systems

---

# 14. tmux Configuration

Config file:

```bash
~/.tmux.conf
```

---

# 15. Recommended Configuration

## Better Prefix

Many people prefer:

```tmux
set -g prefix C-a
unbind C-b
bind C-a send-prefix
```

Similar to GNU screen.

---

## Vim-style Pane Navigation

```tmux
bind h select-pane -L
bind j select-pane -D
bind k select-pane -U
bind l select-pane -R
```

Then:

```text
Ctrl+b h
Ctrl+b j
Ctrl+b k
Ctrl+b l
```

---

## Easier Reload

```tmux
bind r source-file ~/.tmux.conf \; display "Reloaded!"
```

Then:

```text
Ctrl+b r
```

---

## Better Colors

```tmux
set -g default-terminal "screen-256color"
```

---

## Start Window/Panes at 1

```tmux
set -g base-index 1
setw -g pane-base-index 1
```

---

## Enable Mouse

```tmux
set -g mouse on
```

---

# 16. Useful Plugins

Plugin manager:

Tmux Plugin Manager

GitHub:
[Tmux Plugin Manager (TPM)](https://github.com/tmux-plugins/tpm?utm_source=chatgpt.com)

Popular plugins:

* sensible defaults
* resurrect sessions
* continuum auto-save
* clipboard integration
* catppuccin themes

---

# 17. Session Persistence (Amazing)

Plugins:

* resurrect
* continuum

Can:

* restore panes
* restore windows
* restore commands
* auto-save sessions

Very useful for development workstations.

---

# 18. Clipboard Integration

## Copy to System Clipboard (Linux)

Install:

```bash
sudo apt install xclip
```

Add:

```tmux
bind-key -T copy-mode-vi y send-keys -X copy-pipe-and-cancel "xclip -selection clipboard -in"
```

---

# 19. Nested tmux Sessions

SSH into remote machine already using tmux?

Problem:

* Prefix conflicts

Temporary solution:
Press prefix twice:

```text
Ctrl+b Ctrl+b
```

---

# 20. Best Practices

## Use Named Sessions

Good:

```bash
tmux new -s moveit
```

Bad:

```bash
tmux
```

---

## One Project = One Session

Keeps workflows organized.

---

## Use Windows for Logical Groups

Example:

| Window  | Purpose     |
| ------- | ----------- |
| build   | compilation |
| launch  | ROS launch  |
| monitor | htop/logs   |
| git     | development |

---

## Use Panes Sparingly

Too many panes become unusable.

Recommended:

* 2–4 panes max

---

# 21. Advanced Features

## Session Sharing

Two users can attach same session:

```bash
tmux attach -t robotics
```

Great for debugging.

---

## Hooks & Automation

Auto-create workspace layouts.

Example startup script:

```bash
tmux new-session -d -s robotics
tmux rename-window build
tmux send-keys "cd ~/ws" Enter

tmux new-window -n launch
tmux send-keys "ros2 launch ..." Enter

tmux attach -t robotics
```

---

# 22. Common Problems

## Scroll Wheel Not Working

Enable mouse:

```tmux
set -g mouse on
```

---

## Colors Broken

Set:

```tmux
set -g default-terminal "screen-256color"
```

---

## Copy/Paste Weirdness

Use:

* modern terminal
* xclip/xsel
* mouse mode

---

# 23. tmux vs Terminator

You asked earlier about this.

## tmux

Best for:

* SSH
* remote development
* persistence
* keyboard workflows
* servers
* serious productivity

## Terminator

Best for:

* GUI pane splitting
* easier learning curve
* local desktop use

## Common setup

Many advanced users use BOTH:

```text
Terminator
  └── tmux inside
```

This is actually extremely common.

---

# 24. Real-World Robotics Workflow Example

Example Intrinsic/robotics workflow:

## Session: `manipulation`

### Window 1: build

```bash
colcon build
```

### Window 2: sim

```bash
ros2 launch ...
```

### Window 3: monitoring

```bash
htop
```

### Window 4: debugging

```bash
gdb
```

### Pane split:

* left → logs
* right → ros2 topic hz

Detach anytime:

```text
Ctrl+b d
```

Reconnect later.

---

# 25. Learning Path

## Beginner

Learn:

* sessions
* detach/attach
* panes
* windows

## Intermediate

Learn:

* copy mode
* configs
* mouse
* synchronization

## Advanced

Learn:

* plugins
* automation
* scripting
* session restoration

---

# Recommended Minimal ~/.tmux.conf

```tmux
set -g mouse on

set -g base-index 1
setw -g pane-base-index 1

set -g default-terminal "screen-256color"

bind r source-file ~/.tmux.conf \; display "Reloaded!"

bind h select-pane -L
bind j select-pane -D
bind k select-pane -U
bind l select-pane -R
```

---

# Final Recommendation

For your robotics/software workflow:

Best setup is likely:

```text
Terminal emulator:
  Terminator or Ghostty

Inside terminal:9
  tmux

Inside tmux:
  Neovim/VSCode terminal/ROS tools
```

tmux becomes truly valuable once:

* you SSH frequently
* you run long jobs
* you manage multiple processes
* you work on servers/robots
* you want persistent development environments
