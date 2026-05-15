# tmux Cheat Sheet

## Hierarchy

```text
tmux server
 └── sessions
      └── windows
           └── panes
```

## Sessions

A workspace/project.

| Action        | Command                     | Notes                     |
| ------------- | --------------------------- | ------------------------- |
| New session   | `tmux new -s name`          |           |
| New detached session   | `tmux new -d -s name`          |           |
| List sessions | `tmux ls`                   |           |
| Rename        | `tmux rename-session -t old new` | or `Ctrl+b $` (from inside a session) |           |
| Attach        | `tmux attach -t name`       |           |
| Detach        | `Ctrl+b d`                  | session keeps running in background (reconnect with `attach`) |
| Kill session  | `tmux kill-session -t name` |           |
| Switch session  | `tmux switch -t name` |           |
| send commands in sessions | `tmux send-keys -t name "colcon build" Enter` |  |
| kill server | `tmux kill-server` |  |

---

## Windows

Like a terminal tab.

| Action          | Shortcut     | Notes                     |
| --------------- | ------------ | ------------------------- |
| New window      | `Ctrl+b c`   |           |
| Next window     | `Ctrl+b n`   |           |
| Previous window | `Ctrl+b p`   |           |
| Window number   | `Ctrl+b 0-9` |           |
| Rename          | `Ctrl+b ,`   |           |
| Kill            | `Ctrl+b &`   | or type `exit` in terminal |

---

## Panes

Split terminal inside a window.

| Action           | Shortcut             | Notes                     |
| ---------------- | -------------------- | ------------------------- |
| Vertical split   | `Ctrl+b %`           |           |
| Horizontal split | `Ctrl+b "`           |           |
| Move pane        | `Ctrl+b arrows`      |           |
| Resize pane      | `Ctrl+b Ctrl+arrows` |           |
| Move to pane     | `Ctrl+b q <pane number>` |           |
| Close pane       | `Ctrl+b x`           | or type `exit` in terminal |
| Zoom pane        | `Ctrl+b z`           |           |
| Change to default layout | `Ctrl+b Alt<1-5>` |           |


---

## Copy Mode

| Action | Shortcut   |
| ------ | ---------- |
| Enter  | `Ctrl+b [` |
| Search | `/`        |
| Exit   | `q`        |

---

## Misc

| Action        | Command                         | Notes |
| ------------- | ------------------------------- | ----- |
| new session and window | `tmux new -s mysession -n mywindow` |
| View/Switch to any session/window/pane | `Ctrl+b w` |  use →/← to unfold/fold session/windows etc. |
| Reload config | `tmux source-file ~/.tmux.conf` |
| Kill all session   | `tmux kill-server`              |

---

## Link

https://tmuxcheatsheet.com/
