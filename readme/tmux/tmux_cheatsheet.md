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
| Next Match | `n`    |
| Prev Match | `N`    |

To search in the tmux history buffer for the current window, press `Ctrl-b [` to enter copy mode.

**Regex Search (POSIX ERE):**
Tmux search is regex-enabled by default. Note that `\d` and `\s` are **not** supported.
- Use `[0-9]+` instead of `\d+`
- Use `[[:space:]]` instead of `\s`
- Example: `[a-z]{3}-[0-9]{4}`

If you're using emacs key bindings (the default), press `Ctrl-s` then type the string to search for and press `Enter`. Press `n` to search for the same string again. Press `Shift-n` for reverse search. Press `Escape` twice to exit copy mode. You can use `Ctrl-r` to search in the reverse direction.

**find-window**:
If you want to switch to a window based on something displayed in it (this also includes window names and titles but not history), (starting with more than one window open) press `Ctrl-b`+`f` then type the string to search for and press `Enter`. You will be switched to a window containing that text if it's found. If more than one window matches, you'll see a list to select from.

---

## Misc

| Action        | Command                         | Notes |
| ------------- | ------------------------------- | ----- |
| new session and window | `tmux new -s mysession -n mywindow` |
| View/Switch to any session/window/pane | `Ctrl+b w` |  use →/← to unfold/fold session/windows etc. |
| Reload config | `tmux source-file ~/.tmux.conf` |
| Kill all session   | `tmux kill-server`              |

---

# Recommended Minimal ~/.tmux.conf

```tmux
set -g mouse on

set -g base-index 1
setw -g pane-base-index 1

set -g default-terminal "screen-256color"

bind r source-file ~/.tmux.conf \; display "Config reloaded!"

bind h select-pane -L
bind j select-pane -D
bind k select-pane -U
bind l select-pane -R

# Use Prefix + / to enter copy mode and search forward
bind-key / copy-mode \; command-prompt -T search -p "Search forward (regex):" { send-keys -X search-forward -- "%%" }

# Use Prefix + ? to enter copy mode and search backward
bind-key ? copy-mode \; command-prompt -T search -p "Search backward (regex):" { send-keys -X search-backward -- "%%" }
```

`tmux source-file ~/.tmux.conf`

---

## Link

https://tmuxcheatsheet.com/
