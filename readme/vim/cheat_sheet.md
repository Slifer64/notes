# Vim Cheat Sheet

## Navigation

| Action                        | Command               |
| ----------------------------- | --------------------- |
| Move by word forward          | `w`                   |
| Move by word backward         | `b`                   |
| Start of line                 | `0`                   |
| End of line                   | `$`                   |
| Beginning of file             | `gg`                  |
| End of file                   | `G`                   |
| Go to line *N*                | `Ngg` or `:N`         |
| Jump to matching bracket      | `%`                   |
| ----------------------------- | --------------------- |
| Move left / down / up / right | `h` / `j` / `k` / `l` |
| Move to next word end         | `e`                   |
| Top of screen                 | `H`                   |
| Middle of screen              | `M`                   |
| Bottom of screen              | `L`                   |
| Move half-page down/up        | `Ctrl-d` / `Ctrl-u`   |
| Move full page down/up        | `Ctrl-f` / `Ctrl-b`   |

---

## Insert Mode

| Action                  | Command |
| ----------------------- | ------- |
| Insert before cursor    | `i`     |
| Exit insert mode        | `Esc`   |
| ----------------------- | ------- |
| Insert at start of line | `I`     |
| Insert after cursor     | `a`     |
| Insert at end of line   | `A`     |
| Open line below         | `o`     |
| Open line above         | `O`     |
| Replace one character   | `r`     |
| Replace mode            | `R`     |

---

## Editing

| Action                    | Command |
| ------------------------- | ------- |
| Delete character          | `x`     |
| Select and delete text    | `v`+ `→` + `x` |
| Delete previous character | `X`     |
| Delete word               | `dw`    |
| Delete to end of line     | `D`     |
| Delete line               | `dd`    |
| Delete N lines            | `Ndd`   |
| ------------------------- | ------- |
| Change word               | `cw`    |
| Change line               | `cc`    |
| Change to end of line     | `C`     |
| Join lines                | `J`     |
| Indent line               | `>>`    |
| Unindent line             | `<<`    |
| Auto-indent selected text | `=`     |

**Note**: All delete actions work as "cut". You can then press `p` to paste it.

---

## Copy / Paste

| Action                       | Command |
| ---------------------------- | ------- |
| Copy (yank) word             | `yw`    |
| Copy line                    | `yy`    |
| Copy N lines                 | `Nyy`   |
| Copy to end of line          | `y$`    |
| Paste after cursor           | `p`     |
| ---------------------------- | ------- |
| Paste before cursor          | `P`     |
| Delete and store in register | `d`     |
| Copy entire file             | `ggVGy` |

**Note**: To copy and paste outside of vim:
- `v`, then  `→...` to select the text, then `"+y` (`"` tells Vim "I am about to specify a register."), `+` specifies the system clipboard register and `y` copies it. However if you run vim inside tmux this won't work. In this case do the following
- hold down shift (Bypass Vim & tmux) and move the cursor to select the text, and then `ctrl+shift+c` and paste outside of terminal

---

## Undo / Redo

| Action                  | Command  |
| ----------------------- | -------- |
| Undo                    | `u`      |
| Redo                    | `Ctrl-r` |
| Repeat previous command | `.`      |

---

## Search & Replace

| Action                           | Command          |
| -------------------------------- | ---------------- |
| Search forward                   | `/pattern`       |
| Search backward                  | `?pattern`       |
| Next match                       | `n`              |
| Previous match                   | `N`              |
| Search current word              | `*`              |
| Search current word backward     | `#`              |
| Replace first occurrence on line | `:s/old/new/`    |
| Replace all occurrences on line  | `:s/old/new/g`   |
| Replace in entire file           | `:%s/old/new/g`  |
| Replace with confirmation        | `:%s/old/new/gc` |

---

## Visual Mode

| Action              | Command  |
| ------------------- | -------- |
| Character selection | `v`      |
| Line selection      | `V`      |
| Block selection     | `Ctrl-v` |
| Indent selection    | `>`      |
| Unindent selection  | `<`      |
| Copy selection      | `y`      |
| Delete selection    | `d`      |
| Change selection    | `c`      |

---

## File Operations

| Action                   | Command       |
| ------------------------ | ------------- |
| Save file                | `:w`          |
| Save as                  | `:w filename` |
| Quit                     | `:q`          |
| Save and quit            | `:wq`         |
| Quit without saving      | `:q!`         |
| Save and quit (shortcut) | `ZZ`          |
| Reload file from disk    | `:e!`         |

---

## Multiple Files & Buffers

| Action                | Command   |
| --------------------- | --------- |
| Open file             | `:e file` |
| Next buffer           | `:bn`     |
| Previous buffer       | `:bp`     |
| List buffers          | `:ls`     |
| Delete current buffer | `:bd`     |
| Switch to buffer N    | `:b N`    |

---

## Windows & Splits

| Action                  | Command             |
| ----------------------- | ------------------- |
| Horizontal split        | `:split` or `:sp`   |
| Vertical split          | `:vsplit` or `:vsp` |
| Move to next split      | `Ctrl-w w`          |
| Move left/right/up/down | `Ctrl-w h/j/k/l`    |
| Close split             | `:q`                |
| Equalize split sizes    | `Ctrl-w =`          |
| Maximize current split  | `Ctrl-w _`          |

---

## Useful Text Objects

| Action                    | Command |
| ------------------------- | ------- |
| Delete inside parentheses | `di(`   |
| Delete inside quotes      | `di"`   |
| Change inside parentheses | `ci(`   |
| Change inside quotes      | `ci"`   |
| Delete around word        | `daw`   |
| Change around word        | `caw`   |
| Select inside brackets    | `vi[`   |
| Select inside paragraph   | `vip`   |

---

## Marks & Jumps

| Action                 | Command  |
| ---------------------- | -------- |
| Set mark `a`           | `ma`     |
| Jump to mark `a`       | `'a`     |
| Jump to exact position | `` `a `` |
| Jump back              | `Ctrl-o` |
| Jump forward           | `Ctrl-i` |

---

## Registers

| Action                   | Command |
| ------------------------ | ------- |
| Show registers           | `:reg`  |
| Paste register `a`       | `"ap`   |
| Yank into register `a`   | `"ayy`  |
| Delete into register `a` | `"add`  |
| Paste system clipboard   | `"+p`   |
| Copy to system clipboard | `"+y`   |

---

## Macros

| Action                    | Command |
| ------------------------- | ------- |
| Start recording macro `a` | `qa`    |
| Stop recording            | `q`     |
| Execute macro `a`         | `@a`    |
| Execute macro N times     | `N@a`   |
| Repeat last macro         | `@@`    |

---

## Command Multipliers

Most Vim commands can be prefixed with a number:

| Example | Meaning              |
| ------- | -------------------- |
| `5j`    | Move down 5 lines    |
| `10dd`  | Delete 10 lines      |
| `3w`    | Move forward 3 words |
| `4p`    | Paste 4 times        |
| `20G`   | Go to line 20        |

---

## Power User Commands

| Action                        | Command               |
| ----------------------------- | --------------------- |
| Show line numbers             | `:set number`         |
| Relative line numbers         | `:set relativenumber` |
| Toggle paste mode             | `:set paste`          |
| Show hidden characters        | `:set list`           |
| Open file explorer            | `:Ex`                 |
| Run shell command             | `:!command`           |
| Read shell output into buffer | `:r !command`         |
| Format paragraph              | `gqap`                |
| Re-indent entire file         | `gg=G`                |
| Sort selected lines           | `:'<,'>sort`          |

### Vim Mental Model

Most editing commands follow:

```text
[count] [operator] [motion]
```

Examples:

```vim
dw      " delete word
d$      " delete to end of line
caw     " change around word
yip     " yank inner paragraph
3dd     " delete 3 lines
5yw     " yank 5 words
```

