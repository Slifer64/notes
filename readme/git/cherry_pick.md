# Git Cherry-Pick & Conflict Resolution Cheatsheet

### 1. Basic Cherry-Picking

| Action | Command |
| :--- | :--- |
| **Apply latest commit of a branch** | `git cherry-pick <branch-name>` |
| **Apply specific commit by SHA** | `git cherry-pick <commit-hash>` |
| **Apply without opening editor** *(keeps message)* | `git cherry-pick --no-edit <ref>` |
| **Apply changes without committing** *(staged)* | `git cherry-pick -n <ref>` / `--no-commit` |

---

### 2. Multiple Commits & Ranges

| Action | Command |
| :--- | :--- |
| **Pick multiple distinct commits** | `git cherry-pick <hash-1> <hash-2> <hash-3>` |
| **Pick range (exclusive of A, inclusive of B)** | `git cherry-pick <hash-A>..<hash-B>` |
| **Pick range (inclusive of A and B)** | `git cherry-pick <hash-A>^..<hash-B>` |

---

### 3. Conflict Resolution Strategies

| Strategy | Command / Action |
| :--- | :--- |
| **Keep Current (`HEAD` / Ours)** | `git checkout --ours <file>` *(or `git restore --ours <file>`)* |
| **Keep Incoming (Cherry-pick / Theirs)** | `git checkout --theirs <file>` *(or `git restore --theirs <file>`)* |
| **Keep Both** | Edit `<file>` to merge both sections, then delete `<<<<<<<`, `=======`, `>>>>>>>` |

---

### 4. Lifecycle & Flow Control

| Action | Command |
| :--- | :--- |
| **Stage resolved files** | `git add <file>` |
| **Finish cherry-pick after conflicts** | `git cherry-pick --continue` |
| **Abort cherry-pick & revert to original state** | `git cherry-pick --abort` |
| **Skip current commit & move to next** | `git cherry-pick --skip` |
