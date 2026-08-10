# Merge conficts

## Auto-merge from the CI

* **`--ours` (Current Changes):** The version of the file on the branch you are currently on (the one you are merging *into*).
* **`--theirs` (Incoming Changes):** The version of the file from the branch you are pulling or merging *in*.

Here are the commands to use depending on what you want to do:

### To Accept All Current Changes (`--ours`)

If you want to keep your version of the file and discard the incoming changes completely:

```bash
git checkout --ours path/to/conflicted-file.js
git add path/to/conflicted-file.js

```

### To Accept All Incoming Changes (`--theirs`)

If you want to overwrite your local file completely with the incoming changes:

```bash
git checkout --theirs path/to/conflicted-file.js
git add path/to/conflicted-file.js

```

---

> **⚠️ A Quick Warning on Rebasing:** If you are encountering these conflicts during a `git rebase` instead of a `git merge`, **the meanings of `ours` and `theirs` are swapped**. During a rebase, `--ours` refers to the upstream branch you are rebasing onto, and `--theirs` refers to your work that you are moving.

### What if you have dozens of files?

If you want to apply this rule to *every* conflicted file at once across your entire workspace, you can use a dot `.` instead of a specific file path:

```bash
# Accept all incoming changes across the whole project
git checkout --theirs .
git add .

```