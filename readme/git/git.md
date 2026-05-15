# Git

---

## List branches

```bash
git branch # local
git branch -r # remote
git branch -a # local + remote
```

---

## Delete branch

```bash
git checkout <branch_other_than_the_one_to_delete>
git branch -d feature/my-cool-feature
git push origin --delete feature/my-cool-feature

# After deleting the remote branch, other local clones may still show it until they prune remote refs:
git fetch --prune
```

---


## Create a new branch and set tracking origin

```bash
git checkout -b feature/my-cool-feature
git push origin feature/my-cool-feature
git branch --set-upstream-to=origin/feature/my-cool-feature

# or simpler
git checkout -b feature/my-cool-feature && git push -u origin HEAD
# the '-u' flag is short for '--set-upstream'
```

---

## Add a new remote

```bash
# add a new remote named 'upstream'
git remote add upstream https://github.com/moveit/moveit2.git
git fetch upstream

# list remotes
git remote -v
# example output: remotes/origin, remotes/upstream
```

---

## Overwrite branch history

```bash
git checkout <branch_to_overwrite>
git reset --hard <branch_whose_history_to_copy_in_this_branch>
```

---

## Tranfer commit history from on repo to another

Transfer: `formulation-evaluators-mcp` branch `main` --> `git@github.com:PIPACorp/ds-poc.git` branch `formulation-evaluators-mcp`

#### Step 1 — Clone the target repo

```bash
git clone git@github.com:PIPACorp/ds-poc.git
cd ds-poc
```

### Step 2 — Add the source repo as a remote

```bash
git remote add recipe git@github.com:antosidi/recipe_evaluator_mcp_server.git
git fetch recipe
```

### Step 3 — Create the target branch from the source history

```bash
git checkout -b formulation-evaluators-mcp recipe/main
git remote remove recipe
```

This creates a new branch in ds-poc that:
- Points to the exact commit history of recipe/main
- Preserves the entire history
- Does NOT modify other branches like main

### Step 4 — Push the branch

```bash
git push origin formulation-evaluators-mcp
```

Done.

Now:
- `PIPACorp/ds-poc`
- Branch: `formulation-evaluators-mcp`
- Has the full history from `recipe_evaluator_mcp_server/main`

### ⚠️ If Branch Already Exists

If `formulation-evaluators-mcp` already exists and you want to overwrite it:

```bash
git push origin formulation-evaluators-mcp --force
```

⚠️ Only do this if you're sure no one else depends on that branch.


### 🧠 Verification

After pushing:
```bash
git log --oneline --graph
```
You should see the full history from recipe_evaluator_mcp_server.

---

## Commit Message Standard

The widely adopted standard is Conventional Commits (https://www.conventionalcommits.org/):

```
<type>(<scope>): <imperative-verb> <what>

[optional body]
```

- Types: `fix`, `feat`, `refactor`, `docs`, `style`, `test`, `perf`, `chore`
- Scope: the affected module/package (e.g., `robot_state`, `kdl_kinematics`)
- Description: imperative mood, lowercase, no period, ≤72 chars
- Why imperative? It reads as "this commit adds X", not "added X"

---
