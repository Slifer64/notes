I have forked the "moveit/moveit2" git repo into my repo "Slifer64/moveit2". I want to keep it synced with the "moveit/moveit2" while also making my own changes (commits) on top. What is the best way to do this? E.g. having a "moveit" branch that is always synced with "moveit/moveit2" and rebase the "main" branch periodically? However, this means I will have to force-push in main, so if someones tracks my main branch, they will have to git reset hard 
Or is it better having a "moveit" branch synced with the original, having a "dev" branch rebased periodically, and a "main" branch that is periodically merged with the dev branch?
Or something else? 

---

For a long-lived fork like MoveIt / [moveit2 GitHub repo](https://github.com/moveit/moveit2?utm_source=chatgpt.com), the cleanest setup is usually:

* keep one branch that mirrors upstream exactly
* do all your actual work on separate branches
* avoid rebasing any branch that others may consume
* only rewrite history on private/local integration branches

A very common and robust structure is:

```text
upstream/main     -> moveit/moveit2 main
main              -> your stable public branch
dev               -> your integration branch
feature/*         -> actual work branches
```

with:

* `main` updated via merges from `dev`
* `dev` periodically rebased (or merged) onto upstream
* `main` NEVER rebased after publishing

This gives:

* clean development history
* no force-push requirement for users tracking `main`
* easy syncing with upstream
* low risk of breaking collaborators

---

# Recommended setup

## 1. Add upstream remote

```bash
git remote add upstream https://github.com/moveit/moveit2.git
git fetch upstream
```

Now you have:

* `origin` → your fork
* `upstream` → original repo

---

# Option A (Recommended for public/shared fork)

This is usually the best balance.

## Branch roles

### `upstream-main` (optional local tracking branch)

Pure mirror of upstream.

```bash
git checkout -b upstream-main upstream/main
```

You usually never commit here.

Update it with:

```bash
git fetch upstream
git checkout upstream-main
git reset --hard upstream/main
```

You may not even need this branch — `upstream/main` remote-tracking branch already exists.

---

### `dev`

Your integration branch.

This is where you:

* periodically sync upstream
* resolve conflicts
* test compatibility

You CAN rebase this branch if only you use it.

Example:

```bash
git checkout dev
git fetch upstream
git rebase upstream/main
```

or alternatively:

```bash
git merge upstream/main
```

---

### `main`

Stable/public branch.

This is the important part:

* NEVER rebase after publishing
* NEVER force-push
* update via merges only

Example:

```bash
git checkout main
git merge dev
git push origin main
```

People depending on your fork can safely track `main`.

---

# Why this is better than rebasing main

If you periodically:

```bash
git rebase upstream/main
git push --force
```

on `main`, then:

* every downstream user gets history rewritten
* every PR becomes annoying
* anyone tracking your fork must hard-reset/rebase
* CI history changes
* SHA references become invalid

This becomes painful quickly on a public/shared repo.

---

# Typical workflow

## Sync upstream

```bash
git fetch upstream
```

---

## Rebase integration branch

```bash
git checkout dev
git rebase upstream/main
```

Resolve conflicts once here.

---

## Test

Build/test MoveIt2.

---

## Merge into stable main

```bash
git checkout main
git merge dev
git push origin main
```

No history rewriting for users.

---

# Alternative: merge-only workflow

If you want maximum stability and zero rebasing:

```text
upstream/main -> merge into dev -> merge into main
```

Example:

```bash
git checkout dev
git merge upstream/main
```

Advantages:

* zero history rewriting
* easier collaboration
* safer

Disadvantages:

* noisier git history
* many merge commits

For large robotics repos, this is actually pretty common.

---

# Alternative: rebase-only private fork

If:

* you are the ONLY user
* nobody tracks your branches
* you want ultra-clean history

then you can simply:

```text
main = rebased on upstream/main forever
```

with force pushes.

This is okay for:

* personal experimentation
* short-lived forks

Not ideal for a reusable public fork.

---

# My recommendation for your case

Given your robotics/software background and the fact this may become a reusable/customized fork of MoveIt:

## Best structure

```text
upstream/main   (remote tracking)
dev             (rebased periodically)
main            (stable merge-only branch)
feature/*       (work branches)
```

## Rules

### `dev`

* may be rebased
* may be force-pushed
* integration branch only

### `main`

* never rewritten
* merge-only
* safe for users

This is essentially how many large downstream forks operate.

---

# Extra tip: keep your custom work isolated

Instead of modifying upstream files everywhere, try to:

* keep custom packages separate
* isolate patches
* keep commits logically separated

Then rebases become MUCH easier.

Especially in large ROS2/MoveIt2 ecosystems.

---

# Nice additional pattern

You can also maintain:

```text
patches/*
```

or:

```text
feature/custom-planner
feature/custom-servo
feature/custom-control
```

then periodically:

```bash
git rebase upstream/main
```

onto each.

This makes upstream sync much less painful than one giant monolithic branch.
