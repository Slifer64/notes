# Git

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