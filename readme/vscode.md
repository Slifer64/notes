
# Reference config, env and other variables in `*.json` config files

Variables:
- **Predifined**: `${userHome}`, `${workspaceFolder}` etc.
- **Environment**: `${env:USERNAME}` etc.
- **Vscode Config**: e.g. `"my_var": ${config:editor.fontSize}`
- **Command**: Execute an extension command and return a string result that is stored in the variable, e.g. `"processId": "${command:extension.pickNodeProcess}"`
- **Input**: e.g. `${input:variableID}`, `variableID` refers to entries in the inputs section of `launch.json` and `tasks.json`, where additional configuration attributes are specified.

For details see https://code.visualstudio.com/docs/editor/variables-reference

# Update Intellisence C++/Python search paths

`Ctrl+Shift+P`, type `update` and look for `Update C++ properties` and `Update Python Path`.

# Autocomplete in python virtual environment

Press `Ctrl+Shift+P` and in the command panel type `Python: Select interpreter` and paste the absolute path to your `.env/bin/` folder that was created by python venv.

# Set additional search paths for python

If it doesn't exist already, from the root of the VScode editor, create `.vscode/settings.json` and insert:
```json
{
    "python.autoComplete.extraPaths": [
        // "/absolute_path_to_your_package",
    ],
    "python.analysis.extraPaths": [
        // "/absolute_path_to_your_package",
    ]
}
``` 
