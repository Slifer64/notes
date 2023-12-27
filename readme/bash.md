# Contents
- [Bash script](#bash-script)
- [Find/Kill process by name](#findkill-a-process-by-name)
- [Print colored text](#print-colored-text)
- [Show git branch and status](#show-git-branch-and-status)


# Bash script

## Arithmentic operator

### Integer operations
```bash
x=10; y=20
echo $(( $x + $y ))  # addition
echo $(( $x - $y ))  # subtraction
echo $(( $x * $y ))  # multiplication 
echo $(( $x / $y ))  # division
echo $(( $x ** $y ))  # exponentiation
echo $(( $x % $y ))  # modulo
# directly assign the result to `$x`
(( x += 10 ))    # increament
(( x -= 15 ))  # decreament
(( x *= 2 ))  # multiply
(( x /= 5 ))  # divide
(( x %= 5 ))  # modulo
```

### Floating operations
Use `$(echo "<operations>" | bc)`
```bash
a=5.2; b=7.3
result=$(echo "$a * (($b - 1.3)/2)" | bc)
echo "15.6 == "$result
```


## `if-else`

https://linuxize.com/post/bash-if-else-statement/

```bash
val=5
if [[ $val -eq 1 ]]; then
  echo "val = 1"
elif [[ "$val" -gt 3 ]] && [[ $val -ne 4 ]]; then
  echo "val = 5"
else
  exit 1
fi
```

### Common operators:

| operator | description |
|----------|:-------------|
|-n VAR | True if the length of VAR is greater than zero.
| -z VAR | True if the VAR is empty. |
| STRING1 = STRING2 | True if STRING1 and STRING2 are equal. |
| STRING1 != STRING2 | True if STRING1 and STRING2 are not equal. |
| INTEGER1 -eq INTEGER2 | True if INTEGER1 and INTEGER2 are equal. |
| INTEGER1 -ne INTEGER2 | True if INTEGER1 and INTEGER2 are **not** equal. |
| INTEGER1 -gt INTEGER2 | True if INTEGER1 is greater than INTEGER2. |
| INTEGER1 -lt INTEGER2 | True if INTEGER1 is less than INTEGER2. |
| INTEGER1 -ge INTEGER2 | True if INTEGER1 is equal or greater than INTEGER2. |
| INTEGER1 -le INTEGER2 | True if INTEGER1 is equal or less than INTEGER2. |
| -h FILE | True if the FILE exists and is a symbolic link. |
| -r FILE | True if the FILE exists and is readable. |
| -w FILE | True if the FILE exists and is writable. |
| -x FILE | True if the FILE exists and is executable. |
| -d FILE | True if the FILE exists and is a directory. |
| -e FILE | True if the FILE exists and is a file, regardless of type (node, directory, socket, etc.). |
| -f FILE | True if the FILE exists and is a regular file (not a directory or device). |

Example:
```bash
if [[ -d "my_dir" ]]; then
	echo "It is a directory"
fi
```

## arrays
```bash
# declare -a fruits=("Apple" "Banana" "Orange" "Grape" "Melon")
fruits=("Apple" "Banana" "Orange" "Grape" "Melon")

# array length
length=${#fruits[@]}

# element access
echo ${fruits[2]}
i=4
echo ${fruits[$i]}

# append elements
fruits+=("Mango" "Peach")

# convert multiple function return values to an array
array_result=($(my_fun))
```

## `for loop`
https://linuxize.com/post/bash-for-loop/

https://linuxize.com/post/bash-while-loop/

```bash
for i in {0..3}; do
  echo -n "$i " # 0 1 2 3
done

for i in {0..20..5}; do
  echo -n "$i " # 0 5 10 15 20
done

for element in Apple Banana Orange; do
  echo "$element"
done

for f in "${fruits[@]}"; do
  echo -n "$f "
done

for ((i=0; i<${#fruits[@]}; i++)); do
    echo "${fruits[$i]} --> ${B[$i]}"
done

for ...; do
	if [[ ... ]]; then
		break
		# continue
	fi
done
```

Extra:
```bash
for file in *\ *; do
  mv "$file" "${file// /_}"
done

for file in *.jpeg; do
    mv -- "$file" "${file%.jpeg}.jpg"
done
```

## Functions

https://linuxize.com/post/bash-functions/

### Pass arguments and retun single value

```bash
global_var=2

add_numbers() 
{
	local argc=$#
    local argv=$@
    local arg1=$1
    local arg2=$2
    # local arg3=$3

	(( global_var += $argc ))

	result=$(echo "($arg1 + $arg2)*$global_var" | bc)
	echo $result
}

result=$(add_numbers 5.2 7.3)
exit_code=$?

if [[ ! $exit_code -eq 0 ]]; then
	echo -e "\033[1;31mExecution failed...\033[0m"
    return 1
fi

echo "exit code: "$exit_code
echo "50.0 == "$result
```

- The passed parameters are `$1`, `$2`, `$3` … `$n`, corresponding to the position of the parameter after the function’s name.
- The `$0` variable is reserved for the function’s name.
- The `$#` variable holds the number of positional parameters/arguments passed to the function.
- The `$*` and `$@` variables hold all positional parameters/arguments passed to the function.
	- When double-quoted, `"$*"` expands to a single string separated by space (the first character of IFS) - `"$1 $2 $n"`.
	- When double-quoted, `"$@"` expands to separate strings - `"$1" "$2" "$n"`.
	- When not double-quoted, `$*` and `$@` are the same.

### Return multiple values
```bash
my_fun()
{
	# ...
	a="Hello"
	b=6
	c="yoo"
	echo "$a $b $c"
}

rt=($(my_fun $arg1 $arg2 ...))
a=${rt[0]}
b=${rt[1]}
c=${rt[2]}
```

# Search \& replace folder name/pattern

```bash
# Search recursively in all folder and subfolders and replace 'outputs' with 'mp_weights'
find ./ -type f -exec rename 's/outputs/mp_weights/' '{}' \;
```

# Find/Kill a process by name

## Find
```bash
ps -aux | grep 'ros2 launch my_process.launch.py'
# simpler:
pgrep -f 'ros2 launch my_process.launch.py'
```

## Kill
```bash
ps -aux | grep 'ros2 launch my_process.launch.py' | awk {'print $2;'} | xargs kill -INT
pgrep -f 'ros2 launch my_process.launch.py' | xargs kill -INT
# simpler:
pkill -SIGINT -f 'ros2 launch my_process.launch.py'
```
This will sent a `ctrl+C` (`SIGINT`) signal to the process `ros2 launch my_process.launch.py`.

## Kill all
```bash
killall code
```

# Print colored text

## Color codes
<div class="table-wrapper-paragraph">
<table>
<thead>
    <tr><th>Color</th><th>Foreground Code</th><th>Background Code</th></tr>
</thead>
<tbody>
    <tr>    <td>Black</td>          <td>30</td> <td>40</td> </tr>
    <tr>    <td>Red</td>            <td>31</td> <td>41</td> </tr>
    <tr>    <td>Green</td>          <td>32</td> <td>42</td> </tr>
    <tr>    <td>Yellow</td>         <td>33</td> <td>43</td> </tr>
    <tr>    <td>Blue</td>           <td>34</td> <td>44</td> </tr>
    <tr>    <td>Magenta</td>        <td>35</td> <td>45</td> </tr>
    <tr>    <td>Cyan</td>           <td>36</td> <td>46</td> </tr>
    <tr>    <td>Light Gray</td>     <td>37</td> <td>47</td> </tr>
    <tr>    <td>Gray</td>           <td>90</td> <td>100</td> </tr>
    <tr>    <td>Light Red</td>      <td>91</td> <td>101</td> </tr>
    <tr>    <td>Light Green</td>    <td>92</td> <td>102</td> </tr>
    <tr>    <td>Light Yellow</td>   <td>93</td> <td>103</td> </tr>
    <tr>    <td>Light Blue</td>     <td>94</td> <td>104</td> </tr>
    <tr>    <td>Light Magenta</td>  <td>95</td> <td>105</td> </tr>
    <tr>    <td>Light Cyan</td>     <td>96</td> <td>106</td> </tr>
    <tr>    <td>White</td>          <td>97</td> <td>107</td> </tr>
</tbody>
</table>
</div>

## Text formats
<div class="table-wrapper-paragraph"><table>
<thead>
    <tr><th>Code</th><th>Description</th></tr>
</thead>
<tbody>
    <tr>    <td>0</td>  <td>Reset/Normal</td>       </tr>
    <tr>    <td>1</td>  <td>Bold text</td>          </tr>
    <tr>    <td>2</td>  <td>Faint text</td>         </tr>
    <tr>    <td>3</td>  <td>Italics</td>            </tr>
    <tr>    <td>4</td>  <td>Underlined text</td>    </tr>
</tbody>
</table></div>


### Usage
Gneric usage:
```bash
echo -e "\e[<text_format>;<text_format>;<foreground_color>;<background_color>m""Hello world!""\e[0m"
```
Use of `\e[` is equivalent to `\033[`. \
Example:
```bash
echo -e "\e[1;3;4;33;45mBold, italics, underlined yellow text in purple background\e[0m"
```

# Show git branch and status
Add the following to `~/.bashrc`:
```bash
git_branch_and_status() {

	if [ -d .git ] || [ -f .git ] || git rev-parse --is-inside-work-tree >/dev/null 2>&1; 
	then

		repo_name=$(git config --get remote.origin.url | sed 's/.*\/\([^\/]*\)\.git$/\1/')
		if [ -f .git ]; then repo_name="module:$repo_name"; fi

		GIT_BRANCH=$(git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/\1/')
		GIT_STATUS=$(git status 2> /dev/null)

		untracked=""
		not_staged=""
		staged=""
		color="\e[0m" # RESET

		if echo "$GIT_STATUS" | grep -q "Untracked files"; then
			untracked="-"
			color="\e[1;33m" # YELLOW
		fi
		if echo "$GIT_STATUS" | grep -q "Changes not staged for commit"; then
			not_staged="*"
			color="\e[1;31m" # RED
		fi
		if echo "$GIT_STATUS" | grep -q "Changes to be committed"; then
			staged="+"
			color="\e[1;32m" # GREEN
		else
			staged=""
		fi
		
		if echo "$GIT_STATUS" | grep -q "working directory clean"; then
			color="\e[0m" # RESET
		elif echo "$GIT_STATUS" | grep -q "Your branch is ahead of"; then
			color="\e[1;34m" # BLUE
		elif echo "$GIT_STATUS" | grep -q "Your branch is behind"; then
			color="\e[1;36m" # CYAN
		elif echo "$GIT_STATUS" | grep -q "HEAD detached"; then
			color="\e[1;35m" # PURPPLE
		fi
		
		if [ -n "$GIT_BRANCH" ]; then
			# echo -e "\[$color\]("$GIT_BRANCH"$untracked$not_staged$staged)"
			echo -e " \e[1;2;37m($repo_name)\e[0m$color"
			echo "("$GIT_BRANCH"$untracked$not_staged$staged)"
		fi
	
	fi
}
export PS1="\[\e[2;32m\]\w\[\e[0m\]\$(git_branch_and_status)\[\e[0m\]$ "
``` 
To output the username: `PS1="\u ..."`. \
To output the computer name: `PS1="\h ..."`. \
All together: `PS1="\u@\h \[\033[32m\]\w$(git_branch_and_status)\[\033[00m\]$ "`

`\w`: Current working directory. \
`\u`: Username. \
`\h`: Hostname. \
<!-- export PS1="{\[\e[32m\]\u\[\e[m\]@\[\e[36m\]\h\[\e[m\]:\W_\$?}$ " -->