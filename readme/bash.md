# Contents

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

	if [ -d .git ]; then

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
			echo -e "$color"
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