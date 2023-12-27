#!bin/bash

global_var=2

add_numbers () {
	local argc=$#
    local argv=$@
    local __fun__=$0
    local arg1=$1
    local arg2=$2
    # local arg3=$3

    echo "Called '"$__fun__"'"

    sleep 1

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
echo "62.4 == "$result