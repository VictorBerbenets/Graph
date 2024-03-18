#!/bin/bash

#color atributes
red="\033[1;31m"
green="\033[1;32m"
blue="\033[1;36m"
purple="\033[1;35m"
white="\033[1;37m"
usual="\033[0m"
#directories paths and exe file for test generating
resource_dir="../tests/end2end/resources/"
tests_dir="../tests/end2end/resources/tests/"
answs_dir="../tests/end2end/resources/answers/"
exe_test_file="./tests/end2end"
exe_main_file_1="./graph"
checker="./tests/checker"

##------------------------------------------------------------------------------------------##
function run_tests {
    touch compare_file.txt
    npassed_count=0
    echo -e "${white}---end2end testing---${usual}"
    for ((i = 1; i <= ${tests_number}; ++i))
    do
      tmp_check="${tests_dir}test${i}.txt"
      ans_file="${answs_dir}ans${i}.txt"

      echo -n -e "${purple}Test ${i}: ${usual}\n"
      ${checker} ${tmp_check} ${ans_file} > compare_file.txt
      if [ "$(cat compare_file.txt)" == "passed" ]
      then
          echo -e "${green}passed${usual}"
      else
          echo -e "${red}failed${usual}"
          npassed_count=$((${npassed_count} + 1))
      fi
      echo -en "${blue}painted graph:${usual}"
      printf "\n"
      cat ${ans_file}
      printf "\n"
    done
    echo -e "${white}-------------------------------------Total----------------------------------${usual}"
    if [ ${npassed_count} -eq 0 ]
    then
      echo -e "${blue} \t\t\t\tAll tests passed! ${usual}"
    elif [ ${npassed_count} -eq 1 ]
    then
      echo -e "${red} \t\t\t\t ${npassed_count} test failed( ${usual}"
    elif [ ${npassed_count} -gt 1 ]
    then
      echo -e "${red} \t\t\t\t ${npassed_count} tests failed( ${usual}"
    fi

    rm compare_file.txt
}
##------------------------------------------------------------------------------------------##

#main part

error="false"
tests_number=0

if [ $# -eq 3 ]
then
  tests_number=$1
  tests_dir=$2
  answs_dir=$3;
elif [ $# -ne 1 ]
then
    error="true"
    echo -e "${red}invalid number of arguments: expected 3 or 1, got $#"
else
  tests_number=$1
fi

if [ $error = "false" ]
then
    run_tests
fi
