# Jilin Zheng // U49258796

## EC440: Shell

Files relevant to the shell:

- [Makefile](Makefile): This Makefile includes contents related to the shell parser assignment, but only builds the shell for purposes of the shell assignment
- [myshell.c](myshell.c): The implementation of the shell, supporting both a prompted and unprompted mode (specified with a `-n` option), I/O redirection, piping, and background processes

Test scripts (generously provided by classmate on Piazza):

- [touchCommandTest.sh](touchCommandTest.sh): Tests file creation with `touch` command
- [pipedCommandTest.sh](pipedCommandTest.sh): Tests piped commands with redirection
- [pipedCommandTest2.sh](pipedCommandTest2.sh): Tests even more pipes with redirection

## EC440: Shell Parser

The sample code provided for the Parser includes:

- [Makefile](Makefile): See Makefile under the "EC440: Shell" section ~~The Makefile that builds your parser, tests, and runs your test programs as described [here](https://openosorg.github.io/openos/textbook/intro/tools-make.html#a-simple-example)~~
- [myshell_parser.h](myshell_parser.h): The header file that describes the structures and interfaces that clients of your parser needs to know (see [this](https://openosorg.github.io/openos/textbook/intro/tools-testing.html#testing))
- [myshell_parser.c](myshell_parser.c): Course-staff-provided parser solution ~~An initial stub implementation of the functions that you will need to fill in~~
- [run_tests.sh](run_tests.sh): A shell script, described [here](https://openosorg.github.io/openos/textbook/intro/tools-shell.html#shell) for automating running all the tests

Two example test programs that exercise the interface of the parser, discussed [here](https://openosorg.github.io/openos/textbook/intro/tools-testing.html#testing), are:

- [test_simple_input.c](test_simple_input.c): Tests parsing the command `ls`
- [test_simple_pipe.c](test_simple_pipe.c): Tests parsing the command `ls | cat`

Additional test programs I added (* = credit to Piazza post):

- [test_simple_input2.c](test_simple_input2.c): Tests variations of `ls` *
- [test_simple_pipe2.c](test_simple_pipe2.c): Tests variations of `ls | cat` *
- [test_simple_background.c](test_simple_background.c): Tests variations of `ls &` *
- [test_simple_redirect.c](test_simple_redirect.c): Tests variations of `cat < a_file` *
- [test_double_redirect.c](test_double_redirect.c): Tests `cat < x > ` *
- [test_all_features.c](test_all_features.c): Tests `cat < input.txt | grep -v 42 | wc > output.txt &` and `ls -al < input.txt | cat > foo.txt` *
- [test_more_tests.c](test_more_tests.c): Tests various commands that match the description of the Gradescope testcases
- [test_strlen.c](test_strlen.c): Tests `strncat()` and `strlen()` commands to make sure functions are understood

My original parser:

- [jilin_parser.c](jilin_parser.c): My original implementation which seemed to have a strange memory error that I could not debug

Implementation Strategy:
I first tried to tokenize the command line given using functions like `strtok()` and `strdup()`, but kept running into strange memory issues that I just could not debug/did not have control over, i.e. potentially misusing the string.h commands. Instead, I opted for a character-by-character approach. I managed to pass 6/7 test cases, but I still ended up with a weird overwriting problem where I could save one command and its arguments perfectly fine, but upon allocating space for the next command, I would overwrite my previously saved command. The strange thing is that it only happened for one of my test cases, despite my other cases having multiple commands and arguments too. I could not debug it before the parser solution was due, but I certainly gave it everything I had.
