#ifndef MYSHELL_PARSER_H
#define MYSHELL_PARSER_H
#include <stdbool.h>

#define MAX_LINE_LENGTH 512
#define MAX_ARGV_LENGTH (MAX_LINE_LENGTH / 2 + 1)

struct pipeline_command {
  char *command_args[MAX_ARGV_LENGTH]; // arg[0] is command, rest are arguments
  char *redirect_in_path;  // NULL or Name of file to redirect in from 
  char *redirect_out_path; // NULL or Name of a file to redirect out to
  struct pipeline_command *next; // next command in the pipeline. NULL if done 
};

struct pipeline {
  struct pipeline_command *commands; // first command
  bool is_background; // TRUE if should execue in background
};

void pipeline_free(struct pipeline *pipeline);

struct pipeline *pipeline_build(const char *command_line);

/* Additional commands from the parser solution.*/
/*
struct pipeline_command *pipeline_command_alloc();
int parse_background(const char **start, struct pipeline *pipeline);
int parse_redirect_out(const char **token, struct pipeline_command *command);
int parse_redirect_in(const char **token, struct pipeline_command *command);
int parse_args(const char **token, struct pipeline_command *command);
int parse_command(const char **token, struct pipeline_command *command);
int parse_pipes(const char **token, struct pipeline *pipeline);
void pipeline_command_free_args(struct pipeline_command *command);
*/

#endif /* MYSHELL_PARSER_H */
