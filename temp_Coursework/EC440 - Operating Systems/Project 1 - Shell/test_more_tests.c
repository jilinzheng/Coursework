#include "myshell_parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

int main(void){
  puts("A background command can be parsed.");
  struct pipeline* my_pipeline = pipeline_build("ls &\n");
  struct pipeline_command* curr_cmd = my_pipeline->commands;
  assert(my_pipeline != NULL);
  assert(my_pipeline->is_background);
  assert(curr_cmd != NULL);
  assert(strcmp("ls", curr_cmd->command_args[0]) == 0);
  assert(curr_cmd->command_args[1] == NULL);
  assert(curr_cmd->redirect_in_path == NULL);
  assert(curr_cmd->redirect_out_path == NULL);
  assert(curr_cmd->next == NULL);
  pipeline_free(my_pipeline);
  my_pipeline = NULL;
  free(curr_cmd);
  curr_cmd = NULL;
  
  puts("A single, stdin-redirected command can be parsed.");
  my_pipeline = pipeline_build("ls < input.txt\n");
  curr_cmd = my_pipeline->commands;
  assert(my_pipeline != NULL);
  assert(!my_pipeline->is_background);
  assert(curr_cmd != NULL);
  assert(strcmp("ls", curr_cmd->command_args[0]) == 0);
  assert(curr_cmd->command_args[1] == NULL);
  assert(strcmp("input.txt", curr_cmd->redirect_in_path) == 0);
  assert(curr_cmd->redirect_out_path == NULL);
  assert(curr_cmd->next == NULL);
  pipeline_free(my_pipeline);
  my_pipeline = NULL;
  free(curr_cmd);
  curr_cmd = NULL;
  
  puts("A single, stdout-redirected command can be parsed.");
  my_pipeline = pipeline_build("ls > output.txt\n");
  curr_cmd = my_pipeline->commands;
  assert(my_pipeline != NULL);
  assert(!my_pipeline->is_background);
  assert(curr_cmd != NULL);
  assert(strcmp("ls", curr_cmd->command_args[0]) == 0);
  assert(curr_cmd->command_args[1] == NULL);
  assert(curr_cmd->redirect_in_path == NULL);
  assert(strcmp("output.txt", curr_cmd->redirect_out_path) == 0);
  assert(curr_cmd->next == NULL);
  pipeline_free(my_pipeline);
  my_pipeline = NULL;
  free(curr_cmd);
  curr_cmd = NULL;
   
  puts("A command that redirects both stdin and stdout can be parsed.");
  my_pipeline = pipeline_build("ls < input.txt | cat > output.txt\n");
  curr_cmd = my_pipeline->commands;
  assert(my_pipeline != NULL);
  assert(!my_pipeline->is_background);
  assert(curr_cmd != NULL);
  assert(strcmp("ls", curr_cmd->command_args[0]) == 0);
  assert(curr_cmd->command_args[1] == NULL);
  assert(strcmp("input.txt", curr_cmd->redirect_in_path) == 0);
  assert(curr_cmd->redirect_out_path == NULL);
  assert(curr_cmd->next != NULL);
  curr_cmd = curr_cmd->next;
  assert(my_pipeline != NULL);
  assert(!my_pipeline->is_background);
  assert(curr_cmd != NULL);
  assert(strcmp("cat", curr_cmd->command_args[0]) == 0);
  assert(curr_cmd->command_args[1] == NULL);
  assert(curr_cmd->redirect_in_path == NULL);
  assert(strcmp("output.txt", curr_cmd->redirect_out_path) == 0);
  assert(curr_cmd->next == NULL);
  pipeline_free(my_pipeline);
  my_pipeline = NULL;
  free(curr_cmd);
  curr_cmd = NULL;

  puts("A single, two-word command can be parsed with varying whitespace around words.");
  my_pipeline = pipeline_build("  ls      -l    \n");
  curr_cmd = my_pipeline->commands;
  assert(my_pipeline != NULL);
  assert(!my_pipeline->is_background);
  assert(curr_cmd != NULL);
  assert(strcmp("ls", curr_cmd->command_args[0]) == 0);
  assert(strcmp("-l", curr_cmd->command_args[1]) == 0);
  assert(curr_cmd->command_args[2] == NULL);
  assert(curr_cmd->redirect_in_path == NULL);
  assert(curr_cmd->redirect_out_path == NULL);
  assert(curr_cmd->next == NULL);
  pipeline_free(my_pipeline);
  my_pipeline = NULL;
  free(curr_cmd);
  curr_cmd = NULL;
	 
  puts("Two or more piped commands can be parsed.");
  my_pipeline = pipeline_build("ls|man|cat\n");
  curr_cmd = my_pipeline->commands;
  assert(my_pipeline != NULL);
  assert(!my_pipeline->is_background);
  assert(curr_cmd != NULL);
  assert(strcmp("ls", curr_cmd->command_args[0]) == 0);
  assert(curr_cmd->command_args[1] == NULL);
  assert(curr_cmd->redirect_in_path == NULL);
  assert(curr_cmd->redirect_out_path == NULL);
  assert(curr_cmd->next != NULL);
  curr_cmd = curr_cmd->next;
  assert(my_pipeline != NULL);
  assert(!my_pipeline->is_background);
  assert(curr_cmd != NULL);
  assert(strcmp("man", curr_cmd->command_args[0]) == 0);
  assert(curr_cmd->command_args[1] == NULL);
  assert(curr_cmd->redirect_in_path == NULL);
  assert(curr_cmd->redirect_out_path == NULL);
  assert(curr_cmd->next != NULL);
  curr_cmd = curr_cmd->next;
  assert(my_pipeline != NULL);
  assert(!my_pipeline->is_background);
  assert(curr_cmd != NULL);
  assert(strcmp("cat", curr_cmd->command_args[0]) == 0);
  assert(curr_cmd->command_args[1] == NULL);
  assert(curr_cmd->redirect_in_path == NULL);
  assert(curr_cmd->redirect_out_path == NULL);
  assert(curr_cmd->next == NULL);
  pipeline_free(my_pipeline);
  my_pipeline = NULL;
  free(curr_cmd);
  curr_cmd = NULL;
	 
  puts("The shell can parse a command that uses multiple features of the shell.");
  my_pipeline = pipeline_build(" ls -l < input.txt | cat > output.txt &\n");
  curr_cmd = my_pipeline->commands;
  assert(my_pipeline != NULL);
  assert(!my_pipeline->is_background);
  assert(curr_cmd != NULL);
  assert(strcmp("ls", curr_cmd->command_args[0]) == 0);
  assert(strcmp("-l", curr_cmd->command_args[1]) == 0);
  assert(curr_cmd->command_args[2] == NULL);
  assert(strcmp("input.txt", curr_cmd->redirect_in_path) == 0);
  assert(curr_cmd->redirect_out_path == NULL);
  assert(curr_cmd->next != NULL);
  curr_cmd = curr_cmd->next;
  assert(strcmp("cat", curr_cmd->command_args[0]) == 0);
  assert(curr_cmd->command_args[1] == NULL);
  assert(curr_cmd->redirect_in_path == NULL);
  assert(strcmp("output.txt", curr_cmd->redirect_out_path) == 0);
  pipeline_free(my_pipeline);
  my_pipeline = NULL;
  free(curr_cmd);
  curr_cmd = NULL;
}