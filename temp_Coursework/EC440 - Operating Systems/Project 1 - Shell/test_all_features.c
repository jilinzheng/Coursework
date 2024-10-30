#include "myshell_parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

int main()
{
  struct pipeline *my_pipeline = pipeline_build("cat < input.txt | grep -v 42 | wc > output.txt &\n");

  // Test that a pipeline was returned
  assert(my_pipeline != NULL);
  assert(my_pipeline->is_background);
  struct pipeline_command *current_cmd = my_pipeline->commands;
  assert(current_cmd != NULL);

  // Test the first command
  assert(strcmp("cat", current_cmd->command_args[0]) == 0);
  assert(current_cmd->command_args[1] == NULL);
  assert(strcmp(current_cmd->redirect_in_path, "input.txt") == 0);
  assert(current_cmd->redirect_out_path == NULL);

  // Test the second command
  assert(current_cmd->next != NULL);
  current_cmd = current_cmd->next;
  assert(strcmp("grep", current_cmd->command_args[0]) == 0);
  assert(strcmp("-v", current_cmd->command_args[1]) == 0);
  assert(strcmp("42", current_cmd->command_args[2]) == 0);
  assert(current_cmd->command_args[3] == NULL);
  assert(current_cmd->redirect_in_path == NULL);
  assert(current_cmd->redirect_out_path == NULL);

  // Test the third command
  assert(current_cmd->next != NULL);
  current_cmd = current_cmd->next;
  assert(strcmp("wc", current_cmd->command_args[0]) == 0);
  assert(current_cmd->command_args[1] == NULL);
  assert(current_cmd->redirect_in_path == NULL);
  assert(strcmp(current_cmd->redirect_out_path, "output.txt") == 0);

  // Test that there are no more commands
  assert(current_cmd->next == NULL);

  pipeline_free(my_pipeline);


  /* another case */
  my_pipeline = pipeline_build("ls -al < input.txt | cat > foo.txt\n");
  struct pipeline_command *curr_cmd = my_pipeline->commands;
  
  // Test that a pipeline was returned
  assert(my_pipeline != NULL);
  assert(!my_pipeline->is_background);
  assert(curr_cmd != NULL);
  
  // Test the first command
  assert(strcmp("ls", curr_cmd->command_args[0]) == 0);
  assert(strcmp("-al", curr_cmd->command_args[1]) == 0);
  assert(curr_cmd->command_args[2] == NULL);
  assert(strcmp("input.txt", curr_cmd->redirect_in_path) == 0);
  assert(curr_cmd->redirect_out_path == NULL);
  assert(curr_cmd->next != NULL);
  
  // Test the second command
  curr_cmd = curr_cmd->next;
  assert(strcmp("cat", curr_cmd->command_args[0]) == 0);
  assert(curr_cmd->command_args[1] == NULL);
  assert(curr_cmd->redirect_in_path == NULL);
  assert(strcmp("foo.txt", curr_cmd->redirect_out_path) == 0);
  assert(curr_cmd->next == NULL);
  
  pipeline_free(my_pipeline);
}
