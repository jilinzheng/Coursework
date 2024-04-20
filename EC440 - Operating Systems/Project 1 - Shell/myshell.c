#include "myshell_parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <signal.h>

int start_multi_process(struct pipeline_command *command, struct pipeline *pipeline){
  // Get the number of commands
  struct pipeline_command *curr_cmd = command;
  int cmd_count = 0;
  while (curr_cmd != NULL){
    cmd_count++;
    curr_cmd = curr_cmd->next;
  }

  // Reset curr_cmd after getting the number of commands
  curr_cmd = command;
  
  // Dynamically allocate pipes two file descriptors per pipe (W/R)
  int* pipes_fds = malloc((cmd_count - 1) * 2 * sizeof(int));
  for (int ii = 0; ii < cmd_count - 1; ii++){
    if (pipe(pipes_fds + ii * 2)){
      perror("ERROR");
      return errno;
    }
  }

  int pid;
  for (int ii = 0; ii < cmd_count; ii++){
    pid = fork();
    
    switch (pid){
      // ERROR
      case -1:
	perror("ERROR");
	return errno;
      break;

      // Child process
      case 0:
        // Check if we need to redirect input
    	if (curr_cmd->redirect_in_path != NULL){
     	  int redir_in_file = open(curr_cmd->redirect_in_path, O_RDONLY);
     	  if (redir_in_file == -1){
	    perror("ERROR");
	    return errno;
      	  }

     	  // Change STDIN to redir_in_file
      	  dup2(redir_in_file, STDIN_FILENO);
     	  close(redir_in_file);
   	} 

    	// Check if we need to redirect output
    	if (curr_cmd->redirect_out_path != NULL){
      	  int redir_out_file = open(curr_cmd->redirect_out_path, O_WRONLY | O_CREAT, 0777);
      	  if (redir_out_file == -1){
	    perror("ERROR");
	    return errno;
          }

      	  // Change STDOUT to redir_out_file
      	  dup2(redir_out_file, STDOUT_FILENO);
      	  close(redir_out_file);
    	}
  
	// WRITE
	// For the last command
	if (ii == cmd_count - 1){
	  // Close the write side of the pipe
	  close(pipes_fds[ii * 2 + 1]);
	}
	// For the rest of the commands
	else {
	  // Change the output to the write side of the pipe
	  dup2(pipes_fds[ii * 2 + 1], STDOUT_FILENO);
	  close(pipes_fds[ii * 2 + 1]);
	}

	// READ
	// For all commands but the first
	if (ii != 0){
	  // Change the input to the read side of the pipe
	  dup2(pipes_fds[ii * 2 - 2], STDIN_FILENO);
	  close(pipes_fds[ii * 2 - 2]);
	}
	// For the first command
	else {
	  // Close the read side of the pipe
	  close(pipes_fds[ii]);
	}

	// Execute the cmd with error-checking
	if (execvp(curr_cmd->command_args[0], curr_cmd->command_args) == -1){
	  perror("ERROR");
	  return errno;
	}	
      break;

      // Parent process
      default:
      	// Leave last pipe open for writing (user writing to stdin)
      	if (ii != cmd_count - 1){
	  close(pipes_fds[ii * 2 + 1]);
	}

	// Do not close read end on first command so that
	// the subsequent (2nd) command can read from the first pipe written to
	if (ii != 0){
	  close(pipes_fds[ii * 2 - 2]);
	}

	curr_cmd = curr_cmd->next;
      break;
    }
  }

  // Parent process
  if (!pipeline->is_background){
    if (pid != 0){
      // Wait all processes
      while (cmd_count > 0){
        wait(NULL);
        cmd_count--;
      }
    }
  }

  free(pipes_fds);
  
  return 0;
}

int start_single_process(struct pipeline_command *command, struct pipeline *pipeline){
  // Fork the current process
  int pid = fork();
  if (pid == -1){
    perror("ERROR");
    return errno;
  }

  // Child process
  if (pid == 0){
    
    // Check if we need to redirect input
    if (command->redirect_in_path != NULL){
      int redir_in_file = open(command->redirect_in_path, O_RDONLY);
      if (redir_in_file == -1){
	perror("ERROR");
	return errno;
      }

      // Change STDIN to redir_in_file
      dup2(redir_in_file, STDIN_FILENO);
      close(redir_in_file);
    }

    // Check if we need to redirect output
    if (command->redirect_out_path != NULL){
      int redir_out_file = open(command->redirect_out_path, O_WRONLY | O_CREAT, 0777);
      if (redir_out_file == -1){
	perror("ERROR");
	return errno;
      }

      // Change STDOUT to redir_out_file
      dup2(redir_out_file, STDOUT_FILENO);
      close(redir_out_file);
    }
    
    // Execute the cmd and args w/ error-checking
    int exec_status = execvp(command->command_args[0], command->command_args);
    if (exec_status == -1){
      perror("ERROR");
      return errno;
    }
  }
  
  // Parent process
  else {
    if (!pipeline->is_background){
      wait(&pid);
      return 0;
    }
  }
  
  return 0;
}

int start_prompted_shell(){
  // To hold user input
  char buffer[MAX_LINE_LENGTH];
  
  // Prepare to receive user input
  printf("my_shell$");
  char* read_stdin = fgets(buffer, MAX_LINE_LENGTH, stdin);

  // Stay in loop as long as we do not see NULL/EOF (CTRL+D)
  while(read_stdin != NULL){
    struct pipeline *pipeline = pipeline_build(buffer);
    struct pipeline_command *curr_cmd = pipeline->commands;

    // Single process
    if (curr_cmd->next == NULL) {
        if (start_single_process(curr_cmd, pipeline) != 0){
          printf("ERROR: Error starting the process.");
    	  return -1;
        }
      }
    
      // Multiple, piped commands
      else {  
        if (start_multi_process(curr_cmd, pipeline) != 0) {
          printf("ERROR: Error starting the process.");
	  return -1;
        }
      }
    
    // Reset the buffer for the next input
    memset(buffer, 0, sizeof(buffer));

    pipeline_free(pipeline);

    // Prepare to receive next user input
    printf("my_shell$");
    read_stdin = fgets(buffer, MAX_LINE_LENGTH, stdin);
  }
  return 0;
}
  

int start_unprompted_shell(){
  // To hold user input
  char buffer[MAX_LINE_LENGTH];
  
  // Prepare to receive user input
  char* read_stdin = fgets(buffer, MAX_LINE_LENGTH, stdin);

  // Stay in loop as long as we do not see NULL/EOF (CTRL+D)
  while(read_stdin != NULL){
    struct pipeline *pipeline = pipeline_build(buffer);
    struct pipeline_command *curr_cmd = pipeline->commands;

    // Single process
    if (curr_cmd->next == NULL) {
        if (start_single_process(curr_cmd, pipeline) != 0){
          printf("ERROR: Error starting the process.");
    	  return -1;
        }
      }
    
      // Multiple, piped commands
      else {  
        if (start_multi_process(curr_cmd, pipeline) != 0) {
          printf("ERROR: Error starting the process.");
	  return -1;
        }
      }

    // Reset the buffer for the next input
    memset(buffer, 0, sizeof(buffer));

    pipeline_free(pipeline);

    // Prepare to receive next user input
    read_stdin = fgets(buffer, MAX_LINE_LENGTH, stdin);
  }
  return 0;
}

int main(int argc, char* argv[]){
  // We only have the './myshell' arg
  if (argc == 1) {
    if (start_prompted_shell() == -1){
      printf("ERROR: An error has occurred.\n");
      return -1;
    }
  }
  // We have exactly two args and the second one is '-n'
  else if (argc == 2 && strcmp(argv[1], "-n") == 0){
      start_unprompted_shell();
  }
  return 0;
}
