#include "myshell_parser.h"
#include "stddef.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "assert.h"
#include "errno.h"

struct pipeline *pipeline_build(const char *command_line){
  struct pipeline *the_pipeline;
  struct pipeline *pipeline_check = malloc(sizeof(struct pipeline));
  if (pipeline_check == NULL) {
    perror("malloc returned NULL");
  } else {
    the_pipeline = pipeline_check;
    pipeline_check = NULL;
  }

  struct pipeline_command *pipeline_command_check = malloc(sizeof(struct pipeline_command));
  if (pipeline_command_check == NULL){
    perror("malloc returned NULL");
  } else {
    the_pipeline->commands = pipeline_command_check;
    pipeline_command_check = NULL;
  }
  the_pipeline->is_background = false;

  struct pipeline_command *curr_cmd = the_pipeline->commands; // track the current command
  curr_cmd->command_args[0] = NULL;
  curr_cmd->redirect_in_path = NULL;
  curr_cmd->redirect_out_path = NULL;
  curr_cmd->next = NULL;
  
  int trav_ptr = 0; // traversing pointer for the command line
  char buffer[MAX_LINE_LENGTH] = {0}; // to hold the current token
  int cmd_arg_count = 0; // keep count of how many cmds/args there have been in the curr_cmd
  
  while (command_line[trav_ptr] != '\0' && command_line[trav_ptr] != '\n' && trav_ptr < strlen(command_line)){
    switch (command_line[trav_ptr]){
    case '&':
      if (strlen(buffer) != 0){ // if the buffer is not empty
	char *token_check = malloc(((int)strlen(buffer)+1)*sizeof(char));
	if (token_check == NULL){
	  perror("malloc returned NULL");
	} else {
	  curr_cmd->command_args[cmd_arg_count] = token_check;
	  token_check = NULL;
	}
	memcpy(curr_cmd->command_args[cmd_arg_count], buffer, strlen(buffer)+1);
	//	curr_cmd->command_args[cmd_arg_count][(int)strlen(buffer)] = '\0'; // set the null-terminating byte
	cmd_arg_count++;
	memset(buffer, 0, sizeof(buffer)); // clear the buffer
      }
      the_pipeline->is_background = true;
      break;

    case '<':
      if (strlen(buffer) != 0){ // if the buffer is not empty
	char *token_check = malloc(((int)strlen(buffer)+1)*sizeof(char));
	if (token_check == NULL){
	  perror("malloc returned NULL");
	} else {
	  curr_cmd->command_args[cmd_arg_count] = token_check;
	  token_check = NULL;
	}
	memcpy(curr_cmd->command_args[cmd_arg_count], buffer, strlen(buffer)+1);
	//	curr_cmd->command_args[cmd_arg_count][(int)strlen(buffer)] = '\0'; // set the null-terminating byte
	cmd_arg_count++;
	memset(buffer, 0, sizeof(buffer)); // clear the buffer
      }
      trav_ptr++;
      while (command_line[trav_ptr] == ' '){
	trav_ptr++;
      }
      while (command_line[trav_ptr] != ' ' && command_line[trav_ptr] != '\n'
	     && command_line[trav_ptr+1] != '|' && command_line[trav_ptr+1] != '>' && command_line[trav_ptr+1] != '&'){
	strncat(buffer, &command_line[trav_ptr], 1);
	trav_ptr++;
      }
      char *token_check = malloc(((int)strlen(buffer)+1)*sizeof(char));
      if (token_check == NULL){
	perror("malloc returned NULL");
      } else {
	curr_cmd->redirect_in_path = token_check;
	token_check = NULL;
      }
      memcpy(curr_cmd->redirect_in_path, buffer, strlen(buffer)+1);
      //      curr_cmd->redirect_in_path[(int)strlen(buffer)] = '\0'; // set the null-terminating byte
      memset(buffer, 0, sizeof(buffer)); // clear the buffer
      break;

    case '>':
      if (strlen(buffer) != 0){ // if the buffer is not empty
	char *token_check = malloc(((int)strlen(buffer)+1)*sizeof(char));
	if (token_check == NULL){
	  perror("malloc returned NULL");
	} else {
	  curr_cmd->command_args[cmd_arg_count] = token_check;
	  token_check = NULL;
	}
	memcpy(curr_cmd->command_args[cmd_arg_count], buffer, strlen(buffer)+1);
	//	curr_cmd->command_args[cmd_arg_count][(int)strlen(buffer)] = '\0'; // set the null-terminating byte
	cmd_arg_count++;
	memset(buffer, 0, sizeof(buffer)); // clear the buffer
      }
      trav_ptr++;
       while (command_line[trav_ptr] == ' '){
	trav_ptr++;
      }
      while (command_line[trav_ptr] != ' ' && command_line[trav_ptr] != '\n'
	     && command_line[trav_ptr+1] != '|' && command_line[trav_ptr+1] != '<' && command_line[trav_ptr+1] != '&'){
	strncat(buffer, &command_line[trav_ptr], 1);
	trav_ptr++;
      }
      token_check = malloc(((int)strlen(buffer)+1)*sizeof(char));
      if (token_check == NULL){
	perror("malloc returned NULL");
      } else {
	curr_cmd->redirect_out_path = token_check;
      }
      memcpy(curr_cmd->redirect_out_path, buffer, strlen(buffer)+1);
      //      curr_cmd->redirect_out_path[(int)strlen(buffer)] = '\0'; // set the null-terminating byte
      memset(buffer, 0, sizeof(buffer)); // clear the buffer
      break;

    case '|':
      if (strlen(buffer) != 0){ // if the buffer is not empty
	token_check = malloc (((int)strlen(buffer)+1)*sizeof(char));
	if (token_check == NULL){
	  perror("malloc returned NULL");
	} else {
	  curr_cmd->command_args[cmd_arg_count] = token_check;
	  token_check = NULL;
	}
	memcpy(curr_cmd->command_args[cmd_arg_count], buffer, strlen(buffer)+1);
	//	curr_cmd->command_args[cmd_arg_count][(int)strlen(buffer)] = '\0'; // set the null-terminating byte
	cmd_arg_count++;
	memset(buffer, 0, sizeof(buffer)); // clear the buffer
      }
      pipeline_command_check = malloc(sizeof(struct pipeline_command));
      if (pipeline_command_check == NULL){
	perror("malloc returned NULL");
      } else {
	curr_cmd->next = pipeline_command_check; 
      	pipeline_command_check = NULL;	       
      	curr_cmd = curr_cmd->next;	       
      	curr_cmd->redirect_in_path = NULL;       
      	curr_cmd->redirect_out_path = NULL;      
      	curr_cmd->next = NULL;		       
      	cmd_arg_count = 0;                       
      }
      break;

    default:
      if (command_line[trav_ptr] != ' '){ // if the current char is not whitespace
	strncat(buffer, &command_line[trav_ptr], 1); // add the current char to the buffer
      } else if (strlen(buffer) != 0){ // if the buffer is not empty
	token_check = malloc (((int)strlen(buffer)+1)*sizeof(char));
	if (token_check == NULL){
	  perror("malloc returned NULL");
	} else {
	  curr_cmd->command_args[cmd_arg_count] = token_check;
	}
	memcpy(curr_cmd->command_args[cmd_arg_count], buffer, strlen(buffer)+1);
	//	curr_cmd->command_args[cmd_arg_count][(int)strlen(buffer)] = '\0'; // set the null-terminating byte
	cmd_arg_count++;
	memset(buffer, 0, sizeof(buffer)); // clear the buffer
      }
     
      break;
    }

    trav_ptr++;
  }

  if (strlen(buffer) != 0){ // if the buffer is not empty
    char *token_check = malloc (((int)strlen(buffer)+1)*sizeof(char));
    if (token_check == NULL){
      perror("malloc returned NULL");
    } else {
      curr_cmd->command_args[cmd_arg_count] = token_check;
    }
    memcpy(curr_cmd->command_args[cmd_arg_count], buffer, strlen(buffer)+1);
    //    curr_cmd->command_args[cmd_arg_count][(int)strlen(buffer)] = '\0'; // set the null-terminating byte
    cmd_arg_count++;
    memset(buffer, 0, sizeof(buffer)); // clear the buffer
  }
  
  curr_cmd->command_args[cmd_arg_count] = NULL;

  return the_pipeline;
}


void pipeline_free(struct pipeline *pipeline)
{
  struct pipeline_command *cmd_ptr = pipeline->commands, *tmp_cmd_ptr = NULL;
  while (cmd_ptr->next != NULL){
    tmp_cmd_ptr = cmd_ptr;
    cmd_ptr = cmd_ptr->next;

    // free each command/arg WITHIN command_args
    int cmd_arg_count = 0;
    while (tmp_cmd_ptr->command_args[cmd_arg_count] != NULL){
      free(tmp_cmd_ptr->command_args[cmd_arg_count]);
      tmp_cmd_ptr->command_args[cmd_arg_count] = NULL;
      cmd_arg_count++;
    }
    
    if (tmp_cmd_ptr->redirect_in_path != NULL){
      free(tmp_cmd_ptr->redirect_in_path);
      tmp_cmd_ptr->redirect_in_path = NULL;
    }

    if (tmp_cmd_ptr->redirect_out_path != NULL){
      free(tmp_cmd_ptr->redirect_out_path);
      tmp_cmd_ptr->redirect_out_path = NULL;
    }
    
    free(tmp_cmd_ptr);
    tmp_cmd_ptr = NULL;
  }

  // free each command/arg WITHIN command_args
  int cmd_arg_count = 0;
  while (cmd_ptr->command_args[cmd_arg_count] != NULL){
    free(cmd_ptr->command_args[cmd_arg_count]);
    cmd_ptr->command_args[cmd_arg_count] = NULL;
    cmd_arg_count++;
  }
 
  if (cmd_ptr->redirect_in_path != NULL){
    free(cmd_ptr->redirect_in_path);
    cmd_ptr->redirect_in_path = NULL;
  }

  if (cmd_ptr->redirect_out_path != NULL){
    free(cmd_ptr->redirect_out_path);
    cmd_ptr->redirect_out_path = NULL;
  }
  
  free(cmd_ptr);
  cmd_ptr = NULL;
  
  free(pipeline);
  pipeline = NULL;
}
