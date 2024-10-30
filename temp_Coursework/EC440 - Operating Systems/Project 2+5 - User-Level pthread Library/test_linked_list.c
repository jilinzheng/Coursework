#include <stdio.h>
#include <assert.h>
#include <stdlib.h>

// proj 5: singly-linked list for the list of waiting threads
typedef struct Node {
	int payload;
	struct Node *next;
} Node;

// proj 5: linked list helper functions
static Node* create_node(int payload){
	Node *new_node = malloc(sizeof(Node));
	new_node->payload = payload;
	new_node->next = NULL;
	return new_node;
}

static void insert_thread(Node **head, int payload){
	Node *new_node = create_node(payload);
	if (*head == NULL){
		*head = new_node;
		return;
	}

	Node *temp = *head;
	while (temp->next != NULL){
		temp = temp->next;
	}	
	temp->next = new_node;
}

static void remove_thread(Node **head, int payload){
	Node *the_head = *head;
	if (the_head->payload == payload){
		Node *temp = NULL;
		temp = *head;
		*head = the_head->next;
		free(temp);
		return;
	}

	// my use-case probably will not use the following section
	// i am only inserting to the end + removing at the head, i.e. queue
	Node *curr = *head;
	while (curr->next != NULL){
		if (curr->next->payload == payload){
			Node *temp = curr->next;
			curr->next = curr->next->next;
			free(temp);
			return;
		}
		curr = curr->next;
	}
}

static void remove_head(Node **head){
	Node *temp = *head;
	*head = temp->next;
	free(temp);
}

static int get_head(Node *head){
	if (head == NULL){
		return -1;
	}

	return head->payload;
}

int main(int argc, char *argv[]){
	Node *list = NULL;
	int payload = -1;

	for (int ii = 0; ii < 5; ++ii){
		insert_thread(&list, ii);
	}	

	for (int ii = 0; ii < 5; ++ii){
		payload = get_head(list);
		printf("Current head payload is: %d\n", payload);
		remove_thread(&list, payload);
	}

	assert(list == NULL);

	for (int ii = 0; ii < 5; ++ii){
		insert_thread(&list, ii);
	}	

	for (int ii = 0; ii < 5; ++ii){
		payload = get_head(list);
		printf("Current head payload is: %d\n", payload);
		remove_head(&list);
	}

	assert(list == NULL);

	return 0;
}
