#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Queues.h"

// Initialize the StringQueue
void initQueue(StringQueue* q) {
    q->front = q->rear = -1;
    for (int i = 0; i < SIZE; i++) {
        q->items[i] = NULL;
    }
}

// Check if the queue is empty
bool isEmpty(StringQueue* q) {
    return (q->front == -1);
}

// Check if the StringQueue is full
bool isFull(StringQueue* q) {
    return (q->rear == SIZE - 1);
}

// Enqueue operation
void enqueue(StringQueue* q, const char* value) {
    if (isFull(q)) {
        return; // Queue is full
    }
    
    if (isEmpty(q)) {
        q->front = q->rear = 0;
    } else {
        q->rear++;
    }
    
    // Allocate memory for the string and copy it
    q->items[q->rear] = (char*)malloc(strlen(value) + 1);
    strcpy(q->items[q->rear], value);
}

// Dequeue operation
char* dequeue(StringQueue* q) {
    if (isEmpty(q)) {
        return NULL; // Queue is empty
    }
    
    char* value = q->items[q->front];
    
    if (q->front == q->rear) {
        q->front = q->rear = -1; // Reset queue
    } else {
        q->front++;
    }
    
    return value;
}

// Get the front element without removing it
char* peek(StringQueue* q) {
    if (isEmpty(q)) {
        return NULL; // Queue is empty
    }
    
    return q->items[q->front];
}

// Free all allocated memory in the queue
void freeQueue(StringQueue* q) {
    for (int i = 0; i < SIZE; i++) {
        if (q->items[i] != NULL) {
            free(q->items[i]);
        }
    }
}
