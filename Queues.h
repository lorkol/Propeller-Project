#ifndef STRING_QUEUE_H
#define STRING_QUEUE_H

#include <stdbool.h>

// Define the maximum size of the queue and maximum string length
#define SIZE 20
#define MAX_STRING_LENGTH 9

// Define the queue structure
typedef struct {
    char* items[SIZE];
    int front;
    int rear;
} StringQueue;

// Function prototypes

// Initialize the queue
void initQueue(StringQueue* q);

// Check if the queue is empty
bool isEmpty(StringQueue* q);

// Check if the queue is full
bool isFull(StringQueue* q);

// Enqueue operation
void enqueue(StringQueue* q, const char* value);

// Dequeue operation
char* dequeue(StringQueue* q);

// Get the front element without removing it
char* peek(StringQueue* q);

// Free all allocated memory in the queue
void freeQueue(StringQueue* q);

#endif // STRING_QUEUE_H
