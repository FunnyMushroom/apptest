#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>

typedef struct task_t {
    void (*function)(void *);
    void *arg;
    struct task_t *next;
} task_t;

typedef struct threadpool_t {
    pthread_mutex_t lock;
    pthread_cond_t notify;
    pthread_t *threads;
    task_t *task_queue_head;
    task_t *task_queue_tail;
    int thread_count;
    int shutdown;
} threadpool_t;

threadpool_t *threadpool_create(int thread_count);
int threadpool_add(threadpool_t *pool, void (*function)(void *), void *arg);
void threadpool_destroy(threadpool_t *pool);

#endif