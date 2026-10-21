#include "threadpool.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_MAIN
#include "log.h"

static void *thread_do_work(void *arg) {
    threadpool_t *pool = (threadpool_t *)arg;

    while (1) {
        pthread_mutex_lock(&pool->lock);

        while (pool->task_queue_head == NULL && !pool->shutdown) {
            pthread_cond_wait(&pool->notify, &pool->lock);
        }

        if (pool->shutdown && pool->task_queue_head == NULL) {
            pthread_mutex_unlock(&pool->lock);
            break;
        }

        task_t *task = pool->task_queue_head;
        if (task != NULL) {
            pool->task_queue_head = task->next;
            if (pool->task_queue_head == NULL) {
                pool->task_queue_tail = NULL;
            }
        }

        pthread_mutex_unlock(&pool->lock);

        if (task != NULL) {
            task->function(task->arg);
            free(task);
        }
    }

    return NULL;
}

threadpool_t *threadpool_create(int thread_count) {
    printf("[POOL] ▶ 创建线程池开始: 请求 %d 个工作线程\n", thread_count);

    if (thread_count <= 0) {
        printf("[POOL] ❌ 创建失败: 线程数 %d 不合法\n", thread_count);
        return NULL;
    }

    threadpool_t *pool = (threadpool_t *)calloc(1, sizeof(*pool));
    if (pool == NULL) {
        printf("[POOL] ❌ 创建失败: 分配 threadpool_t 内存出错: %s\n", strerror(errno));
        return NULL;
    }

    pool->thread_count = thread_count;

    int ret = pthread_mutex_init(&pool->lock, NULL);
    if (ret != 0) {
        printf("[POOL] ❌ 创建失败: 初始化互斥锁出错: %s\n", strerror(ret));
        free(pool);
        return NULL;
    }

    ret = pthread_cond_init(&pool->notify, NULL);
    if (ret != 0) {
        printf("[POOL] ❌ 创建失败: 初始化条件变量出错: %s\n", strerror(ret));
        pthread_mutex_destroy(&pool->lock);
        free(pool);
        return NULL;
    }

    pool->threads = (pthread_t *)calloc((size_t)thread_count, sizeof(pthread_t));
    if (pool->threads == NULL) {
        printf("[POOL] ❌ 创建失败: 分配线程句柄数组出错: %s\n", strerror(errno));
        pthread_cond_destroy(&pool->notify);
        pthread_mutex_destroy(&pool->lock);
        free(pool);
        return NULL;
    }

    for (int i = 0; i < thread_count; ++i) {
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 256 * 1024);   // 256KB 栈, 嵌入式够用
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
        // 注意: pthread_create 通过返回值报错, 不设 errno,
        //       因此不能用 perror(), 必须用 strerror(返回值)
        ret = pthread_create(&pool->threads[i], &attr, thread_do_work, pool);
        if (ret != 0) {
            printf("[POOL] ❌ 创建失败: 第 %d/%d 个工作线程创建出错: %s\n",
                   i + 1, thread_count, strerror(ret));
            pthread_attr_destroy(&attr);
            threadpool_destroy(pool);
            return NULL;
        }
        pthread_attr_destroy(&attr);
        printf("[POOL]   工作线程 %d/%d 已创建\n", i + 1, thread_count);
    }

    printf("[POOL] ✅ 创建完成: 共 %d 个工作线程已就绪\n", thread_count);
    return pool;
}

int threadpool_add(threadpool_t *pool, void (*function)(void *), void *arg) {
    if (pool == NULL || function == NULL) {
        printf("[POOL] ❌ 投递任务失败: 参数无效(pool=%p, function=%p)\n",
               (void *)pool, (const void *)function);
        return -1;
    }

    task_t *new_task = (task_t *)calloc(1, sizeof(*new_task));
    if (new_task == NULL) {
        printf("[POOL] ❌ 投递任务失败: 分配 task_t 内存出错: %s\n", strerror(errno));
        return -1;
    }

    new_task->function = function;
    new_task->arg = arg;

    pthread_mutex_lock(&pool->lock);
    if (pool->task_queue_tail == NULL) {
        pool->task_queue_head = new_task;
        pool->task_queue_tail = new_task;
    } else {
        pool->task_queue_tail->next = new_task;
        pool->task_queue_tail = new_task;
    }
    pthread_cond_signal(&pool->notify);
    pthread_mutex_unlock(&pool->lock);
    return 0;
}

void threadpool_destroy(threadpool_t *pool) {
    if (pool == NULL) {
        printf("[POOL] ⚠️ 销毁跳过: 线程池句柄为 NULL\n");
        return;
    }

    printf("[POOL] ▶ 销毁线程池开始: 等待 %d 个工作线程退出\n", pool->thread_count);

    pthread_mutex_lock(&pool->lock);
    pool->shutdown = 1;
    pthread_cond_broadcast(&pool->notify);
    pthread_mutex_unlock(&pool->lock);

    for (int i = 0; i < pool->thread_count; ++i) {
        pthread_join(pool->threads[i], NULL);
    }

    int dropped = 0;
    task_t *task = pool->task_queue_head;
    while (task != NULL) {
        task_t *next = task->next;
        free(task);
        task = next;
        dropped++;
    }
    if (dropped > 0) {
        printf("[POOL] ⚠️ 销毁时仍有 %d 个未执行的任务, 已释放\n", dropped);
    }

    pthread_mutex_destroy(&pool->lock);
    pthread_cond_destroy(&pool->notify);
    free(pool->threads);
    free(pool);
    printf("[POOL] ✅ 销毁完成: 资源已释放\n");
}
