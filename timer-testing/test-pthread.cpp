/* demonstration of real time thread on Linux:
 * Build with:
 * g++ -o prog realtime.cpp -O1 -g -lpthread
 *
 * By Jon Watte, 2019-07-19, MIT license.
 * https://github.com/jwatte
 */

#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <sched.h>
#include <stdint.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#define SAMPLE_TIME_MS 1000

bool volatile running = true;

void *myfunc(void *) {

    //  keep track of the passage of time at a 5-millisecond quantized interval
    struct timespec last = {}, now = {};
    clock_gettime(CLOCK_MONOTONIC_RAW, &last);

    while (running) {
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        //  5 milliseconds, as nanoseconds
	int64_t time_step = SAMPLE_TIME_MS * 1000000;
        int64_t tosleep = time_step - (now.tv_sec - last.tv_sec) * 1000000000 - (now.tv_nsec - last.tv_nsec);
        last.tv_nsec += time_step;
        if (last.tv_nsec >= 1000000000) {
            last.tv_nsec -= 1000000000;
            last.tv_sec += 1;
        }
        if (tosleep > (2*time_step)) {
            //  missed by more than one full interval! re-set the clock basis
            last = now;
            tosleep = 0;
        }
        if (tosleep > 0) {
            struct timespec slp = {};
            slp.tv_nsec = (long)tosleep;
            nanosleep(&slp, NULL);
        }

        //  do the thing
        //  whatever it is
	//printf("TOSLEEP : %d, LAST_N : %d, NOW_N : %d \n", tosleep, 2, 3);
    	printf("Time: %d.%d\n", now.tv_sec, now.tv_nsec);
    }

    return NULL;
}

void sigint(int) {
    running = false;
}

int main() {
    //  make sure ctrl-C stops the program under controlled circumstances
    signal(SIGINT, &sigint);

    //  create attributes for an isolated real-time thread
    pthread_attr_t attr = {};
    pthread_attr_init(&attr);
    //  lift the thread off core 0, which takes system interrupts
    cpu_set_t cpuset = {};
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);
    pthread_attr_setaffinity_np(&attr, 1, &cpuset);
    //  make it use FIFO policy for real-time scheduling
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    //  set the priority
    sched_param param = {};
    param.sched_priority = 30;
    pthread_attr_setschedparam(&attr, &param);

    //  create the real-time thread
    pthread_t mythread;
    int err;
    if ((err = pthread_create(&mythread, &attr, &myfunc, NULL)) != 0) {
        char const *emsg = ((err == EAGAIN) ? "EAGAIN" : ((err == EINVAL) ? "EINVAL" : ((err == EPERM) ? "EPERM" : "unknown")));
        fprintf(stderr, "pthread_create() failed (%d %s); are you sure you're root?\n", err, emsg);
        fprintf(stderr, "You may also need to do:\n");
        fprintf(stderr, "echo -1 > /proc/sys/kernel/sched_rt_runtime_us\n");
        exit(1);
    }

    //  wait for the program to be done
    void *ignore = NULL;
    pthread_join(mythread, &ignore);

    return 0;
}
