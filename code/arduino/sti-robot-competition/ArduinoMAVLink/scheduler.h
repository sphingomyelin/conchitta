#ifndef scheduler.h
#define scheduler.h

#define MAX_NUMBER_OF_TASKS 10

typedef void (function_pointer)(void);

typedef struct {
  function_pointer *call_function;
  unsigned int repeat_period;
  unsigned long next_run;
} task_entry;

void init_scheduler();

int register_task(int task_slot, function_pointer *call_function, unsigned int repeat_period_ms);

int run_scheduler_update();

#endif
