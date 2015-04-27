#include "scheduler.h"
#include <Arduino.h>

task_entry tasks[MAX_NUMBER_OF_TASKS];


void init_scheduler() {
  int i;
  for (i=0; i<MAX_NUMBER_OF_TASKS; i++) {
    tasks[i].call_function=NULL;
  
  } 
}

int register_task(int task_slot, function_pointer *call_function, unsigned int repeat_period_ms) {
  if ((task_slot<0) || (task_slot>=MAX_NUMBER_OF_TASKS)) {
    return -1;
  }
  
  tasks[task_slot].call_function=call_function;
  tasks[task_slot].repeat_period=repeat_period_ms;
  tasks[task_slot].next_run=millis();
  
}

int run_scheduler_update() {
  int i;
  int realtime_violation=0;
  for (i=0; i<MAX_NUMBER_OF_TASKS; i++) {
    if ((tasks[i].call_function!=NULL) && (millis() >= tasks[i].next_run)) {
      tasks[i].next_run+=tasks[i].repeat_period;
      tasks[i].call_function();
      if (tasks[i].next_run<millis()) realtime_violation=-i; //realtime violation!!
    }
  }
  return realtime_violation;
  
}
