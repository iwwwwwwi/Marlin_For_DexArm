#include "../../MarlinCore.h"
#include "../../module/planner.h"
#include "../../gcode/gcode.h"

void rail_init(float feedRate_t, int direction) {
    int rail_feedrate;
    rail_feedrate = int(65535/feedRate_t);
    OUT_WRITE(E0_ENABLE_PIN, LOW);
    OUT_WRITE(X_ENABLE_PIN, LOW);
    OUT_WRITE(Y_ENABLE_PIN, LOW);
    OUT_WRITE(Z_ENABLE_PIN, LOW);

    if(direction == 0){
      OUT_WRITE(E0_DIR_PIN, LOW);
      OUT_WRITE(X_DIR_PIN, LOW);
      OUT_WRITE(Y_DIR_PIN, LOW);
      OUT_WRITE(Z_DIR_PIN, LOW);
      
    }else if(direction == 1){
      OUT_WRITE(E0_DIR_PIN, HIGH);
      OUT_WRITE(X_DIR_PIN, HIGH);
      OUT_WRITE(Y_DIR_PIN, HIGH);
      OUT_WRITE(Z_DIR_PIN, HIGH);
    }
    HAL_timer_start(RAIL_TIMER_NUM, 4*planner.settings.axis_steps_per_mm[E_AXIS]);
    ENABLE_RAIL_INTERRUPT();
    HAL_timer_set_compare(RAIL_TIMER_NUM, rail_feedrate);
}

void rail_disable() {
  DISABLE_RAIL_INTERRUPT();
}

HAL_RAIL_TIMER_ISR() {
  HAL_timer_isr_prologue(RAIL_TIMER_NUM);

  TOGGLE(E0_STEP_PIN);
  TOGGLE(X_STEP_PIN);
  TOGGLE(Y_STEP_PIN);
  TOGGLE(Z_STEP_PIN);

  HAL_timer_isr_epilogue(RAIL_TIMER_NUM);
}
