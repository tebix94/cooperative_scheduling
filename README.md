A very basic coopperative scheduler implemented for a STM32 F767ZI in an STMicroelectrinics Nucleo-144 board.

This project is an assignment from Dr Hirales "ADMON DE TAREAS EN SIST. EMBEBIDOS Y TIEMPO REAL" class at Cetys universidad México.

Each task works in the following states:

  1. Ready: this state just reset the task time elapsed register and pushes the tasks into the idle state.
  2. Idle: this state evaluates if the task can be pushed into the running state. For this example all tasks evaluates if its elapsed time register meets the criteria shown in the tasks enlisted below.
  3. Running: this state executes the function of the task, finally it restarts the tasks by moving back to the ready state.

The project started with the following tasks:

  1. Task 1: toggle nucleo board LED 3 which is the red one each 500 ms.
  2. Task 2: count how many times the nucleo board user button has been pressed during a period of 3 seconds.
  3. Task 3: send the count via uart3 every second

The project started without a good copperative scheduler handler. It just prioritizes the 3 tasks in order, and every time a highest order task is in idle state, the next one can be executed and so on...

I mean the algorithm has been explicit defined, then I will create a cooperative scheduler handler function to correctly look for all the tasks by iterating.
