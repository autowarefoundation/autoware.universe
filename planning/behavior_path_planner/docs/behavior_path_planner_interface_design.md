# Interface design

## State transition

```plantuml
@startuml
title PlannerManager::run Method Execution Flow

skinparam defaultTextAlignment center
skinparam noteTextAlignment left
skinparam backgroundColor #FFFFFF
skinparam ArrowColor #333333
skinparam ArrowFontColor #333333
skinparam partitionBackgroundColor #FFCC66

start
if (root_lanelet_ is not set) then (yes)
  :root_lanelet_ = updateRootLanelet(data);
else (no)
endif

repeat
  :Iterate over manager_ptrs_ to setData;
repeat while (Any manager_ptr left)

partition "Result Output" {
  if (is_any_approved_module_running) then (yes)
    if (is_any_candidate_module_running_or_idle) then (yes)
      if (is_out_of_route) then (yes)
        :output = createGoalAroundPath();
        :generateCombinedDrivableArea(output, data);
        stop
      else (no)
      endif
    else (no)
    endif
  else (no)
  endif

  repeat
    :approved_modules_output = runApprovedModules();
    :request_modules = getRequestModules();
    if (request_modules is empty) then (yes)
      stop
    else (no)
    endif

    :[highest_priority_module, candidate_modules_output] = runRequestModules();
    if (highest_priority_module is not set) then (yes)
      stop
    else (no)
    endif

    if (highest_priority_module is waiting approval) then (yes)
      stop
    else (no)
    endif

    :addApprovedModule(highest_priority_module);
    :clearCandidateModules();

    if (iteration limit reached) then (yes)
      stop
    else (no)
    endif
  repeat while (Modules to process or iteration limit not reached)
}

repeat
  :Iterate over manager_ptrs_ to updateObserver;
repeat while (Any manager_ptr left)

:generateCombinedDrivableArea(result_output, data);

stop
@enduml
```

### Running Approved Modules

```plantuml
@startuml
title PlannerManager::runApprovedModules Method Execution Flow

skinparam defaultTextAlignment center
skinparam noteTextAlignment left
skinparam backgroundColor #FFFFFF
skinparam ArrowColor #333333
skinparam ArrowFontColor #333333

start

:Initialize results and output;
:output = getReferencePath(data);
:results.emplace("root", output);

if (approved_module_ptrs_ is empty) then (yes)
  :Return output;
  stop
else (no)
endif

partition #LightBlue "Module Reordering" {
  :Move certain modules to end based on conditions;
}

partition #LightGreen "Lock & Unlock Approved Modules" {
  :Lock output paths of approved modules;
  :Unlock last module if conditions met;
}

partition #Orange "Execute Approved Modules" {
  repeat :For each module_ptr in approved_module_ptrs_;
    :Run module and store result;
  repeat while (More module_ptr left) is (yes)
}

partition #LightYellow "Handle Waiting Approval Modules" {
  :Check and handle waiting approval modules;
  :Update observers;
  if (Modules found waiting for approval) then (yes)
    :Clear and handle candidate modules;
    :Update and erase modules from approved list;
    :Update observers;
  endif
}

partition #LightGray "Remove Failure Modules" {
  :Remove modules with FAILURE status;
  :Update observers;
  if (Modules removed) then (yes)
    :Clear candidate modules;
  endif
}

if (approved_module_ptrs_ is empty after removal) then (yes)
  :Return root result;
  stop
else (no)
endif

partition #Coral "Finalize Output" {
  :Determine final output based on last module's result;
  :Handle success modules and update root lanelet if necessary;
  :Update observers;
}

:Return approved_modules_output;

stop
@enduml
```

### Getting Requested Modules

```plantuml
@startuml
title PlannerManager::getRequestModules Method Execution Flow

skinparam defaultTextAlignment center
skinparam noteTextAlignment left
skinparam backgroundColor #FFFFFF
skinparam ArrowColor #333333
skinparam ArrowFontColor #333333
skinparam partitionBackgroundColor #CCFF33

start
if (previous_module_output.path is null) then (yes)
  :Log error: "Current module output is null...";
  stop
else (no)
endif

:Initialize request_modules as empty vector;

partition "Process Managers" {
  repeat :For each manager_ptr in manager_ptrs_;
    :stop_watch_.tic(manager_ptr->name());
    :Determine conditions for module execution;
    if (skip_module) then (yes)
      :toc(manager_ptr->name());
    else (no)
      :Check and possibly add new module;
    endif
    :toc(manager_ptr->name());
  repeat while (Any manager_ptr left)
}

:Return request_modules;

stop
@enduml
```

### Running Requested Modules

```plantuml
@startuml
title PlannerManager::runRequestModules Method Execution Flow

skinparam defaultTextAlignment center
skinparam noteTextAlignment left
skinparam backgroundColor #FFFFFF
skinparam ArrowColor #333333
skinparam ArrowFontColor #333333

start

:Initialize vectors for modules and results;
:Sort request_modules by priority;

partition #LightBlue "Remove Non-executable Modules" {
  repeat :For each module_ptr in sorted_request_modules;
    :Determine conditions for module execution;
    if (Module is executable) then (yes)
      :Add to executable_modules;
    endif
  repeat while (Any module_ptr left) is (yes) not (no)
}

partition #LightGreen "Run Executable Modules" {
  repeat :For each module_ptr in executable_modules;
    :Run module and store result;
  repeat while (Any module_ptr left) is (yes) not (no)
}

partition #Orange "Remove Expired Modules" {
  :Remove modules with FAILURE or SUCCESS status;
  :Update observers of managers;
}

if (executable_modules is empty) then (yes)
  :clearCandidateModules();
  stop
else (no)
endif

partition #LightYellow "Separate by Approval Condition" {
  repeat :For each module in executable_modules;
    if (Module is waiting approval) then (yes)
      :Add to waiting_approved_modules;
    else (no)
      :Add to already_approved_modules;
    endif
  repeat while (Any module left) is (yes) not (no)
}

partition #LightGray "Select Highest Priority Module" {
  :Choose highest priority from approved or waiting modules;
}

:Update candidate modules;
:Return highest priority module and its result;

stop
@enduml
```

### Update Module State

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam noteTextAlignment left
skinparam backgroundColor #FFFFFF
skinparam ArrowColor #333333
skinparam ArrowFontColor #333333

title Module State Update Flowchart

' IDLE State logic
partition #FFCC66 "IDLE State" {
  start
  if(current_state_ == ModuleStatus::IDLE ?) then (yes)
    if(canTransitIdleToRunningState() ?) then (yes)
      :return ModuleStatus::RUNNING;
      stop
    else (no)
      :return ModuleStatus::IDLE;
      stop
    endif
  else (no)
  endif
}

' RUNNING State logic
partition #66CC99 "RUNNING State" {
  if(current_state_ == ModuleStatus::RUNNING ?) then (yes)
    if(canTransitSuccessState() ?) then (yes)
      :return ModuleStatus::SUCCESS;
      stop
    else (no)
    endif

    if(canTransitFailureState() ?) then (yes)
      :return ModuleStatus::FAILURE;
      stop
    else (no)
    endif

    if(canTransitWaitingApprovalState() ?) then (yes)
      :return ModuleStatus::WAITING_APPROVAL;
      stop
    else (no)
      :return ModuleStatus::RUNNING;
      stop
    endif
  else (no)
  endif
}

' WAITING_APPROVAL State logic
partition #6699CC "WAITING_APPROVAL State" {
  if(current_state_ == ModuleStatus::WAITING_APPROVAL ?) then (yes)
    if(canTransitSuccessState() ?) then (yes)
      :return ModuleStatus::SUCCESS;
      stop
    else (no)
    endif

    if(canTransitFailureState() ?) then (yes)
      :return ModuleStatus::FAILURE;
      stop
    else (no)
    endif

    if(canTransitWaitingApprovalToRunningState() ?) then (yes)
      :return ModuleStatus::RUNNING;
      stop
    else (no)
      :return ModuleStatus::WAITING_APPROVAL;
      stop
    endif
  else (no)
  endif
}

' Terminal States logic
partition #99CC66 "Terminal States" {
  if(current_state_ == ModuleStatus::SUCCESS ?) then (yes)
    :return ModuleStatus::SUCCESS;
    stop
  else (no)
  endif

  if(current_state_ == ModuleStatus::FAILURE ?) then (yes)
    :return ModuleStatus::FAILURE;
    stop
  else (no)
  endif

  :return ModuleStatus::IDLE;
  stop
}
@enduml
```

!!! warning

    Under Construction
