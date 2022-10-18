from statemachine import StateMachine, State

class GlobalOrchestrationSM(StateMachine):
    #### Members for debug purposes ###
    sim_started = True # Simplification for now
    can_gather = False
    has_finished = True
    ###################################

    start = State('Start', initial=True)
    waiting = State('Waiting') # Waiting for the simulation start
    spawning = State('Spawning') # Spawning the robot
    gathering = State('Gathering')
    finishing = State('Finishing')
    # This state may be removed if the simulation is restarted upon completion.
    terminating = State('Terminating')

    run = start.to(waiting)
    await_start = waiting.to(waiting)
    started = waiting.to(spawning)
    spawned = spawning.to(gathering)
    gather = gathering.to(finishing)
    navigate = gathering.to(gathering)
    finish = finishing.to(gathering)
    finished = finishing.to(terminating)

    def __init__(self) -> None:
        super().__init__()