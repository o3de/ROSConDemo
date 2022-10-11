--[[
    Manipulator control
--]]

function clamp(value, lower_limit, upper_limit)
    if lower_limit == upper_limit then
        return value
    else
        if value > upper_limit then
            return upper_limit
        elseif value < lower_limit then
            return lower_limit
        else
            return value
        end
    end
end

local PID = {}
PID.__index = PID

function PID.new(kp, kd, ki, limit_output)
    local self = setmetatable({}, PID)
    self._kp = kp
    self._kd = kd
    self._ki = ki
    self._preError = 0
    self._integral = 0
    self._limit = limit_output
    return self
end

function PID:Reset()
    self._preError = 0
    self._integral = 0
end

function PID:Calculate(dt, err)
    local pOut = (self._kp * err)
    self._integral = self._integral + (err * dt)
    local iOut = (self._ki * self._integral)
    local deriv = ((err - self._preError) / dt)
    local dOut = (self._kd * deriv)
    output = (pOut + iOut + dOut)
    self._preError = err
    if self._limit>0.0 then
        output = clamp(output, -self._limit, self._limit)
    end
    return output
end

local manipulator_control = 
{
    Properties =
    {
    segment1 = { default = EntityId() },
    segment2 = { default = EntityId() },
    segment3 = { default = EntityId() },
    segment4 = { default = EntityId() },

    gripper = { default = EntityId() },
    rest = { default = EntityId() },
    debug = { default = EntityId() },

    apple1 = { default = EntityId() },
    apple2 = { default = EntityId() },
    }
}

State = {
    IDLE = 0,  --!< Idle state / position, suitable for robot moving around the environment.
    PREPARED = 10, --!< State and position which are ready for picking tasks.
    PICKING_BASE = 20, --!< The effector is on its way to pick fruit.
    PICKING_NOSE = 25, --!< The effector is on its way to pick fruit.
    RETRIEVING_NOSE = 30, --!< The effector is retrieving a fruit to storage position.
    RETRIEVING_BASE = 35, --!< The effector is retrieving a fruit to storage position.
    INVALID = -1, --!< Invalid state. Requires an additional context that could help user understand what happened. @see PickingState.
}

function manipulator_control:OnActivate()

    ------------------------------------
    -- Configuration parameters BEGIN --

    -- Maximum velocity in X (in vehicle axis), Y (suction tube), 
    -- Z (vertical) directions
    self.max_velocity = Vector3(40.0, 30.0, 40.0) 

    self.startupWait = 2.0 --[s]

    -- Wait till FPS reaches this value to start the simulation
    -- to prevent violent reactions right after the simulation starts
    self.startapMinFPS = 10.0

    -- Zero threshold - is used to check if manipulator reached the destination.
    -- If absolute value of error (target_position - current_position)
    -- is lower than these values, it's assumed that destination was reached.
    -- It is defined for each segment separatelly.
    self.nearZeroThresholds = {0.1, 0.1, 0.1, 0.1} -- [m]

    -- When manipulator moves, it performes X and Z movements first (movement of 
    -- the base), when destination is reached (based on self.nearZeroThresholds), 
    -- it starts moving the suction tube ("nose"). Before moving the nose it 
    -- waits self.baseZeroTimeout to stabilize.
    self.baseZeroTimeout = 0.1 --[s]

    -- If true, [Numpad 1] and [Numpad 2] keys can be used to select between 
    -- "apple1" and "apple2" entities
    self.enableKeyboardInput = true

    -- If true, current manipulaor state will be printed
    self.printState = true

    -- Time to wait in when the parking (aka retrieving) position is reached
    self.retrieveWait = 0.5 -- [s]

    -- If true, apple will be retreived automatically after reaching. If false,
    -- Retrieve() function must be called.
    self.autoRetrieve = false

    -- Configuration parameters END --
    ------------------------------------

    self.tickBusHandler = TickBus.CreateHandler(self)
    self.tickBusHandler:Connect()    

    self.InputNotificationBus = InputEventNotificationBus.Connect(self, InputEventNotificationId("manipulator_keyboard_control"))

    self.pid1 = PID.new(300.0, 0.0, 0.0, self.max_velocity['z']) 
    self.pid2 = PID.new(200.0, 0.0, 0.0, self.max_velocity['x']) 
    self.pid3 = PID.new(100.0, 0.0, 0.0, self.max_velocity['y'])
    self.pid4 = PID.new(100.0, 0.0, 0.0, self.max_velocity['y'])
    self.gravityThreshold = 0.0

    -- Target translation of each segment. Relative to current position
    self.internalError = {nil, nil, nil, nil}

    -- Target position (x,y,z) in internal reference
    self.targetPos = Vector3(0.0, 0.0, 0.0)

    -- Target
    self.requestWorldPos = Vector3(0.0, 0.0, 0.0)
    self.restPos = Vector3(0.0, -0.1, 0.0)

    self.pickingState = State.PREPARED
    self.lastPickingState = nil

    self.baseZeroTime = 0.0

    self.retrieveTime = 0.0

    self.startupWait = 0.1 --[s]

    self.noseZeroTimeout = 0.5
    self.noseZeroTime = 0.0
    self.manipulatorRequestBus = ManipulatorRequestBus.Connect(self, self.entityId)       
end

function manipulator_control:_getFPS()
    -- TODO Apply some kind of filter to smooth this vale
    -- TODO Can we do it in a different way? Obtain it form a bus?
    return 1.0 / self.deltaTime
end

function manipulator_control:getSegmentPos(entityid)
    return TransformBus.Event.GetLocalTranslation(entityid)
end

function manipulator_control:_setSegmentPos(entityid, pid, target_pos, axis1, print_debug)

    local err = target_pos
    local force = pid:Calculate(self.deltaTime, err)

    local impulse = force * self.deltaTime

    local force_vector = Vector3(0.0, 0.0, impulse)

    local tm = TransformBus.Event.GetWorldTM(entityid)

    force_vector = Transform.TransformVector(tm, force_vector)

    -- If we want to use velocity
    RigidBodyRequestBus.Event.SetLinearVelocity(entityid, force_vector)   
    -- If we want to use force impulse
    --RigidBodyRequestBus.Event.ApplyLinearImpulse(entityid, force_vector)   

    if print_debug then
        Debug.Log('error: '..string.format("%1.3f",target_pos)..'  impulse: '..string.format("%1.3f",force))
    end
end

function manipulator_control:_setPosition()
    self:_setSegmentPos(self.Properties.segment1, self.pid1, self.internalError[1], 'z', false)
    self:_setSegmentPos(self.Properties.segment2, self.pid2, self.internalError[2], 'x', false)
    self:_setSegmentPos(self.Properties.segment3, self.pid3, self.internalError[3], 'y', false)
    self:_setSegmentPos(self.Properties.segment4, self.pid4, self.internalError[4], 'z', false)
end


function manipulator_control:_getWorldPosition(target_position)
    local tm = TransformBus.Event.GetWorldTM(self.Properties.gripper)
    local gripper_pos = Transform.GetTranslation(tm)
    Transform.Invert(tm)

    local pos = Transform.TransformVector(tm, target_position - gripper_pos)

    TransformBus.Event.SetLocalTranslation(self.Properties.debug, pos)

    return pos
end

function manipulator_control:_setInternalState()
    self.internalError[1] = -self.targetPos['z'] + self.gravityThreshold 
    self.internalError[2] = self.targetPos['x'] 
    self.internalError[3] = self.targetPos['y'] 
    self.internalError[4] = self.targetPos['y']
end

function manipulator_control:_isNearZero(segment)
    if math.abs(self.internalError[segment]) < self.nearZeroThresholds[segment] then
        return true
    else
        return false
    end
end


function manipulator_control:_printDebugInfo()
    if self.lastPickingState ~= self.pickingState then
        self.lastPickingState = self.pickingState

        if self.printState then
            if self.pickingState == State.IDLE then
                txt = 'IDLE'

            elseif self.pickingState == State.PREPARED then
                txt = 'PREPARED'

            elseif self.pickingState == State.PICKING_BASE then
                txt = 'PICKING_BASE'

            elseif self.pickingState == State.PICKING_NOSE then
                txt = 'PICKING_NOSE'

            elseif self.pickingState == State.RETRIEVING_NOSE then
                txt = 'RETRIEVING_NOSE'

            elseif self.pickingState == State.RETRIEVING_BASE then
                txt = 'RETRIEVING_BASE'

            elseif self.pickingState == State.INVALID then
                txt = 'INVALID'
            end

            Debug.Log("Changed state to: ["..txt.."]")
        end
    end
end

function manipulator_control:_orchestrator()

    ----------------------------------------------------------

    if self.pickingState == State.INVALID then
        self.requestWorldPos = TransformBus.Event.GetWorldTranslation(self.Properties.rest)
        self.targetPos = self:_getWorldPosition(self.requestWorldPos)
    end

    ----------------------------------------------------------

    if self.pickingState == State.RETRIEVING_BASE then
        self.requestWorldPos = TransformBus.Event.GetWorldTranslation(self.Properties.rest)
        self.targetPos = self:_getWorldPosition(self.requestWorldPos)
        self:_setInternalState()
        if (self:_isNearZero(1) and self:_isNearZero(2) and self:_isNearZero(3) and self:_isNearZero(4)) then
            self.retrieveTime = self.retrieveTime + self.deltaTime
            if self.retrieveTime > self.retrieveWait then
                self.retrieveTime = 0.0
                self.pickingState = State.PREPARED
            end
        end
    end

    ----------------------------------------------------------

    if self.pickingState == State.RETRIEVING_NOSE then

        self.requestWorldPos = TransformBus.Event.GetWorldTranslation(self.Properties.rest)
        self.targetPos['y'] = self:_getWorldPosition(self.requestWorldPos)['y']

        if (self:_isNearZero(3) and self:_isNearZero(4)) then
            self.noseZeroTime = self.noseZeroTime + self.deltaTime
            if self.noseZeroTime > self.noseZeroTimeout then
                self.noseZeroTime = 0.0
                self.pickingState = State.RETRIEVING_BASE
            end
        end
    end

    ----------------------------------------------------------

    if self.pickingState == State.PREPARED then
        self.requestWorldPos = TransformBus.Event.GetWorldTranslation(self.Properties.rest)
        self.targetPos = self:_getWorldPosition(self.requestWorldPos)
    end

    ----------------------------------------------------------

    if self.pickingState == State.PICKING_NOSE then

        self.targetPos = self:_getWorldPosition(self.requestWorldPos)

        if (self:_isNearZero(3) and self:_isNearZero(4)) then
            self.noseZeroTime = self.noseZeroTime + self.deltaTime
            if self.noseZeroTime > self.noseZeroTimeout then
                self.noseZeroTime = 0.0
                self.pickingTime = 0.0
                if self.autoRetrieve then
                    self:Retrieve()
                end
            end
        end
    end

    ----------------------------------------------------------

    if self.pickingState == State.PICKING_BASE then
        self.targetPos = self:_getWorldPosition(self.requestWorldPos)
        self.targetPos['y'] = self.restPos['y']

        if (self:_isNearZero(1) and self:_isNearZero(2)) then
            self.baseZeroTime = self.baseZeroTime + self.deltaTime
            if self.baseZeroTime > self.baseZeroTimeout then
                self.baseZeroTime = 0.0
                self.pickingState = State.PICKING_NOSE
            end
        end
    end

    ----------------------------------------------------------

    self:_setInternalState()
    self:_setPosition()
    self:_printDebugInfo()
end

function manipulator_control:PickApple(apple_pos)
    --if self.pickingState == state.PREPARED then

    self.requestWorldPos = apple_pos
    self.pickingState = State.PICKING_BASE

    Debug.Log("Picking apple at "..tostring(apple_pos))

    --end
end


function manipulator_control:Retrieve()
    self.pickingState = State.RETRIEVING_NOSE
    --Debug.Log("Retracting to "..tostring(rest_pos))
end

function manipulator_control:GetStatus()
    return self.pickingState
end



function manipulator_control:OnPressed(value)
    -- Keypress actions

    if self.enableKeyboardInput then

        if value == 1.0 then
            local currentApple = self.Properties.apple1
            if currentApple ~= nil then
                self:PickApple(TransformBus.Event.GetWorldTranslation(currentApple))
            end
        end

        if value == 2.0 then
            local currentApple = self.Properties.apple2
            if currentApple ~= nil then
                self:PickApple(TransformBus.Event.GetWorldTranslation(currentApple))
            end
        end 
    end
    
end

function manipulator_control:OnDeactivate()
      self.manipulatorRequestBus.Disconnect()

     -- Deactivation Code
end

-- This callback is called every frame by the tick bus after this entity activates
function manipulator_control:OnTick(deltaTime, timePoint)

    self.deltaTime = deltaTime

    if self.Properties.segment1~=nil then
        self:_orchestrator()
    end
end

return manipulator_control
